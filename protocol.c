/*
 * protocol.c
 *
 *  Created on: Nov 1, 2018
 *      Author: alvaro
 */

#include <stdint.h>
#include <stdbool.h>

#include <driverlib/prcm.h>
#include <driverlib/ccfgread.h>
#include <inc/hw_ccfg.h>
#include <inc/hw_fcfg1.h>
#include <driverlib/trng.h>

#include <inc/hw_rfc_dbell.h>
#include <inc/hw_aon_rtc.h>
#include <driverlib/ioc.h>
#include <driverlib/event.h>
#include <driverlib/interrupt.h>
#include <driverlib/sys_ctrl.h>

#include "protocol.h"
#include "rf_core.h"
#include "power_management.h"
#include "timing.h"

#include "printf.h"
#include "profiling.h"
#include "log.h"
#include "misc.h"
#include "board.h"

ptc_control_t ptc;

// Auxiliary functions
static bool Ptc_Init_Random_Seeds();

void Ptc_RTC_Isr()
{
    Brd_Led_Toggle(BRD_RTC_OUT_PIN);
    SysCtrlAonSync();
    AONRTCEventClear(AON_RTC_CH1);
    IntPendClear(INT_AON_RTC_COMB);
}

void Ptc_Init()
{
    // Set bits in 'absent_nodes' for all nodes in the network
    ptc.absent_nodes = (PTC_SENSOR_NODES_NUM >= 32) ? (uint32_t)-1 : (((uint32_t)1 << PTC_SENSOR_NODES_NUM) - 1);

    // Note: The device ID determines the role of a device; sink: 0, sensor: other
    // The device id is the LSB of the secondary BLE access address
    // The secondary BLE address is written using the TI UniFlash tool
    uint32_t secondary_ble_addr_l = HWREG(CCFG_BASE + CCFG_O_IEEE_BLE_0);
    ptc.dev_id = (uint8_t)secondary_ble_addr_l;
    Log_Val_Uint32("dev_id: ", ptc.dev_id);

    // Read and store the BLE primary access address - TODO move this to a packet buffer
    ptc.ble_access_l = HWREG(FCFG1_BASE + FCFG1_O_MAC_BLE_0);
    ptc.ble_access_h = HWREG(FCFG1_BASE + FCFG1_O_MAC_BLE_1);

    // Initialize random seeds using the True Random Number Generator (TRNG)
    Ptc_Init_Random_Seeds();

    // Set transmission power, PHY mode and frequency channel
    ptc.tx_power = RFC_TX_POW_0dBm;
    ptc.phy_mode = RFC_PHY_MODE_125KBPS;
    ptc.channel = 37; // advertising channel

    Rfc_Set_Tx_Power(ptc.tx_power);
    Rfc_BLE5_Set_PHY_Mode(ptc.phy_mode);
    Rfc_BLE5_Set_Channel(ptc.channel);

    // Initialize transmission and reception structures
    ptc.tx_param.buf = NULL; // empty packet
    ptc.tx_param.len = 0;
    ptc.tx_param.rat_start_time = 0;

    ptc.rx_result.buf = ptc.rx_buf;
    ptc.rx_result.buf_len = PTC_RXTX_BUF_LEN;

    // Set start time of the first frame
    ptc.start_of_next_frame = 0;

    // Initialize FSM variables
    ptc.flags = 0;
    ptc.state = PTC_S_WAIT_RF_CORE_INIT;

    if (Ptc_Dev_Is_Sink_Node())
    {
        Ptc_Process = Ptc_Process_Sink_Init;
        ptc.next_state = PTC_S_IDLE;
    }

    // TODO remove the following lines
    AONRTCCompareValueSet(AON_RTC_CH1, 0);
    AONRTCCombinedEventConfig(AON_RTC_CH1);
    AONRTCChannelEnable(AON_RTC_CH1);

    IntRegister(INT_AON_RTC_COMB, Ptc_RTC_Isr);
    IntPendClear(INT_AON_RTC_COMB);
    IntEnable(INT_AON_RTC_COMB);

    IOCPinTypeGpioOutput(BRD_RTC_OUT_PIN);
}

void Ptc_Process_Sink_Init()
{
    switch (ptc.state)
    {
    // Common states for both device roles
    case PTC_S_IDLE:
        break;

    case PTC_S_WAIT_RF_CORE_INIT:
        ptc.state = PTC_S_WAIT_START_OF_FRAME;
        break;

    case PTC_S_WAIT_RF_CORE_WAKEUP:
        Rfc_Synchronize_RAT();
        ptc.state = ptc.next_state;
        break;

    case PTC_S_WAIT_START_OF_FRAME:
    {
        // Calculate wake up time and go to sleep
        ptc.start_of_next_frame += PTC_RTC_FRAME_TIME;
        uint32_t wakeup_time = ptc.start_of_next_frame - PTC_RTC_TOTAL_WAKEUP_TIME;

        Pma_MCU_Sleep(wakeup_time);
//        Pma_Dummy_MCU_Sleep(wakeup_time);

        AONRTCEventClear(AON_RTC_CH1); // TODO remove
        AONRTCCompareValueSet(AON_RTC_CH1, ptc.start_of_next_frame); // TODO remove

        Log_Val_Uint32("rtc_SoNF: ", ptc.start_of_next_frame);

        ptc.state = PTC_S_WAIT_RF_CORE_WAKEUP;
        ptc.next_state = PTC_S_SCHEDULE_BEACON_TX;
    }
    break;

    case PTC_S_SCHEDULE_BEACON_TX:
    {
        //
        // Calculate time of the start of transmission and request the RF core to send the packet
        //

        // 1. Synchronize with RTC; wait for the start of the next RTC period (up to ~30 us)
        Tm_Synch_With_RTC();

        // 2. Get current time (RTC and RAT)
        uint32_t rat_current_time = Rfc_Get_RAT_Time();
        uint32_t rtc_current_time = Tm_Get_RTC_Time();

        Log_Val_Uint32("rat_ct: ", rat_current_time);
        Log_Val_Uint32("rtc_ct: ", rtc_current_time);

        // 3. Calculate the number of RTC ticks left for the start of transmission
        uint32_t rtc_ticks_to_start_of_frame = ptc.start_of_next_frame - rtc_current_time;

        // 4. Convert RTC ticks into RAT ticks: 1 RTC tick = 4e6/32768 RAT ticks = 15625/128 RAT ticks
        //    One RTC clock cycle is equal to 2 units of the RTC counter
        uint32_t rat_ticks_to_start_of_frame = ((rtc_ticks_to_start_of_frame / TM_RTC_TICKS_PER_CYCLE)*15625) / 128;

        // 5. Round up (if modulus is greater than half of the divisor; > 0.5 clock periods)
        if ((rat_ticks_to_start_of_frame % 128) > (128/2)) // round up
            rat_ticks_to_start_of_frame++;

        // 6. Remove measured offset in the start of transmission (~160 microseconds)
        rat_ticks_to_start_of_frame -= PTC_RAT_TX_START_OFFSET;

        // 7. Calculate absolute time of start of transmission
        uint32_t rat_tx_start = rat_current_time + rat_ticks_to_start_of_frame;

        Log_Val_Uint32("rtc_TtSoF: ", rtc_ticks_to_start_of_frame);
        Log_Val_Uint32("rat_TtSoF: ", rat_ticks_to_start_of_frame);
        Log_Val_Uint32("rat_txs: ", rat_tx_start);

        // 8. Request the RF core the transmission of the beacon
        ptc.tx_param.buf = NULL;
        ptc.tx_param.rat_start_time = rat_tx_start;
        Rfc_BLE5_Adv_Aux(&ptc.tx_param);

        Tm_Start_Timeout(TM_TOUT_PTC_ID, 30);
        ptc.state = PTC_S_WAIT_TIMEOUT;
    }
    break;

    case PTC_S_WAIT_TIMEOUT:

        if (Tm_Timeout_Completed(TM_TOUT_PTC_ID))
        {
            if ( HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG) & (1 << (PTC_RAT_CH + 12)) )
            {
                Log_Val_Hex32("RFHWIFG: ", HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG));
            }

            ptc.state = PTC_S_WAIT_START_OF_FRAME;
        }

        break;

    default:
        assertion(!"Ptc_Process_Sink_Init: Unknown state!");
    }
}

inline uint8_t Ptc_Get_FSM_State()
{
    return ptc.state;
}

void Ptc_Handle_Error()
{
    // TODO
}

// ********************************
// Static functions
// ********************************

static bool Ptc_Init_Random_Seeds()
{
    //
    // Get random value from the True Random Number Generator (TRNG)
    // Note: TRNGConfigure() defines the entropy amount
    //
    bool power_domain_was_off = false;
    bool result = true;

    // Turn on power domain if not already on
    if (PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON)
    {
        PRCMPowerDomainOn(PRCM_DOMAIN_PERIPH);
        power_domain_was_off = true;
    }

    // Enable TRNG clock
    PRCMPeripheralRunEnable(PRCM_PERIPH_TRNG);
    PRCMLoadSet();
    while(!PRCMLoadGet());

    // Enable TRNG and get random value
    TRNGConfigure(1 << 6, 1 << 8, 1); // min samp num: 2^6, max samp num: 2^8, cycles per sample 16 - TODO: increase arguments to increase entropy
    TRNGEnable();
    uint32_t trng_status;
    while ((trng_status = TRNGStatusGet()) & TRNG_NEED_CLOCK) {}; // wait while TRNG is busy - FIXME could this deadlock ?

    // Store random seeds
    if (trng_status & TRNG_NUMBER_READY)
    {
        // Success; initialize random seeds
        uint32_t rand_val_l = TRNGNumberGet(TRNG_LOW_WORD);
        uint32_t rand_val_h = TRNGNumberGet(TRNG_HI_WORD);
        ptc.random_seeds[0] = (uint16_t)rand_val_l;
        ptc.random_seeds[1] = (uint16_t)(rand_val_l >> 16);
        ptc.random_seeds[2] = (uint16_t)rand_val_h;
        ptc.random_seeds[3] = (uint16_t)(rand_val_h >> 16);
    }
    else
    {
        // Error occurred; clear random seeds and return 'false'
        result = false;
        for (size_t n = 0; n < PTC_RAND_SEEDS_NUM; n++)
            ptc.random_seeds[n] = 0;
    }
    // Disable TRNG
    TRNGDisable();
    PRCMPeripheralRunDisable(PRCM_PERIPH_TRNG);
    PRCMLoadSet();
    while(!PRCMLoadGet());

    // If power domain was initially off, turn it back off
    if (power_domain_was_off == true)
    {
        PRCMPowerDomainOff(PRCM_DOMAIN_PERIPH);
        while (PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON); // FIXME could this deadlock ?
    }

    return result;
}

