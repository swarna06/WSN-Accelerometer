/*
 * protocol.c
 *
 *  Created on: Nov 1, 2018
 *      Author: alvaro
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

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
static uint32_t Ptc_Calculate_RAT_Start_Time(uint32_t rtc_start_of_next_event, int rat_time_offset);
static void Ptc_Adjust_Local_Clock(uint32_t rtc_start_of_curr_frame, uint32_t rat_rx_timestamp);
static void Ptc_Request_Beacon_Tx(uint32_t rat_start_of_tx);
static bool Ptc_Process_Beacon();
static void Ptc_Request_Data_Pkt_Tx(uint32_t rat_start_of_tx);
static void Ptc_Process_Data_Pkt();
static size_t Ptc_Add_Field_To_Payload(uint8_t** payload_p, void* data_p, size_t data_len);
static void Ptc_Get_Field_From_Payload(uint8_t** payload_p, void* data_p, size_t data_len);

#ifdef PTC_START_OF_FRAME_OUT
void Ptc_RTC_Isr()
{
    // Toggle bit at the start of each frame
    Brd_Led_Toggle(BRD_RTC_OUT_PIN);
    SysCtrlAonSync();
    AONRTCEventClear(AON_RTC_CH1);
    IntPendClear(INT_AON_RTC_COMB);
}
#endif // #ifdef PTC_START_OF_FRAME_OUT

void Ptc_Init()
{
    // Set bits in 'absent_nodes' for all nodes in the network
    ptc.absent_nodes = (PTC_SENSOR_NODES_NUM >= 32) ? (uint32_t)-1 : (((uint32_t)1 << PTC_SENSOR_NODES_NUM) - 1);

    // Note: The device ID determines the role of a device; sink: 0, sensor: other
    // The device id is the LSB of the secondary BLE access address
    // The secondary BLE address is written using the TI UniFlash tool
    uint32_t secondary_ble_addr_l = HWREG(CCFG_BASE + CCFG_O_IEEE_BLE_0);
    ptc.dev_id = (uint8_t)secondary_ble_addr_l;

    #ifdef PTC_VERBOSE
    Log_Val_Uint32("dev_id: ", ptc.dev_id);
    #endif // #ifdef PTC_VERBOSE

    // Read and store the BLE primary access address - TODO move this to a packet buffer
    ptc.ble_access_l = HWREG(FCFG1_BASE + FCFG1_O_MAC_BLE_0);
    ptc.ble_access_h = HWREG(FCFG1_BASE + FCFG1_O_MAC_BLE_1);

    // Initialize random seeds using the True Random Number Generator (TRNG)
    Ptc_Init_Random_Seeds();

    // Set transmission power, PHY mode and frequency channel
    ptc.tx_power = PTC_DEFAULT_TX_POW;
    ptc.phy_mode = PTC_DEFAULT_PHY_MODE;
    ptc.channel = PTC_DEFAULT_CHANNEL;

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
    ptc.next_state = PTC_S_IDLE;
    ptc.err_count = PTC_MAX_ERR_NUM;

    #ifdef PTC_START_OF_FRAME_OUT
    // Configure an RTC CH in compare mode and enable interrupt
    // to toggle a pin at the start of each frame
    AONRTCCompareValueSet(AON_RTC_CH1, 0);
    AONRTCCombinedEventConfig(AON_RTC_CH1);
    AONRTCChannelEnable(AON_RTC_CH1);

    IntRegister(INT_AON_RTC_COMB, Ptc_RTC_Isr);
    IntPendClear(INT_AON_RTC_COMB);
    IntEnable(INT_AON_RTC_COMB);

    IOCPinTypeGpioOutput(BRD_RTC_OUT_PIN);
    #endif // #ifdef PTC_START_OF_FRAME_OUT
}

void Ptc_Process()
{
    switch (ptc.state)
    {
    // Common states for both device roles
    case PTC_S_IDLE:
        break;

    case PTC_S_WAIT_RF_CORE_INIT:
        if (Ptc_Dev_Is_Sink_Node())
            ptc.state = PTC_S_WAIT_START_OF_FRAME;
        else
        {
            Rfc_Synchronize_RAT();
            ptc.state = PTC_S_SCHEDULE_FIRST_BEACON_RX;
        }
        break;

    case PTC_S_WAIT_RF_CORE_WAKEUP:
        Rfc_Synchronize_RAT();
        ptc.state = ptc.next_state;
        break;

    case PTC_S_SCHEDULE_FIRST_BEACON_RX:
        // Start reception
        Rfc_BLE5_Scanner(0, PTC_FRAME_TIME_SEC*1000*1000 + 10*1000); // TODO define timeout
        ptc.state = PTC_S_WAIT_FIRST_BEACON;
        break;

    case PTC_S_WAIT_FIRST_BEACON:
        Rfc_BLE5_Get_Scanner_Result(&ptc.rx_result);
        if ((ptc.rx_result.err_flags == 0) && Ptc_Process_Beacon()) // no errors
            ptc.state = PTC_S_WAIT_START_OF_SLOT;
        else
        {
            Pma_MCU_Sleep(Tm_Get_RTC_Time() + PTC_RTC_FRAME_TIME);

            ptc.state = PTC_S_WAIT_RF_CORE_WAKEUP;
            ptc.next_state = PTC_S_SCHEDULE_FIRST_BEACON_RX;
        }
        break;

    case PTC_S_WAIT_START_OF_FRAME:
    {
        // Calculate time of start of next frame and next slot
        ptc.start_of_next_frame += PTC_RTC_FRAME_TIME;

        if (Ptc_Dev_Is_Sink_Node())
        {
            ptc.start_of_next_slot = ptc.start_of_next_frame + PTC_RTC_SLOT_TIME;
            ptc.dev_index = 1;
        }
        else
            ptc.start_of_next_slot = ptc.start_of_next_frame + (PTC_RTC_SLOT_TIME * ptc.dev_id);

        // Calculate wake up time and go to sleep
        uint32_t wakeup_time = ptc.start_of_next_frame - PTC_RTC_TOTAL_WAKEUP_TIME;
        Pma_MCU_Sleep(wakeup_time);

        #ifdef PTC_START_OF_FRAME_OUT
        // Set RTC compare value to match the start of next frame
        AONRTCEventClear(AON_RTC_CH1);
        AONRTCCompareValueSet(AON_RTC_CH1, ptc.start_of_next_frame);
        #endif // #ifdef PTC_START_OF_FRAME_OUT

        #ifdef PTC_VERBOSE
        Log_Val_Uint32("rtc_SoNF: ", ptc.start_of_next_frame);
        #endif // #ifdef PTC_VERBOSE

        ptc.state = PTC_S_WAIT_RF_CORE_WAKEUP;
        ptc.next_state = PTC_S_SCHEDULE_BEACON_RADIO_OP;
    }
    break;

    case PTC_S_SCHEDULE_BEACON_RADIO_OP:
    {
        // Calculate the start time of packet reception/transmission and request operation to the RF core
        // An offset is used to compensate for (measured) latencies of the RF core in the start of the reception/transmission
        int32_t offset = Ptc_Dev_Is_Sink_Node() ? PTC_RAT_TX_START_OFFSET : PTC_RAT_RX_START_OFFSET;
        uint32_t rat_start_time = Ptc_Calculate_RAT_Start_Time(ptc.start_of_next_frame, offset);

        // Request radio operation to the RF core
        // If device is sink -> transmit; if device is sensor receive
        if (Ptc_Dev_Is_Sink_Node())
        {
            Ptc_Request_Beacon_Tx(rat_start_time);
            ptc.state = PTC_S_WAIT_TIMEOUT;
            ptc.next_state = PTC_S_WAIT_START_OF_SLOT;
        }
        else
        {
            Rfc_BLE5_Scanner(rat_start_time, PTC_RX_TIMEOUT_USEC + PTC_OFFSET_RX_TOUT_USEC);
            ptc.state = PTC_S_WAIT_PKT_RECEPTION;
        };

        Tm_Start_Timeout(TM_TOUT_PTC_ID, 30); // TODO remove
    }
    break;

    case PTC_S_WAIT_START_OF_SLOT:
    {
        // Calculate wake up time and go to sleep
        uint32_t wakeup_time = ptc.start_of_next_slot - PTC_RTC_TOTAL_WAKEUP_TIME;
        Pma_MCU_Sleep(wakeup_time);

        ptc.state = PTC_S_WAIT_RF_CORE_WAKEUP;
        ptc.next_state = PTC_S_SCHEDULE_SLOT_RADIO_OP;
    }
    break;

    case PTC_S_SCHEDULE_SLOT_RADIO_OP:
    {
        // Calculate the start time of packet reception/transmission and request operation to the RF core
        // An offset is used to compensate for (measured) latencies of the RF core in the start of the reception/transmission
        // IMPORTANT: Notice the negation in the !Ptc_Dev_Is_Sink_Node()? conditional; roles are exchanged with respect to the start of frame
        int32_t offset = !Ptc_Dev_Is_Sink_Node() ? PTC_RAT_TX_START_OFFSET : PTC_RAT_RX_START_OFFSET;
        uint32_t rat_start_time = Ptc_Calculate_RAT_Start_Time(ptc.start_of_next_slot, offset);

        // Request radio operation to the RF core
        // If device is sink -> receive; if device is sensor transmit
        if (Ptc_Dev_Is_Sink_Node())
        {
            Rfc_BLE5_Scanner(rat_start_time, PTC_RX_TIMEOUT_USEC + PTC_OFFSET_RX_TOUT_USEC);
            ptc.state = PTC_S_WAIT_PKT_RECEPTION;
        }
        else
        {
            Ptc_Request_Data_Pkt_Tx(rat_start_time);
            ptc.state = PTC_S_WAIT_TIMEOUT;
            ptc.next_state = PTC_S_WAIT_START_OF_FRAME;
        };

        Tm_Start_Timeout(TM_TOUT_PTC_ID, 30); // TODO remove
    }
    break;

    case PTC_S_WAIT_PKT_RECEPTION:
        Rfc_BLE5_Get_Scanner_Result(&ptc.rx_result);
        if (Ptc_Dev_Is_Sink_Node())
        {
            if (ptc.rx_result.err_flags == 0)
                Ptc_Process_Data_Pkt();
            else
            {
//                Log_Val_Hex32("Rx error: ", ptc.rx_result.err_flags);
                Log_String_Literal(""); Log_Value_Hex(ptc.start_of_next_frame);
                Log_String_Literal(", "); Log_Value_Hex(ptc.dev_index);
                Log_String_Literal(", "); Log_Value_Hex(ptc.rx_result.err_flags << 1);
                Log_Line(""); // new line
            }

            ptc.dev_index++;
            if (ptc.dev_index > PTC_SENSOR_NODE_NUM)
                ptc.next_state = PTC_S_WAIT_START_OF_FRAME;
            else
            {
                ptc.start_of_next_slot += PTC_RTC_SLOT_TIME;
                ptc.next_state = PTC_S_WAIT_START_OF_SLOT;
            }
        }
        else
        {
            if (ptc.rx_result.err_flags == 0 && Ptc_Process_Beacon())
            {
                ptc.flags |= PTC_F_BEACON_RXED;
                ptc.err_count = PTC_MAX_ERR_NUM;
            }
            else
            {
                ptc.flags &= ~PTC_F_BEACON_RXED;
                ptc.err_count--;
                if (ptc.err_count == 0) // max consecutive errors reached ?
                {
                    ptc.flags &= ~PTC_F_IN_SYNC; // out of sync
                    Rfc_BLE5_Set_PHY_Mode(PTC_DEFAULT_PHY_MODE);
                    Rfc_BLE5_Set_Channel(PTC_DEFAULT_CHANNEL);

                    // If we go to the PTC_S_WAIT_FIRST_BEACON we will sleep immediately
                    ptc.state = PTC_S_WAIT_FIRST_BEACON;
                    break;
                }
            }
            ptc.next_state = PTC_S_WAIT_START_OF_SLOT;
        }

        ptc.state = PTC_S_WAIT_TIMEOUT;
        break;

    case PTC_S_WAIT_TIMEOUT:

//        if (!Ptc_Dev_Is_Sink_Node())
//            ptc.state = ptc.next_state;
//        else if (Tm_Timeout_Completed(TM_TOUT_PTC_ID))
        if (Tm_Timeout_Completed(TM_TOUT_PTC_ID))
            ptc.state = ptc.next_state;

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
//    // TODO
//    while (1) // Stop execution; flush log buffer
//    {
//        Log_Process();
//    }
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

static uint32_t Ptc_Calculate_RAT_Start_Time(uint32_t rtc_start_of_next_event, int rat_time_offset)
{
    // Calculate start time of radio operation //
    uint32_t rtc_current_time, rtc_ticks_to_event;
    uint32_t rat_current_time, rat_ticks_to_event, rat_start_time;

    // 1. Synchronize with RTC; wait for the start of the next RTC period (up to ~30 us)
    Tm_Synch_With_RTC();

    // 2. Get current time (RTC and RAT)
    rat_current_time = Rfc_Get_RAT_Time();
    rtc_current_time = Tm_Get_RTC_Time();

    // 3. Calculate the number of RTC ticks left for the start of transmission
    rtc_ticks_to_event = rtc_start_of_next_event - rtc_current_time;

    // 4. Convert RTC ticks to RAT ticks (apply measured offset ~160 microseconds)
    //    Convert RTC ticks into RAT ticks: 1 RTC tick = 4e6/32768 RAT ticks = 15625/128 RAT ticks
    //    One RTC clock cycle is equal to 2 units of the RTC counter
    rat_ticks_to_event = ((rtc_ticks_to_event / TM_RTC_TICKS_PER_CYCLE)*15625) / 128;
    rat_ticks_to_event += rat_time_offset;

    // 5. Calculate absolute time of start of transmission
    rat_start_time = rat_current_time + rat_ticks_to_event;

    return rat_start_time;
}

static void Ptc_Adjust_Local_Clock(uint32_t rtc_start_of_curr_frame, uint32_t rat_rx_timestamp)
{
    // Wait for start of next RTC cycle and get current time (RAT)
    Tm_Synch_With_RTC();
    uint32_t rat_current_time = Rfc_Get_RAT_Time();

    // Calculate the number of RAT ticks since the reception of the beacon
    // Convert this value to RTC ticks (1 RTC tick = 4e6/32768 RAT ticks = 15625/128 RAT ticks)
    uint32_t rat_time_since_rx = rat_current_time - rat_rx_timestamp;
    uint32_t rtc_time_since_rx = (rat_time_since_rx * 128 * TM_RTC_TICKS_PER_CYCLE) / 15625;
    rtc_time_since_rx += 6; // xxx trimming

    // Calculate new value of the RTC counter
    uint32_t rtc_new_time_sec = (rtc_start_of_curr_frame + rtc_time_since_rx) >> 16;
    uint32_t rtc_new_time_subsec = ((rtc_start_of_curr_frame + rtc_time_since_rx) & 0x0000FFFF) << 16;

    // Update the RTC counter
    HWREG(AON_RTC_BASE + AON_RTC_O_SEC) = rtc_new_time_sec;
    HWREG(AON_RTC_BASE + AON_RTC_O_SUBSEC) = rtc_new_time_subsec;

    // Adjust timing module after synchronization
    Tm_Adjust_Time();
}

static void Ptc_Request_Beacon_Tx(uint32_t rat_start_of_tx)
{
    size_t payload_len = 0;
    uint8_t* payload_p = ptc.tx_buf;

    // Include RTC time stamp in the payload
    // The time stamp should correspond to the transmission absolute time (RTC)
    payload_len += Ptc_Add_Field_To_Payload(&payload_p, Ptc_Payload_Field(ptc.dev_id));
    payload_len += Ptc_Add_Field_To_Payload(&payload_p, Ptc_Payload_Field(ptc.start_of_next_frame));
    payload_len += Ptc_Add_Field_To_Payload(&payload_p, Ptc_Payload_Field(ptc.channel)); // FIXME remove
    payload_len += Ptc_Add_Field_To_Payload(&payload_p, Ptc_Payload_Field(ptc.tx_power));
    payload_len += Ptc_Add_Field_To_Payload(&payload_p, Ptc_Payload_Field(ptc.phy_mode));

    ptc.tx_param.buf = ptc.tx_buf;
    ptc.tx_param.len = payload_len;
    ptc.tx_param.rat_start_time = rat_start_of_tx;
    Rfc_BLE5_Adv_Aux(&ptc.tx_param);
}

static bool Ptc_Process_Beacon()
{
    uint8_t* payload_p = ptc.rx_result.buf;

    uint8_t dev_id;
    uint32_t rtc_start_of_curr_frame;

    Ptc_Get_Field_From_Payload(&payload_p, Ptc_Payload_Field(dev_id));
    if (dev_id != PTC_SINK_NODE_DEV_ID)
        return false;

    Ptc_Get_Field_From_Payload(&payload_p, Ptc_Payload_Field(rtc_start_of_curr_frame));

    // Update local clock
    uint32_t rat_rx_timestamp = ptc.rx_result.rat_timestamp;
    Ptc_Adjust_Local_Clock(rtc_start_of_curr_frame, rat_rx_timestamp);

    if (!(ptc.flags & PTC_F_IN_SYNC))
    {
        // Calculate start of next slot and frame
        ptc.start_of_next_slot = rtc_start_of_curr_frame + (PTC_RTC_SLOT_TIME * ptc.dev_id);
        ptc.start_of_next_frame = rtc_start_of_curr_frame + PTC_RTC_FRAME_TIME;
        ptc.flags |= PTC_F_IN_SYNC;
    }

    #ifdef PTC_VERBOSE
    size_t payload_len = ptc.rx_result.payload_len;
    Log_String_Literal("Beacon:");
    Log_String_Literal(" payload_len: "); Log_Value_Uint(payload_len);
    Log_String_Literal(" dev_id: "); Log_Value_Hex(dev_id);
    Log_String_Literal(" rtc_time: "); Log_Value_Hex(rtc_start_of_curr_frame);
    Log_Line(""); // new line
    #endif // #ifdef PTC_VERBOSE

    return true;
}

static void Ptc_Request_Data_Pkt_Tx(uint32_t rat_start_of_tx)
{
    size_t payload_len = 0;
    uint8_t* payload_p = ptc.tx_buf;

    uint8_t ack = ptc.flags & PTC_F_BEACON_RXED ? 1 : 0;

    // Include RTC time stamp in the payload
    // The time stamp should correspond to the transmission absolute time (RTC)
    payload_len += Ptc_Add_Field_To_Payload(&payload_p, Ptc_Payload_Field(ptc.dev_id));
    payload_len += Ptc_Add_Field_To_Payload(&payload_p, Ptc_Payload_Field(ack));

    ptc.tx_param.buf = ptc.tx_buf;
    ptc.tx_param.len = payload_len;
    ptc.tx_param.rat_start_time = rat_start_of_tx;
    Rfc_BLE5_Adv_Aux(&ptc.tx_param);
}

static void Ptc_Process_Data_Pkt()
{
    uint8_t* payload_p = ptc.rx_result.buf;
//    size_t payload_len = ptc.rx_result.payload_len;

    uint8_t dev_id;
    uint8_t ack;

    Ptc_Get_Field_From_Payload(&payload_p, Ptc_Payload_Field(dev_id));
    Ptc_Get_Field_From_Payload(&payload_p, Ptc_Payload_Field(ack));

//    #ifdef PTC_VERBOSE
    Log_String_Literal(""); Log_Value_Hex(ptc.start_of_next_frame);
    Log_String_Literal(", "); Log_Value_Hex(dev_id);
    Log_String_Literal(", "); Log_Value_Hex(ack);
//    Log_String_Literal(" payload_len: "); Log_Value_Uint(payload_len);
//    Log_String_Literal(" dev_id: "); Log_Value_Hex(dev_id);
//    Log_String_Literal(" ack: "); Log_Value_Hex(ack);
    Log_Line(""); // new line
//    #endif // #ifdef PTC_VERBOSE
}

static size_t Ptc_Add_Field_To_Payload(uint8_t** payload_p, void* data_p, size_t data_len)
{
    uint8_t* _data_p = (uint8_t*)data_p;

    for (size_t n = 0; n < data_len; n++, _data_p++, (*payload_p)++)
        *(*payload_p) = *_data_p;

    return data_len;
}

static void Ptc_Get_Field_From_Payload(uint8_t** payload_p, void* data_p, size_t data_len)
{
    uint8_t* _data_p = (uint8_t*)data_p;

    for (size_t n = 0; n < data_len; n++, _data_p++, (*payload_p)++)
        *_data_p = *(*payload_p);
}

// ********************************
// Test functions
// ********************************
