/*
 * sync_test.c
 *
 *  Created on: Mar 23, 2019
 *      Author: alvaro
 */

#include <inc/hw_ccfg.h>
#include <driverlib/interrupt.h>
#include <driverlib/sys_ctrl.h>
#include <driverlib/ioc.h>

#include "sync_test.h"
#include "timing.h"
#include "radio.h"
#include "misc.h"
#include "power_management.h"
#include "board.h"
#include "configuration.h"
#include "log.h"
#include "profiling.h"

bool proceed = false;

static uint8_t subsec_inc_offset[] = {0, 8, 17, 25, 34, 42, 50,
                                      59, 67, 75, 84, 92, 101, 109,
                                      117, 126, 134, 143, 151, 159, 168};

static sts_control stc;

static void Sts_Sink_Process();
static void Sts_Sensor_Process();

static uint32_t Sts_Get_RTC_Time();
static bool Sts_Get_RAT_Int_Flag();
static void Sts_Clear_RAT_Int_Flag();
static void Sts_Compensate_Drift(int32_t offset_ticks);

#ifdef CFG_DEBUG_START_OF_FRAME_OUT
void Sts_RTC_Isr();
#endif  // #ifdef PTC_START_OF_FRAME_OUT

void Sts_Init()
{
    Rad_Enable_Radio_Event_Output();
    Rad_Set_Data_Rate(RAD_DATA_RATE_125KBPS);

    // Get device ID
    uint32_t secondary_ble_addr_l = HWREG(CCFG_BASE + CCFG_O_IEEE_BLE_0);
    stc.dev_id = (uint8_t)secondary_ble_addr_l;

    stc.flags = 0;

    if (stc.dev_id == 0)
    {
        stc.rtc_sync_time = Sts_Get_RTC_Time() + STS_SYNC_PERIOD_RTC;
        stc.rtc_wakeup_time = stc.rtc_sync_time - STS_WAKEUP_DELAY;

        Sts_Process = Sts_Sink_Process;
        stc.state = STS_S_WAIT_WAKEUP_TIME;
    }
    else
    {
        stc.rtc_sync_time = 0;
        stc.rtc_wakeup_time = 0;

        Sts_Process = Sts_Sensor_Process;

        Rad_Turn_On_Radio();
        stc.state = STS_S_WAIT_RADIO_STARTUP;
    }

    stc.tx_param.delayed_start = true;
    stc.tx_param.payload_p = stc.txrx_buf;
    stc.tx_param.payload_len = 0;

    stc.rx_param.delayed_start = false;
    stc.rx_param.dest_buf = stc.txrx_buf;
    stc.rx_param.dest_buf_len = RAD_MAX_PAYLOAD_LEN;
    stc.rx_param.start_time = 0;
    stc.rx_param.timeout_usec = 0;

    stc.samp_idx = 0;
    for (size_t i = 0; i < STS_SAMP_NUM; i++)
        stc.sync_err_samples[i] = 0;

    #ifdef CFG_DEBUG_START_OF_FRAME_OUT
    // Configure an RTC CH in compare mode and enable interrupt
    // to toggle a pin at the start of each frame
    AONRTCCompareValueSet(AON_RTC_CH1, 0);
    AONRTCCombinedEventConfig(AON_RTC_CH1);
    AONRTCChannelEnable(AON_RTC_CH1);

    IntRegister(INT_AON_RTC_COMB, Sts_RTC_Isr);
    IntPendClear(INT_AON_RTC_COMB);
    IntEnable(INT_AON_RTC_COMB);

    IOCPinTypeGpioOutput(BRD_RTC_OUT_PIN);
    #endif // #ifdef PTC_START_OF_FRAME_OUT
}

inline uint8_t Sts_Get_FSM_State()
{
    return stc.state;
}

static void Sts_Sink_Process()
{
    switch (stc.state)
    {
    case STS_S_WAIT_WAKEUP_TIME:

        Pma_MCU_Sleep(stc.rtc_wakeup_time);

        // Set interrupt compare value to match the synchronization instant
        AONRTCCompareValueSet(AON_RTC_CH1, stc.rtc_sync_time);

        // Set PHY mode and turn on the radio
        Rad_Set_Data_Rate(RAD_DATA_RATE_125KBPS);
        Rad_Turn_On_Radio();

        stc.state = STS_S_WAIT_RADIO_STARTUP;

        proceed = false;
//        stc.state = STS_S_DUMMY;

        break;

    case STS_S_WAIT_RADIO_STARTUP:

        if (Rad_Radio_Is_On() == true)
        {
            // Schedule beacon transmission
            Tm_Synch_With_RTC();
            uint32_t rat_curr_time = Rad_Get_Radio_Time();

            uint32_t rtc_curr_time = Sts_Get_RTC_Time();
            uint32_t rtc_delta = stc.rtc_sync_time - rtc_curr_time;

            // TODO could RAT simply be increased without considering RTC time ?

            // Calculate start time
            volatile uint32_t rat_delta, tmp;
            tmp = rtc_delta * 15625;
            rat_delta = tmp / 256;
            if ((tmp % 256) > (256/2))
                rat_delta++;

            stc.tx_param.start_time = rat_curr_time + rat_delta;

            Log_Val_Uint32("delta:", rtc_delta);

            Rad_Set_RAT_Output();
            stc.state = STS_S_WAIT_SET_RAT_OUTPUT;
        }
        else if (Rad_Get_Err_Code())
        {
            assertion("Radio error!");
        }

        break;

    case STS_S_WAIT_SET_RAT_OUTPUT:

        if (Rad_Ready() == true)
        {
            Rad_Set_RAT_Cmp_Val(stc.tx_param.start_time, NULL);
            stc.state = STS_S_WAIT_SET_RAT_CMP_VAL;
        }
        else if (Rad_Get_Err_Code())
        {
            assertion("Radio error!");
        }

        break;

    case STS_S_WAIT_SET_RAT_CMP_VAL:

        if (Rad_Ready() == true)
        {
            stc.tx_param.start_time -= STS_RADIO_OP_DELAY;
//            stc.tx_param.payload_len = RAD_MAX_PAYLOAD_LEN;
            stc.tx_param.payload_len = 0;
            Rad_Transmit_Packet(&stc.tx_param);
            stc.state = STS_S_WAIT_PKT_TX;
        }
        else if (Rad_Get_Err_Code())
        {
            assertion("Radio error!");
        }

        break;

    case STS_S_WAIT_PKT_TX:

        if (Rad_Ready() == true)
        {
            Rad_Turn_Off_Radio();
            stc.state = STS_S_WAIT_RADIO_OFF;
        }
        else if (Rad_Get_Err_Code())
        {
            assertion("Radio error!");
        }

        break;

    case STS_S_WAIT_RADIO_OFF:

        if (Rad_Radio_Is_On() == false)
        {
            stc.state = STS_S_DUMMY;
        }
        else if (Rad_Get_Err_Code())
        {
            assertion("Radio error!");
        }

        break;

    case STS_S_DUMMY:

        if (proceed == true)
        {
            stc.rtc_sync_time += STS_SYNC_PERIOD_RTC;
            stc.rtc_wakeup_time = stc.rtc_sync_time - STS_WAKEUP_DELAY;

            stc.state = STS_S_WAIT_WAKEUP_TIME;
        }

        break;

    default:
        assertion(!"Invalid state");
        break;
    }
}


static void Sts_Sensor_Process()
{
    switch (stc.state)
    {
    case STS_S_WAIT_RADIO_STARTUP:

        if (Rad_Radio_Is_On() == true)
        {
            if (stc.flags & STS_F_1ST_PKT_RXED)
            {
                Rad_Set_RAT_Output();
                stc.state = STS_S_WAIT_SET_RAT_OUTPUT;
            }
            else
            {
                Rad_Receive_Packet(&stc.rx_param);
                stc.state = STS_S_WAIT_1ST_PKT;
            }
        }
        else if (Rad_Get_Err_Code())
        {
            assertion("Radio error!");
        }

        break;

    case STS_S_WAIT_1ST_PKT:

        if (Rad_Ready() == true)
        {
            if (stc.rx_param.error == RAD_RX_ERR_NONE)
            {
                Tm_Synch_With_RTC();
                uint32_t rat_curr_time = Rad_Get_Radio_Time();
                uint32_t rtc_curr_time = Sts_Get_RTC_Time();

                uint32_t rat_delta = Pfl_Delta_Time32(stc.rx_param.timestamp, rat_curr_time);
                volatile uint32_t rtc_delta, tmp;
                tmp = rat_delta * 256;
                rtc_delta = tmp / 15625;
                if ((tmp % 15625) > (15625/2))
                    rtc_delta++;

                stc.rtc_sync_time = (rtc_curr_time - rtc_delta) + STS_SYNC_PERIOD_RTC;
                stc.rtc_wakeup_time = stc.rtc_sync_time - STS_WAKEUP_DELAY;

                stc.rat_sync_time = stc.rx_param.timestamp + STS_SYNC_PERIOD_RAT;

                stc.flags |= STS_F_1ST_PKT_RXED;
                stc.state = STS_S_WAIT_WAKEUP_TIME;
            }
            else
            {
                // Print error and restart reception
                Log_Val_Uint32("rx_err:", stc.rx_param.error);
                Rad_Receive_Packet(&stc.rx_param);
            }
        }
        else if (Rad_Get_Err_Code())
        {
            assertion("Radio error!");
        }

        break;

    case STS_S_WAIT_WAKEUP_TIME:

        Pma_MCU_Sleep(stc.rtc_wakeup_time);

        // Set PHY mode and turn on the radio
        Rad_Set_Data_Rate(RAD_DATA_RATE_125KBPS);
        Rad_Turn_On_Radio();

        stc.state = STS_S_WAIT_RADIO_STARTUP;

        break;

    case STS_S_WAIT_SET_RAT_OUTPUT:

        if (Rad_Ready() == true)
        {
            Sts_Clear_RAT_Int_Flag();
            Rad_Set_RAT_Cmp_Val(stc.rat_sync_time, NULL);
            stc.state = STS_S_WAIT_SET_RAT_CMP_VAL;
        }
        else if (Rad_Get_Err_Code())
        {
            assertion("Radio error!");
        }

        break;

    case STS_S_WAIT_SET_RAT_CMP_VAL:

        if (Rad_Ready() == true)
        {
            Rad_Receive_Packet(&stc.rx_param);
            stc.state = STS_S_WAIT_PKT_RX;
        }
        else if (Rad_Get_Err_Code())
        {
            assertion("Radio error!");
        }

        break;

    case STS_S_WAIT_PKT_RX:

        if (Rad_Ready() == true)
        {
            if (stc.rx_param.error == RAD_RX_ERR_NONE)
            {
                Tm_Synch_With_RTC();
                uint32_t rat_curr_time = Rad_Get_Radio_Time();
                uint32_t rtc_curr_time = Sts_Get_RTC_Time();

                uint32_t rat_delta = Pfl_Delta_Time32(stc.rx_param.timestamp, rat_curr_time);
                volatile uint32_t rtc_delta, tmp;
                tmp = rat_delta * 256;
                rtc_delta = tmp / 15625;
                if ((tmp % 15625) > (15625/2))
                    rtc_delta++;

                stc.rtc_sync_time = (rtc_curr_time - rtc_delta) + STS_SYNC_PERIOD_RTC;
                stc.rtc_wakeup_time = stc.rtc_sync_time - STS_WAKEUP_DELAY;

                int32_t sync_err_ticks = stc.rx_param.timestamp - stc.rat_sync_time;
//                Log_Val_Int32("sync_err:", sync_err_ticks);
                stc.rat_sync_time = stc.rx_param.timestamp + STS_SYNC_PERIOD_RAT;

                Sts_Compensate_Drift(sync_err_ticks);

//                // Adjust offset
//                uint32_t new_rate = 8388583; // -3 ppm
////                uint32_t new_rate = 8388633; // +3 ppm
//                uint16_t new_rate_0 = 0x0000FFFF & new_rate;
//                uint16_t new_rate_1 = (0x00FF0000 & new_rate) >> 16;
//
//                HWREG(AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINC0) = new_rate_0;
//                HWREG(AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINC1) = new_rate_1;
//                HWREG(AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINCCTL) = 0x01;
            }
            else
            {
                stc.rtc_sync_time += STS_SYNC_PERIOD_RTC;
                stc.rtc_wakeup_time = stc.rtc_sync_time - STS_WAKEUP_DELAY;
                stc.rat_sync_time += STS_SYNC_PERIOD_RAT;

                Log_Line("nok");
                Log_Val_Uint32("rx_err:", stc.rx_param.error);
            }

            Rad_Turn_Off_Radio();
            stc.state = STS_S_WAIT_RADIO_OFF;
//            stc.state = STS_S_DUMMY;
        }
        else if (Rad_Get_Err_Code())
        {
            assertion("Radio error!");
        }

        break;

    case STS_S_WAIT_RADIO_OFF:

        if (Rad_Radio_Is_On() == false)
        {
            stc.state = STS_S_WAIT_WAKEUP_TIME;
        }
        else if (Rad_Get_Err_Code())
        {
            assertion("Radio error!");
        }

        break;

    case STS_S_DUMMY:

        if (Sts_Get_RAT_Int_Flag() == true) // ! do not call if radio is off
        {
            stc.state = STS_S_WAIT_WAKEUP_TIME;
        }

        break;

    default:
        assertion(!"Invalid state");
        break;
    }
}

static inline uint32_t Sts_Get_RTC_Time()
{
    // The obtained value is 1 OSC cycle delayed
    return (Tm_Get_RTC_Time() - 2);
}

static inline bool Sts_Get_RAT_Int_Flag()
{
    return (HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG) & RFC_DBELL_RFHWIFG_RATCH5);
}

static inline void Sts_Clear_RAT_Int_Flag()
{
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG) = ~RFC_DBELL_RFHWIFG_RATCH5;
}

static void Sts_Compensate_Drift(int32_t offset_ticks)
{
    // Add new sample, update index and calculate average
    stc.sync_err_samples[stc.samp_idx] = offset_ticks;
    if (stc.samp_idx < STS_SAMP_NUM - 1)
        stc.samp_idx++;
    else
        stc.samp_idx = 0;

    int32_t sum = 0;
    for (size_t i = 0; i < STS_SAMP_NUM; i++)
        sum += stc.sync_err_samples[i];

    Log_Val_Int32("sum:", sum);
    bool negative = false;
    if (sum < 0)
    {
        sum = sum * (-1); // this has to be done to avoid error in the rounding (modulo & negative)
        negative = true;
    }

    uint32_t average = sum / STS_SAMP_NUM;
    if (sum % STS_SAMP_NUM > STS_SAMP_NUM/2) // round to nearest
        average++;

    // Calculate ppm offset and adjust clock rate
    uint32_t ppm = average/(RAD_RAT_TICKS_PER_USEC * STS_SYNC_PER_SEC);
    assertion(ppm < sizeof(subsec_inc_offset));

    uint32_t new_rate = HWREG(AON_RTC_BASE + AON_RTC_O_SUBSECINC);
    if (negative == false)
        new_rate -= subsec_inc_offset[ppm];
    else
        new_rate += subsec_inc_offset[ppm];

    uint16_t new_rate_0 = 0x0000FFFF & new_rate;
    uint16_t new_rate_1 = (0x00FF0000 & new_rate) >> 16;

    HWREG(AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINC0) = new_rate_0;
    HWREG(AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINC1) = new_rate_1;
    HWREG(AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINCCTL) = 0x01;

    Log_Val_Int32("new:", offset_ticks);
    Log_Val_Int32("ave:", average);

    Log_Val_Int32("ppm:", ppm);
    Log_Val_Int32("offs:", subsec_inc_offset[ppm]);
    Log_Val_Int32("rate:", new_rate);
}


#ifdef CFG_DEBUG_START_OF_FRAME_OUT
void Sts_RTC_Isr()
{
    // Toggle bit at the start of each frame
    Brd_Led_Toggle(BRD_RTC_OUT_PIN);
    SysCtrlAonSync();
    AONRTCEventClear(AON_RTC_CH1);
    IntPendClear(INT_AON_RTC_COMB);

    proceed = true;
}
#endif // #ifdef PTC_START_OF_FRAME_OUT
