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
#include "fixedptc.h"

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
static int32_t Sts_Get_Weighted_Moving_Average(int32_t* samples, size_t idx);
void Sts_Compensate_Drift(int32_t offset_ticks);
void Sts_Compensate_Drift2(int32_t offset_ticks);

#ifdef CFG_DEBUG_START_OF_FRAME_OUT
void Sts_RTC_Isr();
#endif  // #ifdef PTC_START_OF_FRAME_OUT

void Sts_Init()
{
    Rad_Enable_Radio_Event_Output();
    Rad_Set_Data_Rate(STS_DATA_RATE);

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
    stc.rx_param.timeout = 0;

    stc.missed_pkt_cnt = 0;

    stc.samp_idx = 0;
    for (size_t i = 0; i < STS_SAMP_NUM; i++)
        stc.sync_err_samples[i] = 0;
    stc.samp_cnt = 0;

    HWREG(AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINC0) = 0;
    HWREG(AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINC1) = 0x80;
    HWREG(AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINCCTL) = 0x01;

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

        // Set PHY mode and turn on the radio
        Rad_Set_Data_Rate(STS_DATA_RATE);
        Rad_Turn_On_Radio();

        stc.state = STS_S_WAIT_RADIO_STARTUP;

        break;

    case STS_S_WAIT_RADIO_STARTUP:

        if (Rad_Radio_Is_On() == true)
        {
            if (stc.flags & STS_F_MASTER_INITIALIZED)
            {
                Rad_Set_RAT_Output();
                stc.state = STS_S_WAIT_SET_RAT_OUTPUT;
            }
            else
            {
                Tm_Synch_With_RTC();
                stc.rat_sync_time = Rad_Get_Radio_Time();
                stc.rtc_sync_time = Sts_Get_RTC_Time();
                stc.rtc_wakeup_time = stc.rtc_sync_time - STS_WAKEUP_DELAY;

                Rad_Turn_Off_Radio();
                stc.flags |= STS_F_MASTER_INITIALIZED;
                stc.state = STS_S_WAIT_RADIO_OFF;
            }
        }
        else if (Rad_Get_Err_Code())
        {
            assertion("Radio error!");
        }

        break;

    case STS_S_WAIT_SET_RAT_OUTPUT:

        if (Rad_Ready() == true)
        {
            Rad_Set_RAT_Cmp_Val(stc.rat_sync_time, NULL);
            stc.state = STS_S_WAIT_SET_RAT_CMP_VAL;
        }
        else if (Rad_Get_Err_Code())
        {
            assertion("Radio error!");
        }

        break;

    case STS_S_WAIT_SET_RAT_CMP_VAL:

        #if (CFG_DEBUG_RFC_ERR_BUTTON == CFG_SETTING_ENABLED)
        if (!GPIO_readDio(BRD_GPIO_IN0))
        {
            while (Sts_Get_RAT_Int_Flag() == false) {};
            Log_Line("Button pressed, no tx");
            stc.state = STS_S_WAIT_PKT_TX;
        } else
        #endif // #if (CFG_DEBUG_RFC_ERR_BUTTON == CFG_SETTING_ENABLED)
        if (Rad_Ready() == true)
        {
            stc.tx_param.start_time = stc.rat_sync_time - STS_RADIO_OP_DELAY;
            stc.tx_param.payload_len = RAD_MAX_PAYLOAD_LEN;
//            stc.tx_param.payload_len = 0;
            Rad_Transmit_Packet(&stc.tx_param);
            stc.state = STS_S_WAIT_PKT_TX;

            Log_Line("tx");
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

//        if (proceed == true)
        {
            Brd_Led_Toggle(BRD_RTC_OUT_PIN);

            stc.rtc_sync_time += STS_SYNC_PERIOD_RTC;
            stc.rtc_wakeup_time = stc.rtc_sync_time - STS_WAKEUP_DELAY;

            stc.rat_sync_time += STS_SYNC_PERIOD_RAT;

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
                Rad_Turn_Off_Radio();
                stc.state = STS_S_WAIT_RADIO_OFF;
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
        Rad_Set_Data_Rate(STS_DATA_RATE);
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
            // Calculate reception start time and timeout, and start radio operation
            stc.rx_param.delayed_start = true;

            stc.rx_param.start_time = stc.rat_sync_time;
            stc.rx_param.start_time -= (STS_GUART_TIME_RAT_TICKS + STS_RADIO_OP_DELAY);
            stc.rx_param.start_time -= STS_RX_START_DELAY;

            stc.rx_param.timeout = STS_PREAMBLE_TIME_RAT_TICKS;
            stc.rx_param.timeout += 2*STS_GUART_TIME_RAT_TICKS;
            stc.rx_param.timeout += STS_RADIO_OP_DELAY;

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
                stc.rat_sync_time = stc.rx_param.timestamp + STS_SYNC_PERIOD_RAT;

//                Sts_Compensate_Drift(sync_err_ticks);
                Sts_Compensate_Drift2(sync_err_ticks);
                stc.missed_pkt_cnt = 0;
            }
            else
            {
                stc.rtc_sync_time += STS_SYNC_PERIOD_RTC;
                stc.rtc_wakeup_time = stc.rtc_sync_time - STS_WAKEUP_DELAY;
                stc.rat_sync_time += STS_SYNC_PERIOD_RAT;

                stc.missed_pkt_cnt++;

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

static int32_t Sts_Get_Weighted_Moving_Average(int32_t* samples, size_t idx)
{
    uint32_t total_weight = 0;
    for (size_t n = 0; n < STS_SAMP_NUM; n++)
        total_weight += STS_SAMP_NUM - n; // TODO calculate once at init

    uint32_t weight = STS_SAMP_NUM;
    int32_t sum = 0;
    do
    {
        sum += weight * samples[idx];
        weight--;
        idx = idx > 0 ? idx - 1 : STS_SAMP_NUM - 1;
    } while (weight > 0);

    // Calculate average and round up
    // (absolute value is used to avoid error with modulo & negative value)
    int32_t tmp = sum >= 0 ? sum : (-1)*sum;
    int32_t average = tmp / total_weight;
    if (tmp % total_weight > total_weight / 2)
        average++;

    average = sum >= 0 ? average : (-1)*average;
    return average;
}

void Sts_Compensate_Drift(int32_t offset_ticks)
{
    // Add new sample, calculate average, and update index
    stc.sync_err_samples[stc.samp_idx] = offset_ticks;

    int32_t sum = Sts_Get_Weighted_Moving_Average(stc.sync_err_samples, stc.samp_idx);
    uint32_t average = sum >= 0 ? sum : (-1)*sum;

#if (STS_SAMP_NUM > 1)
    if (stc.samp_idx < STS_SAMP_NUM - 1)
        stc.samp_idx++;
    else
        stc.samp_idx = 0;
#endif

    // Calculate ppm offset and adjust clock rate
    uint32_t time_interval = RAD_RAT_TICKS_PER_USEC * STS_SYNC_PER_SEC * (1 + stc.missed_pkt_cnt);
    uint32_t ppm = average/time_interval;
    if (ppm >= sizeof(subsec_inc_offset))
    {
        Log_Line("ppm >= sizeof(subsec_inc_offset)");
        ppm = sizeof(subsec_inc_offset) - 1;
    }

    uint32_t new_rate = HWREG(AON_RTC_BASE + AON_RTC_O_SUBSECINC);
//    uint32_t new_rate = 0x800000;
    if (sum >= 0)
        new_rate -= subsec_inc_offset[ppm];
    else
        new_rate += subsec_inc_offset[ppm];

    // Adjust if we have enough samples
    if (stc.samp_cnt == 0)
    {
        uint16_t new_rate_0 = 0x0000FFFF & new_rate;
        uint16_t new_rate_1 = (0x00FF0000 & new_rate) >> 16;

        HWREG(AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINC0) = new_rate_0;
        HWREG(AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINC1) = new_rate_1;
        HWREG(AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINCCTL) = 0x01;
    }
    else
        stc.samp_cnt--;

    // Print result
    Log_String_Literal(""); Log_Value_Int(offset_ticks);
    Log_String_Literal(", "); Log_Value_Uint(average);
    Log_String_Literal(", "); Log_Value_Int(ppm);
    Log_String_Literal(", "); Log_Value_Int(1 + stc.missed_pkt_cnt);
    Log_Line(""); // new line
}

void Sts_Compensate_Drift2(int32_t offset_ticks)
{
    const fixedptu PPM_PER_TICK = fixedpt_rconst(0.250);
    const fixedptu INC_PER_PPM = fixedpt_rconst(8.3885);

    volatile uint32_t ticks_abs = offset_ticks >= 0 ? offset_ticks : (-1)*offset_ticks;
    if (ticks_abs > (1 << 13)) // max range exceeded ?
    {
        Log_Line("ticks_abs > (1 << 13)");
        return;
    }

    uint32_t interval = STS_SYNC_PER_SEC * (1 + stc.missed_pkt_cnt);
    ticks_abs = ticks_abs/interval; // TODO increase precision

    // Convert number of ticks to fixed point format
    volatile fixedptu ticks_fix = fixedpt_fromint(ticks_abs);

    // Divide by time interval since last beacon
//    volatile uint32_t interval = STS_SYNC_PER_SEC * (1 + stc.missed_pkt_cnt);
//    volatile fixedptu interval_fix = fixedpt_fromint(interval);
//    ticks_fix = fixedpt_xmul(ticks_fix, interval_fix);

    // Calculate corresponding SUBSECINC
    volatile fixedptu inc_fix = 0;
    inc_fix = fixedpt_xmul(ticks_fix, PPM_PER_TICK);
    inc_fix = fixedpt_xmul(inc_fix, INC_PER_PPM);

    volatile fixedptu inc_fracpart_fix = fixedpt_fracpart(inc_fix);
    volatile uint32_t inc = fixedpt_toint(inc_fix);
    if (inc_fracpart_fix > FIXEDPT_ONE_HALF)
        inc++;

    // Calculate delta and update SUBSECINC register
    uint32_t new_rate = HWREG(AON_RTC_BASE + AON_RTC_O_SUBSECINC);
    if (offset_ticks >= 0)
        new_rate -= inc;
    else
        new_rate += inc;

    uint16_t new_rate_0 = 0x0000FFFF & new_rate;
    uint16_t new_rate_1 = (0x00FF0000 & new_rate) >> 16;

    HWREG(AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINC0) = new_rate_0;
    HWREG(AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINC1) = new_rate_1;
    HWREG(AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINCCTL) = 0x01;

    // Print result
    Log_String_Literal(""); Log_Value_Int(offset_ticks);
    Log_String_Literal(", "); Log_Value_Int(inc);
    Log_String_Literal(", "); Log_Value_Int(1 + stc.missed_pkt_cnt);
    Log_Line(""); // new line

//    while (1)
//    {
//        Log_Process();
//    }

//    Log_Val_Uint32("offset_ticks:", offset_ticks);
//    Log_Val_Uint32("PPM_PER_TICK:", PPM_PER_TICK);
//    Log_Val_Uint32("PPM_PER_INC:", PPM_PER_INC);
//    Log_Val_Uint32("ticks_fix:", ticks_fix);
//    Log_Val_Uint32("interval:", interval);
//    Log_Val_Uint32("interval_fix:", interval_fix);
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
