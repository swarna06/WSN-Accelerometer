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

bool proceed = false;

static sts_control stc;

static void Sts_Sink_Process();
static void Sts_Sensor_Process();

static uint32_t Sts_Get_RTC_Time();

#ifdef CFG_DEBUG_START_OF_FRAME_OUT
void Sts_RTC_Isr();
#endif  // #ifdef PTC_START_OF_FRAME_OUT

void Sts_Init()
{
    // Get device ID
    uint32_t secondary_ble_addr_l = HWREG(CCFG_BASE + CCFG_O_IEEE_BLE_0);
    stc.dev_id = (uint8_t)secondary_ble_addr_l;

    if (stc.dev_id == 0)
        Sts_Process = Sts_Sink_Process;
    else
        Sts_Process = Sts_Sensor_Process;

    stc.sync_time = Tm_Get_RTC_Time() + STS_SYNC_PERIOD_RTC;
    stc.wakeup_time = stc.sync_time - STS_WAKEUP_DELAY;
    stc.state = STS_S_WAIT_WAKEUP_TIME;

    stc.tx_param.delayed_start = true;
    stc.tx_param.payload_p = stc.tx_buf;
    stc.tx_param.payload_len = 0;

    Rad_Enable_Radio_Event_Output();

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

static void Sts_Sink_Process()
{
    switch (stc.state)
    {
    case STS_S_WAIT_WAKEUP_TIME:

        Pma_MCU_Sleep(stc.wakeup_time);

        // Set interrupt compare value to match the synchronization instant
        AONRTCCompareValueSet(AON_RTC_CH1, stc.sync_time);

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
            uint32_t rtc_delta = stc.sync_time - rtc_curr_time;

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
            stc.sync_time += STS_SYNC_PERIOD_RTC;
            stc.wakeup_time = stc.sync_time - STS_WAKEUP_DELAY;

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

}

static inline uint32_t Sts_Get_RTC_Time()
{
    // The obtained value is 1 OSC cycle delayed
    return (Tm_Get_RTC_Time() - 2);
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
