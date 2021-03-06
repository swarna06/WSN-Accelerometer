/*
 * timing.c
 *
 *  Created on: 27 oct. 2018
 *      Author: Alvaro
 */

#include <driverlib/aon_rtc.h>
#include <driverlib/aux_timer.h>
#include <driverlib/aon_event.h>
#include <driverlib/sys_ctrl.h>
#include <driverlib/ioc.h>

#include "timing.h"
#include "misc.h"
#include "board.h"
#include "configuration.h"

// Control structure to keep the state of the timing module
static tm_control_t tcs;

static void Tm_Start_System_Tick();
static void Tm_Enable_LF_Clock_Output();
static void Tm_Init_RTC();

void Tm_Init()
{
    uint8_t n;
    uint16_t *tp = (uint16_t *)tcs.timeout;
    tm_period_t *pp = (tm_period_t *)tcs.period;

    // reset all period flags
    for (n = TM_PER_NUM; n; --n, ++pp)
        pp->flags = 0;

    // reset all timeout counters
    for (n = TM_TOUT_NUM; n; --n, tp++)
        *tp = 0;

    Tm_Init_RTC();
    Tm_Start_System_Tick(1); // system tick (~1 msec)

    Tm_Enable_LF_Clock_Output();
}

bool Tm_Sys_Tick()
{
    // Check if RTC CH2 has generated an event
    // The return value of this function indicates if the time events should be updated
    if (AONRTCEventGet(AON_RTC_CH2))
    {
        AONRTCEventClear(AON_RTC_CH2);
        return true;
    }
    else
        return false;
}

void Tm_Adjust_Time()
{
    const int32_t MIN_DELTA = 4;
//    const int32_t MIN_DELTA = TM_RTC_TICKS_PER_MSEC;
    // Update the compare value of the RTC channel
    Tm_Synch_With_RTC();
    uint32_t compare_val = AONRTCCompareValueGet(AON_RTC_CH2);
    uint32_t curr_time = Tm_Get_RTC_Time();

    // Compare value should be ahead of RTC counter by at least 4 units otherwise it won't generate a compare event
    int32_t delta = compare_val - curr_time;
    if (delta < MIN_DELTA)
        AONRTCCompareValueSet(AON_RTC_CH2, curr_time + MIN_DELTA);
    else if (delta > TM_RTC_TICKS_PER_MSEC)
        AONRTCCompareValueSet(AON_RTC_CH2, curr_time + TM_RTC_TICKS_PER_MSEC);
}

void Tm_Process()
{
    uint8_t n;
    tm_period_t *pp;
    uint16_t *tp;

    // Update periods
    for (n = TM_PER_NUM, pp = tcs.period; n; --n, ++pp)
    {
        if (pp->flags & TM_F_PER_ACTIVE)
        {
            --(pp->counter);

            if (!pp->counter) // period completed ?
            {
                pp->flags |= TM_F_PER_COMPLETED;
                pp->counter = pp->period;
            }
        }
    }

    // Update timeouts
    for (n = TM_TOUT_NUM, tp = tcs.timeout; n; --n, tp++)
    {
        if (*tp)
            --(*tp);
    }
}

void Tm_Start_Period(uint8_t per_idx, uint16_t per_val)
{
    assertion(per_idx < TM_PER_NUM);

    tcs.period[per_idx].flags |= TM_F_PER_ACTIVE;
    tcs.period[per_idx].flags &= ~(TM_F_PER_COMPLETED);
    tcs.period[per_idx].period = per_val;
    tcs.period[per_idx].counter = per_val;
}

bool Tm_Period_Completed(uint8_t per_idx)
{
    assertion(per_idx < TM_PER_NUM);

    if (tcs.period[per_idx].flags & TM_F_PER_COMPLETED)
    {
        tcs.period[per_idx].flags &= ~(TM_F_PER_COMPLETED);
        return true;
    }
    else
        return false;
}

void Tm_End_Period(uint8_t per_idx)
{
    assertion(per_idx < TM_PER_NUM);
    tcs.period[per_idx].flags &= ~(TM_F_PER_ACTIVE);
}

void Tm_Start_Timeout(uint8_t tout_idx, uint16_t tout_val)
{
    assertion(tout_idx < TM_TOUT_NUM);
    tcs.timeout[tout_idx] = tout_val;
}

bool Tm_Timeout_Completed(uint8_t tout_idx)
{
    assertion(tout_idx < TM_TOUT_NUM);
    return (!(tcs.timeout[tout_idx]));
}

static void Tm_Enable_LF_Clock_Output()
{
#if (CFG_DEBUG_LF_OSC_OUT == CFG_SETTING_ENABLED)
    IOCPortConfigureSet(BRD_LF_OSC_PIN, IOC_PORT_AON_CLK32K, IOC_STD_OUTPUT);
    AONIOC32kHzOutputEnable();
#endif // #if (CFG_DEBUG_LF_OSC_OUT == CFG_SETTING_ENABLED)
}

// ********************************
// Static functions
// ********************************

static void Tm_Init_RTC()
{
    // Enable 16 kHz signal used to sync up with the Radio Timer (RAT)
    HWREG(AON_RTC_BASE + AON_RTC_O_CTL) |= AON_RTC_CTL_RTC_UPD_EN;

    // Enable RTC if it isn't already running
    if (AONRTCActive() == false)
    {
        AONRTCReset();
        AONRTCEnable();
    }

    // Set increment value of RTC counter
    HWREG(AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINC0) = 0;
    HWREG(AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINC1) = 0x80;
    HWREG(AUX_WUC_BASE + AUX_WUC_O_RTCSUBSECINCCTL) = 0x01;

    // Make sure writes take effect
    SysCtrlAonSync();

    // Reset RTC
    AONRTCReset();
}

static void Tm_Start_System_Tick()
{
    // Note: RTC CH2 is used to generate a periodic signal used as the system tick
    uint32_t current_time, compare_val;

    // Set compare value of RTC CH2 - TODO is it necessary to use absolute values ?
    Tm_Synch_With_RTC(); // wait for the start of next RTC cycle
    current_time = Tm_Get_RTC_Time();
    if (TM_RTC_TICKS_PER_MSEC > current_time)
        compare_val = TM_RTC_TICKS_PER_MSEC;
    else
        compare_val = current_time + (TM_RTC_TICKS_PER_MSEC - (current_time % TM_RTC_TICKS_PER_MSEC));
    AONRTCCompareValueSet(AON_RTC_CH2, compare_val); // TODO this value must be updated during wake up if CH2 is disabled before going to sleep

    // Set the operation mode (periodic compare),
    // and the auto-increment value, and enable RTC CH2
    AONRTCModeCh2Set(AON_RTC_MODE_CH2_CONTINUOUS);
    AONRTCIncValueCh2Set(TM_RTC_TICKS_PER_MSEC);
    AONRTCChannelEnable(AON_RTC_CH2); // TODO disable CH2 befor going to sleep
}

