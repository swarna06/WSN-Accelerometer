/*
 * timing.c
 *
 *  Created on: 27 oct. 2018
 *      Author: Alvaro
 */

#include <driverlib/aon_rtc.h>
#include <driverlib/aux_timer.h>
#include <driverlib/aon_event.h>

#include "timing.h"
#include "misc.h"

// Control structure to keep the state of the timing module
static tm_control_t tcs;

void Tm_Init_RTC()
{
    // Reset RTC
    AONRTCReset();

    // Enable 16 kHz signal used to sync up with the RAdio Timer
    HWREG(AON_RTC_BASE + AON_RTC_O_CTL) |= AON_RTC_CTL_RTC_UPD_EN;

    // Enable RTC
    AONRTCEnable();
    while (HWREG(AON_RTC_BASE + AON_RTC_O_SYNC)); // synch CPU and AON
}

void Tm_Start_RTC_Period(uint32_t period_ms)
{
    const uint32_t TICKS_PER_PERIOD = period_ms*TM_RTC_TICKS_PER_MSEC;
    uint32_t curr_time, cmp_val;

    // Configure and enable CH2 (periodic compare)
    AONRTCModeCh2Set(AON_RTC_MODE_CH2_CONTINUOUS);
    Tm_Synch_With_RTC(); // wait for next RTC cycle
    curr_time = AONRTCCurrentCompareValueGet();

    if (TICKS_PER_PERIOD > curr_time)
        cmp_val = TICKS_PER_PERIOD;
    else
        cmp_val = curr_time + (TICKS_PER_PERIOD - (curr_time % TICKS_PER_PERIOD));

    AONRTCCompareValueSet(AON_RTC_CH2, cmp_val);
    AONRTCIncValueCh2Set(TICKS_PER_PERIOD);
    AONRTCChannelEnable(AON_RTC_CH2);
}

void Tm_Enable_Abs_Time_Per()
{
    const uint32_t TICKS_PER_PERIOD = TM_ABS_TIME_PERIOD_MS*TM_RTC_TICKS_PER_MSEC;
    uint32_t curr_time, cmp_val;

    // Configure and enable CH2 (periodic compare)
    AONRTCModeCh1Set(AON_RTC_MODE_CH1_COMPARE);
    Tm_Synch_With_RTC(); // wait for next RTC cycle
    curr_time = AONRTCCurrentCompareValueGet();

    if (TICKS_PER_PERIOD > curr_time)
        cmp_val = TICKS_PER_PERIOD;
    else
        cmp_val = curr_time + (TICKS_PER_PERIOD - (curr_time % TICKS_PER_PERIOD));

    AONRTCCompareValueSet(AON_RTC_CH1, cmp_val);
    AONRTCChannelEnable(AON_RTC_CH1);
}

bool Tm_Abs_Time_Per_Completed()
{
    if (AONRTCEventGet(AON_RTC_CH1))
    {
        const uint32_t TICKS_PER_PERIOD = TM_ABS_TIME_PERIOD_MS*TM_RTC_TICKS_PER_MSEC;
        AONRTCEventClear(AON_RTC_CH1);
        AONRTCCompareValueSet(AON_RTC_CH1, AONRTCCompareValueGet(AON_RTC_CH1) + TICKS_PER_PERIOD);

        return true;
    }
    else
        return false;
}

void Tm_Abs_Period_Update()
{
    const uint32_t TICKS_PER_PERIOD = TM_ABS_TIME_PERIOD_MS*TM_RTC_TICKS_PER_MSEC;
    AONRTCEventClear(AON_RTC_CH1);
    AONRTCCompareValueSet(AON_RTC_CH1, AONRTCCompareValueGet(AON_RTC_CH1) + TICKS_PER_PERIOD);
}

bool Tm_RTC_Period_Completed()
{
    if (AONRTCEventGet(AON_RTC_CH2))
    {
        AONRTCEventClear(AON_RTC_CH2);
        return true;
    }
    else
        return false;
}

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
    Tm_Start_RTC_Period(1); // system tick (~1 msec)
}

void Tm_Adjust_Counters()
{
    Tm_Synch_With_RTC(); // wait for next RTC cycle
    uint32_t rtc_curr_time = Tm_Get_RTC_Time();
    uint32_t rtc_period;
    uint32_t remain_rtc_ticks = 0;

    uint8_t n;
    tm_period_t *pp = (tm_period_t *)tcs.period;

    // reset all period flags
    for (n = TM_PER_NUM; n; --n, ++pp)
    {
        if (pp->flags & TM_F_PER_ACTIVE)
        {
            rtc_period = pp->period * TM_RTC_TICKS_PER_MSEC; // TODO move to LUT

            if (rtc_period > rtc_curr_time)
                remain_rtc_ticks = rtc_period - rtc_curr_time;
            else
                remain_rtc_ticks = (rtc_curr_time + rtc_period) - (rtc_curr_time % rtc_period);

            pp->counter = remain_rtc_ticks/TM_RTC_TICKS_PER_MSEC;
            if (!pp->counter) // period completed ?
            {
                pp->flags |= TM_F_PER_COMPLETED;
                pp->counter = pp->period;
            }
        }
    }
}

void Tm_Update_Time_Events()
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

    // update timeouts
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


