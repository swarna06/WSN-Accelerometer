/*
 * timing.h
 *
 *  Created on: 27 oct. 2018
 *      Author: Alvaro
 */

#ifndef TIMING_H_
#define TIMING_H_

#include <driverlib/aon_rtc.h>

// RTC ticks per millisecond
#define TM_RTC_TICKS_PER_MSEC   64

// Number of RTC clocks used to generate the system ticks
#define TM_SYS_TICK_RTC_CYCLES  TM_RTC_TICKS_PER_MSEC*1 // ~1 ms

// Total number of periods and timeouts
#define TM_PER_NUM              2       // xxx do not forget to update !
#define TM_TOUT_NUM             2       // xxx do not forget to update !

// List of periods, values in milliseconds
#define TM_PER_HEARTBEAT_ID     0
#define TM_PER_HEARTBEAT_VAL    1000

#define TM_PER_COORD_ID         1

// List of timeouts, values in milliseconds
#define TM_TOUT_TEST_ID         0
#define TM_TOUT_TEST_VAL        500

#define TM_RFC_TOUT_ID          1

// Synchronize with RTC (write to SYNC register prior reading to force a wait until next SCLK_LF edge)
#define Tm_Synch_With_RTC()     HWREG(AON_RTC_BASE + AON_RTC_O_SYNC) = 1; \
                                HWREG(AON_RTC_BASE + AON_RTC_O_SYNC);

// Macro to get RTC counter value (32-bit)
#define Tm_Get_RTC_Time()       AONRTCCurrentCompareValueGet()

// RTC CH1 period
#define TM_ABS_TIME_PERIOD_MS   200
#define TM_ABS_TIME_PER_TICKS   (TM_ABS_TIME_PERIOD_MS*TM_RTC_TICKS_PER_MSEC)

// Macro for getting the delta time between two time stamps
#define Tm_Delta_Time32(start, end) (end > start ? end - start : (0xFFFFFFFF - start) + end)

// Flags
#define TM_F_PER_ACTIVE         0x01
#define TM_F_PER_COMPLETED      0x02

// Period structure
typedef struct
{
  uint8_t flags;
  uint16_t counter,
  period;
} tm_period_t;

// Timing control structure
typedef struct
{
  tm_period_t period[TM_PER_NUM];
  uint16_t timeout[TM_TOUT_NUM];
} tm_control_t;

void Tm_Init();

bool Tm_Sys_Tick();

void Tm_Process();

void Tm_Start_Period(uint8_t per_idx, uint16_t per_val);

bool Tm_Period_Completed(uint8_t per_idx);

void Tm_End_Period(uint8_t per_idx);

void Tm_Start_Timeout(uint8_t tout_idx, uint16_t tout_val);

bool Tm_Timeout_Completed(uint8_t tout_idx);

void Tm_Enable_Abs_Time_Per();

bool Tm_Abs_Time_Per_Completed();

void Tm_Abs_Period_Update();

#endif /* TIMING_H_ */
