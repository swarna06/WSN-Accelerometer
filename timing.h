#ifndef TIMING_H__
#define TIMING_H__

#include <stdint.h>

// RTC ticks per millisecond
#define TM_RTC_TICKS_PER_MSEC   64

// Auxiliary timer ticks per microsecond
#define TM_AUXT_TICKS_PER_US    48

// Auxiliary timer operation modes
#define TM_AUXT_MODE_ONE_SHOT   true
#define TM_AUXT_MODE_PERIODIC   false

// System tick
#define TM_HW_SYSTICK_PER       48000 // 1ms
#define TM_TICK_PER_MS			1 // systick 1 ms
#define TM_TICKS(time_ms)		(time_ms/TM_TICK_PER_MS)

// Total number of periods and timeouts
#define TM_PER_NUM				3       // TODO keep this value updated !
#define TM_TOUT_NUM				1       // TODO keep this value updated !

// Periods
#define TM_PER0_IDX             0
#define TM_PER0_VAL		        TM_TICKS(1000)

#define TM_HEART_BEAT_PER_IDX   1
#define TM_HEART_BEAT_PER_VAL   TM_TICKS(1000)

#define TM_PER_XOSC_CAL_IDX     2
#define TM_PER_XOSC_CAL_VAL     TM_TICKS(500)

// Timeouts
#define TEST_TOUT_IDX			0
#define TEST_TOUT_VAL			TM_TICKS(20)

// Flags
#define TM_F_PER_ACTIVE			0x01
#define TM_F_PER_COMPLETED		0x02

typedef struct
{
  uint8_t flags;
  uint16_t counter,
  period;
} tm_period_t;

typedef struct
{
  tm_period_t period[TM_PER_NUM];
  uint16_t timeout[TM_TOUT_NUM];
} tm_control_t;

void Tm_Init();

void Tm_Update_Events();

void Tm_Start_Period(uint8_t per_idx,
                     uint16_t per_val,
                     uint16_t delay);

uint8_t Tm_Period_Completed(uint8_t per_idx);

void Tm_End_Period(uint8_t per_idx);

void Tm_Start_Timeout(uint8_t tout_idx, uint16_t tout_val);

uint8_t Tm_Timeout_Completed(uint8_t tout_idx);

void Tm_Init_Aux_Timer(bool mode);

void Tm_Set_Aux_Timer(uint32_t ticks);

void Tm_Start_Aux_Timer();

void Tm_Enable_Aux_Timer_Int(void (*timer_isr)(void));

void Tm_Disable_Aux_Timer_Int();

bool Tm_Aux_Timer_Event_Occurred();

void Tm_Wait_Aux_Timer_Event();

void Tm_Delay_Microsec(uint32_t ticks);

void Tm_Init_RTC();

void Tm_Start_RTC_Period(uint32_t period_ms);

bool Tm_RTC_Period_Completed();

#endif // TIMING_H__ 
