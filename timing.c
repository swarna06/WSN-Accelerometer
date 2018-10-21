
#include <driverlib/prcm.h>
#include <driverlib/systick.h>
#include <driverlib/timer.h>

#include "timing.h"

uint8_t state = 0;

static volatile tm_control_t tcs;	// keeps the state of the 'timing' module

void Tm_HW_Setup()
{
    // Systick setup
    SysTickPeriodSet(TM_HW_SYSTICK_PER);
    SysTickIntRegister(Tm_Update_Events);
//    IntPrioritySet(INT_SYSTICK, INT_PRI_LEVEL2); // TODO xxx what shoud be the priority for the timer interrupt ?
    SysTickIntEnable();
    SysTickEnable();
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

    // Setup hardware
    Tm_HW_Setup();
}

void Tm_Update_Events()
{
	uint8_t n;
	volatile tm_period_t *pp;
	volatile uint16_t *tp;

    // update periods
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

void Tm_Start_Period(uint8_t per_idx,
                     uint16_t per_val,
                     uint16_t delay)
{
    if (per_idx >= TM_PER_NUM)		// TODO assert()
        return;

    tcs.period[per_idx].flags |= TM_F_PER_ACTIVE;
    tcs.period[per_idx].flags &= ~(TM_F_PER_COMPLETED);
    tcs.period[per_idx].period = per_val;
    if (delay)
        tcs.period[per_idx].counter = delay;
    else
        tcs.period[per_idx].counter = per_val;
}

uint8_t Tm_Period_Completed(uint8_t per_idx)
{
	if (per_idx >= TM_PER_NUM)		// TODO assert()
        return 0;

	if (tcs.period[per_idx].flags & TM_F_PER_COMPLETED)
    {
        tcs.period[per_idx].flags &= ~(TM_F_PER_COMPLETED);
        return 1;
    }
    else
        return 0;
}

void Tm_End_Period(uint8_t per_idx)
{
	if (per_idx >= TM_PER_NUM)		// TODO assert()
        return;

	tcs.period[per_idx].flags &= ~(TM_F_PER_ACTIVE);
}

void Tm_Start_Timeout(uint8_t tout_idx, uint16_t tout_val)
{
	if (tout_idx >= TM_TOUT_NUM)		// TODO assert()
        return;

	tcs.timeout[tout_idx] = tout_val;
}

uint8_t Tm_Timeout_Completed(uint8_t tout_idx)
{
	if (tout_idx >= TM_TOUT_NUM)		// TODO assert()
        return 0;

	return (!(tcs.timeout[tout_idx]));
}

void Tm_Init_Aux_Timer(bool mode)
{
    PRCMPeripheralRunEnable(PRCM_PERIPH_TIMER0);
    PRCMLoadSet();
    while(!PRCMLoadGet());

    // Setup timer0
    // Mode: TIMER_CFG_ONE_SHOT, TIMER_CFG_PERIODIC
    if (mode == TM_AUX_TIMER_MODE_ONE_SHOT)
        TimerConfigure(GPT0_BASE, TIMER_CFG_ONE_SHOT);
    else
        TimerConfigure(GPT0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(GPT0_BASE, TIMER_A, 0);
}

void Tm_Set_Aux_Timer(uint32_t ticks)
{
    TimerLoadSet(GPT0_BASE, TIMER_A, ticks);
}

void Tm_Start_Aux_Timer()
{
    TimerIntClear(GPT0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(GPT0_BASE, TIMER_A);
}

void Tm_Enable_Aux_Timer_Int(void (*timer_isr)(void))
{
    TimerIntClear(GPT0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntRegister(GPT0_BASE, TIMER_A, timer_isr);
    TimerIntEnable(GPT0_BASE, TIMER_A);
}

void Tm_Disable_Aux_Timer_Int()
{
    TimerIntDisable(GPT0_BASE, TIMER_A);
}

bool Tm_Aux_Timer_Event_Occurred()
{
    return (TimerIntStatus(GPT0_BASE, false) & TIMER_TIMA_TIMEOUT);
}

void Tm_Wait_Aux_Timer_Event()
{
    while (!(TimerIntStatus(GPT0_BASE, false) & TIMER_TIMA_TIMEOUT));
}

void Tm_Delay(uint32_t ticks)
{
    TimerLoadSet(GPT0_BASE, TIMER_A, ticks);
    TimerIntClear(GPT0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(GPT0_BASE, TIMER_A);
    while (!(TimerIntStatus(GPT0_BASE, false) & TIMER_TIMA_TIMEOUT));
}
