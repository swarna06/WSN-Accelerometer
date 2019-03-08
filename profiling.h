/*
 * profiling.h
 *
 *  Created on: Nov 25, 2018
 *      Author: alvaro
 */

#ifndef PROFILING_H_
#define PROFILING_H_

#include <driverlib/prcm.h>
#include <driverlib/timer.h>

#include "configuration.h"

// Profiling global variables
extern volatile uint32_t pfl_tic;
extern volatile uint32_t pfl_toc;
extern volatile uint32_t pfl_wcet;

// Hardware definitions (timer number and peripheral id)
#define PFL_TIMER_BASE                          (GPT3_BASE)
#define PFL_PRMC_PERIPHERAL                     (PRCM_PERIPH_TIMER3)

// Constants for conversion between clock ticks and 'real' time
#define PFL_TICKS_PER_US                        48
#define PFL_NSEC_PER_TICK                       21

// Macro functions for conversion between clock ticks and 'real' time
#define Pfl_Ticks_To_Microsec(v)                (v/PFL_TICKS_PER_US)
#define Pfl_Ticks_To_Nanosec(v)                 (v*PFL_NSEC_PER_TICK)

// Macro functions for profiling
#define Pfl_Get_Current_Time()                  (HWREG(PFL_TIMER_BASE + GPT_O_TAR))  // get counter value
#define Pfl_Delta_Time32(start, end)            (end > start ? end - start : (((uint32_t)-1) - start) + end) // note: ((uint32_t)-1) = 0xFFFFFFFF
#define Pfl_Update_WCET(wcet, exec_time)        if (wcet < exec_time) wcet = exec_time; // worst-case execution time
#define Pfl_Update_BCET(bcet, exec_time)        if (bcet > exec_time) bcet = exec_time; // best-case execution time
#define Pfl_Update_AET(avg, ns, N)              if (N < (uint32_t)-1) { N++; avg -= avg/N; avg += ns/N; }; // average execution time

#define Pfl_Update_WCET2(wcet, start, end)      if (wcet < Pfl_Delta_Time32(start, end)) wcet = Pfl_Delta_Time32(start, end); // worst-case execution time
#define Pfl_Update_BCET2(bcet, start, end)      if (bcet > Pfl_Delta_Time32(start, end)) bcet = Pfl_Delta_Time32(start, end); // best-case execution time

// Macro functions for global time stamps
#define Pfl_Tic()                               pfl_tic = Pfl_Get_Current_Time();
#define Pfl_Toc()                               { \
                                                    pfl_toc = Pfl_Get_Current_Time(); \
                                                    Pfl_Update_WCET(pfl_wcet, Pfl_Delta_Time32(pfl_tic, pfl_toc)); \
                                                }
#define Pfl_Get_Exec_Time()                     Pfl_Delta_Time32(pfl_tic, pfl_toc)
#define Pfl_Get_Exec_Time_Microsec()            (Pfl_Ticks_To_Microsec(Pfl_Get_Exec_Time()))
#define Pfl_Get_Exec_Time_Nanosec()             (Pfl_Ticks_To_Nanosec(Pfl_Get_Exec_Time()))
#define Pfl_Get_WCET()                          (pfl_wcet)

static inline void Pfl_Init()
{
#if (CFG_DEBUG_PROFILING == CFG_SETTING_ENABLED)
    // Power timer and enable module clock
    if (PRCMPowerDomainStatus(PRCM_DOMAIN_TIMER) != PRCM_DOMAIN_POWER_ON)
        PRCMPowerDomainOn(PRCM_DOMAIN_TIMER);
    while (PRCMPowerDomainStatus(PRCM_DOMAIN_TIMER) != PRCM_DOMAIN_POWER_ON);

    PRCMPeripheralRunEnable(PFL_PRMC_PERIPHERAL);
    PRCMLoadSet();
    while(!PRCMLoadGet());

    // Setup and enable timer (free running timer)
    TimerConfigure(PFL_TIMER_BASE, TIMER_CFG_PERIODIC_UP);
    TimerLoadSet(PFL_TIMER_BASE, TIMER_A, 0xFFFFFFFF); // note: only timer A should be loaded when timer is configured in full-width mode
    TimerEnable(PFL_TIMER_BASE, TIMER_A);
#endif // #if (CFG_DEBUG_PROFILING == CFG_SETTING_ENABLED)
}

#endif /* PROFILING_H_ */
