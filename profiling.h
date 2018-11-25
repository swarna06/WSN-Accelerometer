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

// Hardware definitions (timer number and peripheral id)
#define PFL_TIMER_BASE                  (GPT3_BASE)
#define PFL_PRMC_PERIPHERAL             (PRCM_PERIPH_TIMER3)

// Constants for conversion between clock ticks and 'real' time
#define PFL_TICKS_PER_US                48
#define PFL_NSEC_PER_TICK               21

// Macro functions for conversion between clock ticks and 'real' time
#define Pfl_Ticks_To_Microsec(v)        (v/PFL_TICKS_PER_US)
#define Pfl_Ticks_To_Nanosec(v)         (v*PFL_NSEC_PER_TICK)

// Macro functions for profiling
#define Pfl_Get_Current_Time()          (HWREG(PFL_TIMER_BASE + GPT_O_TAR))  // get counter value
#define Pfl_Delta_Time32(start, end)    (end > start ? end - start : (((uint32_t)-1) - start) + end) // note: ((uint32_t)-1) = 0xFFFFFFFF
#define Pfl_WCET(wcet, exec_time)       if (wcet < exec_time) wcet = exec_time; // worst-case execution time
#define Pfl_BCET(bcet, exec_time)       if (bcet > exec_time) min_time = exec_time; // best-case execution time
#define Pfl_AET(avg, ns, N)             if (N < (uint32_t)-1) { N++; avg -= avg/N; avg += ns/N; }; // average execution time

void Pfl_Init()
{
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
}

#endif /* PROFILING_H_ */
