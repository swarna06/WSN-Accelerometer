/*
 * power_management.c
 *
 *  Created on: Nov 19, 2018
 *      Author: alvaro
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include <driverlib/prcm.h>
#include <driverlib/aon_event.h>
#include <driverlib/cpu.h>
#include <driverlib/aon_rtc.h>
#include <driverlib/sys_ctrl.h>

#include "power_management.h"
#include "timing.h"
#include "rf_core.h"
#include "serial_port.h"

#include "board.h"

void Pma_RTC_Isr()
{
    AONRTCEventClear(AON_RTC_CH0);
    SysCtrlAonSync();
    IntPendClear(INT_AON_PROG0);
    IntDisable(INT_AON_PROG0);
}

void Pma_Init()
{
    // Enable HF XOSC
    OSCHF_TurnOnXosc();
    while (!OSCHF_AttemptToSwitchToXosc());

    // Use 32768 Hz XOSC
    OSCClockSourceSet(OSC_SRC_CLK_LF, OSC_XOSC_LF);
    while(OSCClockSourceGet(OSC_SRC_CLK_LF) != OSC_XOSC_LF);

    // Enable RTC if it isn't already running
    if (AONRTCActive() == false)
    {
        AONRTCReset();
        AONRTCEnable();
    }

    // Enable RTC CH0
    AONRTCCompareValueSet(AON_RTC_CH0, 0);
    AONRTCChannelEnable(AON_RTC_CH0);

    // Set MCU wake-up RTC event
    AONRTCEventClear(AON_RTC_CH0);
    AONEventMcuSet(AON_EVENT_MCU_EVENT0, AON_EVENT_RTC_CH0);
    AONEventMcuWakeUpSet(AON_EVENT_MCU_WU0, AON_EVENT_RTC_CH0);

    // Make sure writes take effect
    SysCtrlAonSync();

    // Setup RTC interrupt
    IntRegister(INT_AON_PROG0, Pma_RTC_Isr);
    IntPendClear(INT_AON_PROG0);

    // Disable VIMS and cache retention - TODO is this needed ?
    uint32_t mode_vims;
    while ((mode_vims = VIMSModeGet(VIMS_BASE)) == VIMS_MODE_CHANGING); // get the current VIMS mode

    if (mode_vims == VIMS_MODE_ENABLED)
        VIMSModeSet(VIMS_BASE, VIMS_MODE_OFF); // now turn off the VIMS

    PRCMCacheRetentionDisable(); // now disable retention
}

void Pma_Power_On_Peripheral(uint16_t peripheral)
{
    // Most significant bits of 'peripheral' indicate power domain
    // Least significant bits of 'peripheral' indicate peripheral module
    uint32_t power_domain;
    uint32_t periph_module;

    // Assign PRMC value for power domain
    if (peripheral & PMA_F_DOMAIN_RF_CORE)
        power_domain = PRCM_DOMAIN_RFCORE;
    else if (peripheral & PMA_F_DOMAIN_SERIAL)
        power_domain = PRCM_DOMAIN_SERIAL;
    else if (peripheral & PMA_F_DOMAIN_PERIPH)
        power_domain = PRCM_DOMAIN_PERIPH;
    else
        return; // invalid peripheral value; do nothing

    // Assign PRMC value for peripheral module
    switch(peripheral)
    {
    case PMA_PERIPH_RF_CORE: periph_module = (uint32_t)-1; // value doesn't matter
    case PMA_PERIPH_UART0: periph_module = PRCM_PERIPH_UART0; break;
    case PMA_PERIPH_GPIO: periph_module = PRCM_PERIPH_GPIO; break;
    default:
        return; // invalid peripheral value; do nothing
    }

    // Turn on power domain if not already on
    if (PRCMPowerDomainStatus(power_domain) != PRCM_DOMAIN_POWER_ON)
        PRCMPowerDomainOn(power_domain);

    // Enable peripheral's clock only when CPU is running
    if (peripheral != PMA_PERIPH_RF_CORE)
    {
        PRCMPeripheralRunEnable(periph_module);
        PRCMLoadSet();
        while(!PRCMLoadGet()); // FIXME deadlock risk ?
    }

    // Wait until power domain becomes on
    while (PRCMPowerDomainStatus(power_domain) != PRCM_DOMAIN_POWER_ON); // FIXME deadlock risk ?
}

inline void Pma_CPU_Sleep(uint32_t tout_ms)
{
    if (tout_ms == 0)
        return;

    // Set compare value and enable RTC channel
    AONRTCEventClear(AON_RTC_CH0);
    AONRTCCompareValueSet(AON_RTC_CH0, Tm_Get_RTC_Time() + tout_ms*TM_RTC_TICKS_PER_MSEC);

    // Enable compare interrupt
    IntPendClear(INT_AON_PROG0);
    IntEnable(INT_AON_PROG0);

    // Synchronize with AON domain
    SysCtrlAonSync();

    // Go to sleep
    PRCMSleep();

    // Synchronize with AON domain
    SysCtrlAonSync();

    // Disable RTC interrupt in case the CPU was awaken by other interrupt source
    IntDisable(INT_AON_PROG0);
}

void Pma_MCU_Sleep(uint32_t rtc_wakeup_time)
{
    Brd_Led_Off(BRD_LED0);  // FIXME remove
    while (!Sep_UART_Idle()); // wait until the UART FIFO becomes FIXME remove

    // Put MCU in standby mode
    // Sequence taken from TI's power driver (PowerCC26XX.c)

    // Set compare value of RTC channel
    AONRTCEventClear(AON_RTC_CH0);
    AONRTCCompareValueSet(AON_RTC_CH0, rtc_wakeup_time);

    // Enable compare interrupt
    IntPendClear(INT_AON_PROG0);
    IntEnable(INT_AON_PROG0);

    // ********************************
    // Prepare to sleep
    // ********************************
    // 1. Freeze the IOs on the boundary between MCU and AON
    AONIOCFreezeEnable();

    // 2. Switch from XOSC to RCOSC
    OSCHF_SwitchToRcOscTurnOffXosc();

    // 3. Allow AUX to power down
    AONWUCAuxWakeupEvent(AONWUC_AUX_ALLOW_SLEEP);

    // 4. Make sure writes take effect
    SysCtrlAonSync();

    // 5. Request power off all domains in the MCU voltage domain
    uint32_t powered_domains = 0;
    if (PRCMPowerDomainStatus(PRCM_DOMAIN_RFCORE) == PRCM_DOMAIN_POWER_ON)
        powered_domains |= PRCM_DOMAIN_RFCORE;
    if (PRCMPowerDomainStatus(PRCM_DOMAIN_SERIAL) == PRCM_DOMAIN_POWER_ON)
        powered_domains |= PRCM_DOMAIN_SERIAL;
    if (PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) == PRCM_DOMAIN_POWER_ON)
        powered_domains |= PRCM_DOMAIN_PERIPH;
    PRCMPowerDomainOff(powered_domains | PRCM_DOMAIN_CPU);

    // 6. Request uLDO during standby
    PRCMMcuUldoConfigure(true);

    // 8. Setup recharge parameters
    SysCtrlSetRechargeBeforePowerDown(XOSC_IN_HIGH_POWER_MODE);

    // 9. Make sure all writes have taken effect
    SysCtrlAonSync();

    // 10. Invoke deep sleep to go to STANDBY
    PRCMDeepSleep();

    // ********************************
    // Wakeup
    // ********************************
    // 1. Start forcing on power to AUX
    AONWUCAuxWakeupEvent(AONWUC_AUX_WAKEUP);

    // 2. Turn on power domains
    PRCMPowerDomainOn(powered_domains);

    // 3. Release request for uLDO
    PRCMMcuUldoConfigure(false);

    // 4. Wait until all power domains are back on
    while (PRCMPowerDomainStatus(powered_domains) != PRCM_DOMAIN_POWER_ON);

    // 5. Disable IO freeze
    AONIOCFreezeDisable();

    // 6. Synchronize with AON domain (ensure RTC shadow value is updated)
    SysCtrlAonSync();

    // 7. Wait until AON AUX becomes ready
    while (AONWUCPowerStatusGet() & AONWUC_AUX_POWER_DOWN);

    // 8. Enable XOSC
    OSCHF_TurnOnXosc();
    while (!OSCHF_AttemptToSwitchToXosc());

    // 9. Wake up modules
    Pma_MCU_Wakeup();

    Brd_Led_On(BRD_LED0);  // FIXME remove
}


void Pma_MCU_Wakeup()
{
    Rfc_Wakeup();
    Sep_Wakeup();
}
