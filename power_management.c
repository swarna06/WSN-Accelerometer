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
#include <driverlib/ioc.h>

#include "power_management.h"
#include "timing.h"

#include <driverlib/gpio.h>
#include "board.h"
#include "printf.h"
#include "serial_port.h"

#define SLEEP_TIME_MS       10

static volatile pma_control_t pcs;

void Pma_RTC_Isr()
{
    AONRTCEventClear(AON_RTC_CH0);
    SysCtrlAonSync();
    IntPendClear(INT_AON_PROG0);
    IntDisable(INT_AON_PROG0);
}

void Pma_Init()
{
    // Enable XOSC
    OSCClockSourceSet(OSC_SRC_CLK_MF | OSC_SRC_CLK_HF, OSC_XOSC_HF);
    if (OSCClockSourceGet(OSC_SRC_CLK_HF) != OSC_XOSC_HF)
        OSCHfSourceSwitch();

    // Enable RTC if it isn't already running
    if (AONRTCActive() == false)
    {
        AONRTCReset();
        AONRTCEnable();
    }

    // Set MCU wake-up RTC event
    AONRTCEventClear(AON_RTC_CH0);
    AONEventMcuSet(AON_EVENT_MCU_EVENT0, AON_EVENT_RTC_CH0);
    AONEventMcuWakeUpSet(AON_EVENT_MCU_WU0, AON_EVENT_RTC_CH0);

    // Enable CH0 of RTC
    AONRTCCompareValueSet(AON_RTC_CH0, AONRTCCompareValueGet(AON_RTC_CH0) + SLEEP_TIME_MS*TM_RTC_TICKS_PER_MSEC);
    AONRTCChannelEnable(AON_RTC_CH0);

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

    // Initialize control structure
    pcs.powered_peripherals = 0;
}

void Pma_Power_On_Peripheral(uint16_t periph_id)
{
    int32_t power_domain = -1;
    int32_t peripheral = -1;
    // Most significant bits of id indicates power domain
    // Least significant bits of id indicates peripheral
    if (periph_id & PMA_POWER_DOMAIN_RF_CORE) // TODO separate RF core powering from other peripherals
    {
        power_domain = PRCM_DOMAIN_RFCORE;
    }
    else if (periph_id & PMA_POWER_DOMAIN_SERIAL)
    {
        power_domain = PRCM_DOMAIN_SERIAL;
        switch(periph_id)
        {
        case PMA_PERIPH_UART0: peripheral = PRCM_PERIPH_UART0; break;
        }
    }
    else if (periph_id & PMA_POWER_DOMAIN_PERIPH)
    {
        power_domain = PRCM_DOMAIN_PERIPH;
        switch(periph_id)
        {
        case PMA_PERIPH_GPIO: peripheral = PRCM_PERIPH_GPIO; break;
        }
    }

    // Check if power_domain was assigned
    if (power_domain < 0)
        return;

    // Turn on power domain if not already on
    if (PRCMPowerDomainStatus(power_domain) != PRCM_DOMAIN_POWER_ON)
        PRCMPowerDomainOn(power_domain);

    // Enable peripheral's clock only when CPU is running
    if (peripheral >= 0)
    {
        PRCMPeripheralRunEnable(peripheral);
        PRCMLoadSet();
        while(!PRCMLoadGet()); // FIXME deadlock risk ?
    }

    // Wait until power domain becomes on
    while (PRCMPowerDomainStatus(power_domain) != PRCM_DOMAIN_POWER_ON); // FIXME deadlock risk ?

    // Keep track of powered peripherals
    pcs.powered_peripherals |= periph_id;
}

inline void Pma_CPU_Sleep(uint32_t tout_ms)
{
    if (tout_ms == 0)
        return;

    // Set compare value of RTC channel
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
    OSCClockSourceSet(OSC_SRC_CLK_MF | OSC_SRC_CLK_HF, OSC_RCOSC_HF);
    if (OSCClockSourceGet(OSC_SRC_CLK_HF) != OSC_RCOSC_HF)
        OSCHfSourceSwitch();

    OSCClockSourceSet(OSC_SRC_CLK_LF, OSC_RCOSC_LF);
    while(OSCClockSourceGet(OSC_SRC_CLK_LF) != OSC_RCOSC_LF);

    // 3. Allow AUX to power down
    AONWUCAuxWakeupEvent(AONWUC_AUX_ALLOW_SLEEP);

    // 4. Make sure writes take effect
    SysCtrlAonSync();

    // 5. Request power off all domains in the MCU voltage domain
    uint32_t power_domains = 0;
    if (pcs.powered_peripherals & PMA_POWER_DOMAIN_RF_CORE)
        power_domains |= PRCM_DOMAIN_RFCORE;
    if (pcs.powered_peripherals & PMA_POWER_DOMAIN_SERIAL)
        power_domains |= PRCM_DOMAIN_SERIAL;
    if (pcs.powered_peripherals & PMA_POWER_DOMAIN_PERIPH)
        power_domains |= PRCM_DOMAIN_PERIPH;
    PRCMPowerDomainOff(power_domains | PRCM_DOMAIN_CPU);

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
    PRCMPowerDomainOn(power_domains);

    // 3. Release request for uLDO
    PRCMMcuUldoConfigure(false);

    // 4. Wait until all power domains are back on
    while (PRCMPowerDomainStatus(power_domains) != PRCM_DOMAIN_POWER_ON);

    // 5. Disable IO freeze
    AONIOCFreezeDisable();

    // 6. Synchronize with AON domain (ensure RTC shadow value is updated)
    SysCtrlAonSync();

    // 7. Wait until AON AUX becomes ready
    while (AONWUCPowerStatusGet() & AONWUC_AUX_POWER_DOWN);

    // 8. Enable XOSC - TODO check how to do this properly
    OSCClockSourceSet(OSC_SRC_CLK_MF | OSC_SRC_CLK_HF, OSC_XOSC_HF);
    while (OSCClockSourceGet(OSC_SRC_CLK_HF) != OSC_XOSC_HF)
    {
        OSCHfSourceSwitch();
    }
}
