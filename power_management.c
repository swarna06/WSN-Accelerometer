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

    // Initialize control structure
    pcs.powered_peripherals = 0;
}

void Pma_Power_On_Peripheral(uint8_t periph_id)
{
    int32_t power_domain = -1;
    int32_t peripheral = -1;
    // Most significant bits of id indicates power domain
    // Least significant bits of id indicates peripheral
    if (periph_id & PMA_POWER_DOMAIN_RF_CORE)
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
        power_domain = PMA_POWER_DOMAIN_PERIPH;
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
    }

    // Wait until power domain becomes on
    while (PRCMPowerDomainStatus(power_domain) != PRCM_DOMAIN_POWER_ON); // TODO deadlock risk ?

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

void RTC_IRQ_Handler(void)
{
    AONRTCEventClear(AON_RTC_CH1);
    IntPendClear(INT_AON_PROG0);
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

    // 7. Disable VIMS and cache retention - FIXME is this needed ?
    uint32_t mode_vims;
    while ((mode_vims = VIMSModeGet(VIMS_BASE)) == VIMS_MODE_CHANGING); //7.1 Get the current VIMS mode

    if (mode_vims == VIMS_MODE_ENABLED)
        VIMSModeSet(VIMS_BASE, VIMS_MODE_OFF); // 7.2 Now turn off the VIMS

    PRCMCacheRetentionDisable(); // 7.3 Now disable retention

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

    // 2. Turn on power domains - TODO check if power domains are ON
    PRCMPowerDomainOn(power_domains);

    // 3. TODO re-enable peripheral clocks (including RF core)

    // 4. Release request for uLDO
    PRCMMcuUldoConfigure(false);

    // 5. Wait until all power domains are back on
    while (PRCMPowerDomainStatus(power_domains) != PRCM_DOMAIN_POWER_ON);

    // 6. Disable IO freeze and ensure RTC shadow value is updated
    AONIOCFreezeDisable();

    // 7. Synchronize with AON domain
    SysCtrlAonSync();

    // 8. Wait until AON AUX becomes ready
    while (AONWUCPowerStatusGet() & AONWUC_AUX_POWER_DOWN);

    // 8. Enable XOSC - TODO check how to do this properly
    OSCClockSourceSet(OSC_SRC_CLK_MF | OSC_SRC_CLK_HF, OSC_XOSC_HF);
    while (OSCClockSourceGet(OSC_SRC_CLK_HF) != OSC_XOSC_HF)
    {
        OSCHfSourceSwitch();
    }
}

void Pma_MCU_Sleep2()
{
    if (HWREG(AON_RTC_BASE + AON_RTC_O_EVFLAGS) & AON_RTC_EVFLAGS_CH1)
    {
        AONRTCEventClear(AON_RTC_CH1);
        IntRegister(INT_AON_PROG0, RTC_IRQ_Handler);
        IntPendClear(INT_AON_PROG0);
        IntEnable(INT_AON_PROG0);

        // Allow for power down
        AONWUCDomainPowerDownEnable();

        // Enable the DC/DC converter for lower power
        PowerCtrlSourceSet(PWRCTRL_PWRSRC_DCDC);

        // Set the HF clocks to correct source
        OSCClockSourceSet(OSC_SRC_CLK_MF | OSC_SRC_CLK_HF, OSC_RCOSC_HF);
        if (OSCClockSourceGet(OSC_SRC_CLK_HF) != OSC_RCOSC_HF)
            OSCHfSourceSwitch();

        // Set the LF clocks to correct source
        OSCClockSourceSet(OSC_SRC_CLK_LF, OSC_RCOSC_LF);
        while(OSCClockSourceGet(OSC_SRC_CLK_LF) != OSC_RCOSC_LF);

        // Configure recharge interval
        //        AONWUCRechargeCtrlConfigSet(); TODO

        // Configure one or more wake-up sources for MCU
        AONEventMcuWakeUpSet(AON_EVENT_MCU_WU0, AON_EVENT_RTC_CH1);

        // Configure power-down clock for MCU
        AONWUCMcuPowerDownConfig(AONWUC_NO_CLOCK);

        // Configure power-down clock for AUX
        AONWUCAuxPowerDownConfig(AONWUC_NO_CLOCK);

        // Configure system SRAM retention
        AONWUCMcuSRamConfig(MCU_RAM0_RETENTION | MCU_RAM1_RETENTION |
                            MCU_RAM2_RETENTION | MCU_RAM3_RETENTION);

        // Turn off JTAG
        AONWUCJtagPowerOff();

        // Configure the wake-up source to generate an event
        AONRTCEventClear(AON_RTC_CH1);
        AONRTCCompareValueSet(AON_RTC_CH1, AONRTCCompareValueGet(AON_RTC_CH1) + SLEEP_TIME_MS*TM_RTC_TICKS_PER_MSEC);

        // Request AUX_PD power down
        HWREG(AUX_WUC_BASE + AUX_WUC_O_PWRDWNREQ) = AUX_WUC_PWRDWNREQ_REQ;

        // Disconnect AUX from system bus
        HWREG(AUX_WUC_BASE + AUX_WUC_O_MCUBUSCTL) = AUX_WUC_MCUBUSCTL_DISCONNECT_REQ;

        // Latch I/O state
        GPIO_setDio(BRD_LED0);
        GPIO_setDio(BRD_LED1);
        AONIOCFreezeEnable();

        // Turn off power domains and verify they are turned off
        PRCMPowerDomainOff(PRCM_DOMAIN_RFCORE |
                           PRCM_DOMAIN_SERIAL |
                           PRCM_DOMAIN_PERIPH |
                           PRCM_DOMAIN_VIMS |
                           PRCM_DOMAIN_SYSBUS |
                           PRCM_DOMAIN_CPU);
        while((PRCMPowerDomainStatus(PRCM_DOMAIN_RFCORE | PRCM_DOMAIN_SERIAL | PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_OFF));

        // Request digital supply to be Micro LDO
        AUXWUCPowerCtrl(PWRCTRL_PWRSRC_ULDO);

        // Synchronize transactions to AON domain
        SysCtrlAonSync();

        // Sleep (wait for event)
        //        CPUwfe();
//        CPUwfi();
        PRCMDeepSleep();

        SysCtrlAonSync();

        // Power peripherals
        PRCMPowerDomainOn(PRCM_DOMAIN_PERIPH);
        while((PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON));

        // Power GPIO
        PRCMPeripheralRunEnable(PRCM_PERIPH_GPIO);
        PRCMLoadSet();
        while(!PRCMLoadGet());

        // Configure pins as 'standard' output
        IOCPinTypeGpioOutput(BRD_LED0);
        IOCPinTypeGpioOutput(BRD_LED1);

        GPIO_clearDio(BRD_LED0);
//        GPIO_setDio(BRD_LED0);
        GPIO_setDio(BRD_LED1);

        AONIOCFreezeDisable();

        Sep_Init();
        PRINTF("woke up\r\n");

        IntDisable(INT_AON_PROG0);
        AONRTCEventClear(AON_RTC_CH1);
        AONRTCCompareValueSet(AON_RTC_CH1, AONRTCCompareValueGet(AON_RTC_CH1) + SLEEP_TIME_MS*TM_RTC_TICKS_PER_MSEC);
    }
}


