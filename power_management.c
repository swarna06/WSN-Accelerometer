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

#define SLEEP_TIME_MS       2000

void Pma_Init()
{
    // Set MCU wake-up events
    AONEventMcuWakeUpSet(AON_EVENT_MCU_WU0, AON_EVENT_RTC_CH1);

    AONRTCEventClear(AON_RTC_CH1);
    AONRTCCompareValueSet(AON_RTC_CH1, AONRTCCompareValueGet(AON_RTC_CH1) + SLEEP_TIME_MS*TM_RTC_TICKS_PER_MSEC);
}

void RTC_IRQ_Handler(void)
{
    AONRTCEventClear(AON_RTC_CH1);
    IntPendClear(INT_AON_PROG0);
}

void Pma_Sleep()
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
        CPUwfi();

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

        AONIOCFreezeDisable();

        GPIO_clearDio(BRD_LED0);

        Sep_Init();
        PRINTF("woke up\r\n");

        IntDisable(INT_AON_PROG0);
        AONRTCEventClear(AON_RTC_CH1);
        AONRTCCompareValueSet(AON_RTC_CH1, AONRTCCompareValueGet(AON_RTC_CH1) + SLEEP_TIME_MS*TM_RTC_TICKS_PER_MSEC);
    }



//    CPUwfi(); // wait for interrupt
//    CPUwfe();
//    HWREG(AON_RTC_BASE + AON_RTC_O_SYNC);
//    AONRTCEventClear(AON_RTC_CH1);
}

void Pma_Deep_Sleep()
{

}

void Pma_Wake_Up()
{

}


