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
#include <inc/hw_ccfg.h>

#include <driverlib/aon_batmon.h>
#include <sensor_read.h>

#include "power_management.h"
#include "timing.h"
#include "rf_core.h"
#include "serial_port.h"

#include "board.h"
#include "log.h"
#include "misc.h"
#include "spi_bus.h"
#include "protocol.h"

static pma_control_t pmc;
int32_t abuf[5];

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

    #ifdef PMA_SLEEP_OUT
    Pma_Power_On_Peripheral(PMA_PERIPH_GPIO);
    IOCPinTypeGpioOutput(BRD_SLEEP_PIN);
    Brd_Led_On(BRD_SLEEP_PIN);
    #endif // #ifdef PMA_SLEEP_OUT

    // Initialize battery voltage buffer and index
    pmc.idx = PMA_BATT_VOLT_SAMP_NUM - 1;
    uint16_t batt_volt = AONBatMonBatteryVoltageGet();
    for (size_t i = 0; i < PMA_BATT_VOLT_SAMP_NUM; i++)
        pmc.batt_volt[i] = batt_volt;
}

inline bool Pma_Batt_Volt_Meas_Ready()
{
    return AONBatMonNewTempMeasureReady();
}

void Pma_Process()
{
    // Add sample to buffer
    pmc.batt_volt[pmc.idx] = (uint16_t)AONBatMonBatteryVoltageGet();
    if (pmc.idx > 0)
        pmc.idx--;
    else
        pmc.idx = PMA_BATT_VOLT_SAMP_NUM - 1;
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
#ifdef PMA_SLEEP_OUT
    Brd_Led_Off(BRD_SLEEP_PIN); // signal indicates MCU state (sleep/active)
#endif // #ifdef PMA_SLEEP_OUT

    while (Sep_UART_Busy()); // wait until the UART FIFO becomes FIXME remove

#ifdef PMA_DUMMY_SLEEP

    // Dummy sleep; busy-wait until RTC event
    uint32_t curr_time = Tm_Get_RTC_Time();
    assertion(rtc_wakeup_time > curr_time + PMA_MIN_SLEEP_RTC_TICKS);

    AONRTCEventClear(AON_RTC_CH0);
    AONRTCCompareValueSet(AON_RTC_CH0, rtc_wakeup_time);
    Ptc_Get_Acc(abuf);
        if((uint8_t )(HWREG(CCFG_BASE + CCFG_O_IEEE_BLE_0))!=0)  // poll sensor data and accumulate buffer(pointer) if it is a sensor node
        {
                                    Sen_Read_Acc(abuf);
                                    Brd_Led_Toggle(BRD_LED0);
                                   /* Log_Value_Int(abuf[0]);Log_String_Literal(",");
                                      Log_Value_Int(abuf[1]);Log_String_Literal(",");
                                      Log_Value_Int(abuf[2]);Log_String_Literal(",");
                                      Log_Value_Int(abuf[3]);
                                      Log_Line(" ");*/

        }
    while (!AONRTCEventGet(AON_RTC_CH0)) // busy wait
    {
        Log_Process(); // flush log queue meanwhile
    }

#else

    // Put MCU in standby mode
    // Sequence taken from TI's power driver (PowerCC26XX.c)

    // Set compare value of RTC channel
    uint32_t curr_time = Tm_Get_RTC_Time();
    assertion(rtc_wakeup_time > curr_time + PMA_MIN_SLEEP_RTC_TICKS);
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

    // 5a. Request JTAG power domain off, noise in the TCK could power on this domain (TODO is this necessary?)
    AONWUCJtagPowerOff();

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

#endif // #ifndef PMA_DUMMY_SLEEP

#ifdef PMA_SLEEP_OUT
    Brd_Led_On(BRD_SLEEP_PIN); // signal indicates MCU state (sleep/active)
#endif // #ifdef PMA_SLEEP_OUT
}

void Pma_MCU_Wakeup()
{
    Rfc_Wakeup();
    Sep_Wakeup();
}

uint8_t Pma_Get_Batt_Volt_Fixed_Point()
{
    // Calculate average
    uint32_t sum = 0;
    for (size_t i = 0; i < PMA_BATT_VOLT_SAMP_NUM; i++) // TODO validate if average of fixed point variable is done properly
        sum += pmc.batt_volt[i];

    // Battery voltage measurement in a <int.frac> format size <3.8> in units of volt
    // The 3 least significant bits are discarded to fit the value in a single byte
    return (sum / PMA_BATT_VOLT_SAMP_NUM) >> 3;
}

void Pma_Get_Batt_Volt_Parts(uint8_t batt_volt,
                             uint8_t* int_part,
                             uint32_t* frac_part)
{
    // Extract the integer and fractional parts
    *int_part = batt_volt >> 5;

    *frac_part = 0;
    if (batt_volt & (1 << 4)) *frac_part += 50000 / (1 << 0);
    if (batt_volt & (1 << 3)) *frac_part += 50000 / (1 << 1);
    if (batt_volt & (1 << 2)) *frac_part += 50000 / (1 << 2);
    if (batt_volt & (1 << 1)) *frac_part += 50000 / (1 << 3);
    if (batt_volt & (1 << 0)) *frac_part += 50000 / (1 << 4);

    if (*frac_part < 10000) // hotfix TODO how to do zero padding for correct representation?
    {
        if (*frac_part > 500) *frac_part = 10000;
        else *frac_part = 0;
    }
}

void Pma_Get_Batt_Volt(uint8_t* int_part, uint32_t* frac_part)
{
    uint8_t batt_volt = Pma_Get_Batt_Volt_Fixed_Point();

    Pma_Get_Batt_Volt_Parts(batt_volt, int_part, frac_part);
}

