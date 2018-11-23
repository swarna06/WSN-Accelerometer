
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

//#define DRIVERLIB_NOROM

#include <driverlib/prcm.h>
#include <driverlib/sys_ctrl.h>
#include <driverlib/ioc.h>

#include <driverlib/gpio.h>
#include <driverlib/timer.h>

#include <driverlib/aon_event.h>

#include <driverlib/ccfgread.h>
#include <inc/hw_ccfg.h>
#include <inc/hw_fcfg1.h>
#include <driverlib/trng.h>

#include "board.h"
#include "timing.h"
#include "serial_port.h"
#include "rf_core.h"
#include "host_interface.h"
#include "protocol.h"
#include "log.h"
#include "coordinator.h"
#include "power_management.h"

#include "printf.h"

const uint32_t TIMER_TICKS_PER_USEC = 48;
const uint32_t DELAY_TIME_USEC = 1000*500; // 100 ms

void Startup();
void GPIO_Init();

int main(void)
{
    Pma_Init();
    Startup();
    GPIO_Init();

    while (1)
    {
        const uint32_t WAKEUP_TIME_MS = 50;
        GPIO_toggleDio(BRD_LED0);
        Pma_MCU_Sleep(Tm_Get_RTC_Time() + WAKEUP_TIME_MS*TM_RTC_TICKS_PER_MSEC);

//        AONRTCEventClear(AON_RTC_CH0);
//        AONRTCCompareValueSet(AON_RTC_CH0, Tm_Get_RTC_Time() + WAKEUP_TIME_MS*TM_RTC_TICKS_PER_MSEC);
//        while (!AONRTCEventGet(AON_RTC_CH0));

//        Pma_Power_On_Peripheral(PMA_PERIPH_GPIO);
    }

    return 0;
}

void Startup()
{
    // Set clock source
    OSCClockSourceSet(OSC_SRC_CLK_MF | OSC_SRC_CLK_HF, OSC_XOSC_HF);
    if (OSCClockSourceGet(OSC_SRC_CLK_HF) != OSC_XOSC_HF)
    {
        OSCHfSourceSwitch();
    }
}

void GPIO_Init()
{
//    // Power peripherals
//    PRCMPowerDomainOn(PRCM_DOMAIN_PERIPH);
//    while((PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON));
//
//    // Power GPIO
//    PRCMPeripheralRunEnable(PRCM_PERIPH_GPIO);
//    PRCMLoadSet();
//    while(!PRCMLoadGet());
    Pma_Power_On_Peripheral(PMA_PERIPH_GPIO);

    // Configure pins as 'standard' output
    IOCPinTypeGpioOutput(BRD_LED0);
    IOCPinTypeGpioOutput(BRD_LED1);
}



