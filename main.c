/**
 * main.c
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include <driverlib/prcm.h>
#include <driverlib/sys_ctrl.h>
#include <driverlib/ioc.h>

#include <driverlib/gpio.h>

#include "board.h"
#include "timing.h"

void Startup();
void GPIO_Init();

int main(void)
{
    Startup();
    GPIO_Init();

    Tm_Init_Aux_Timer(TM_AUXT_MODE_ONE_SHOT);
    Tm_Init_RTC();

    Tm_Start_RTC_Period(500);

    while (1)
    {
        if (Tm_RTC_Period_Completed())
        {
            GPIO_toggleDio(BRD_GREEN_LED);
        }


//        Tm_Delay_Microsec(1000*500);
    }

	return 0;
}

void Startup()
{
    // Power peripherals
    PRCMPowerDomainOn(PRCM_DOMAIN_PERIPH);
    while((PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON));

    // Set clock source
    OSCClockSourceSet(OSC_SRC_CLK_MF | OSC_SRC_CLK_HF, OSC_XOSC_HF);
    while (OSCClockSourceGet(OSC_SRC_CLK_HF) != OSC_XOSC_HF)
    {
        OSCHfSourceSwitch();
    }
}

void GPIO_Init()
{
    // Power GPIO
    PRCMPeripheralRunEnable(PRCM_PERIPH_GPIO);
    PRCMLoadSet();
    while(!PRCMLoadGet());

    // GPIO as 'standard' output
    IOCPinTypeGpioOutput(BRD_RED_LED);
    IOCPinTypeGpioOutput(BRD_GREEN_LED);

    IOCPinTypeGpioOutput(BRD_DEBUG_PIN0);
    GPIO_clearDio(BRD_DEBUG_PIN0);
    IOCPinTypeGpioOutput(BRD_DEBUG_PIN1);
    GPIO_clearDio(BRD_DEBUG_PIN1);

    IOCPinTypeGpioInput(BRD_BUTTON1);
    IOCPinTypeGpioInput(BRD_BUTTON2);
    IOCIOPortPullSet(BRD_BUTTON1, IOC_IOPULL_UP);
    IOCIOPortPullSet(BRD_BUTTON2, IOC_IOPULL_UP);
}
