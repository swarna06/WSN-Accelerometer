/**
 * main.c
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>

#include <driverlib/prcm.h>
#include <driverlib/sys_ctrl.h>
#include <driverlib/ioc.h>

#include <driverlib/gpio.h>

#include "misc.h"
#include "printf.h"
#include "smartrf_settings.h"

#include "board.h"
#include "timing.h"
#include "serial_port.h"
#include "radio.h"

void Startup();
void GPIO_Init();

int main(void)
{
    Startup();
    GPIO_Init();

    Sep_UART_Init();

    Tm_Init_Aux_Timer(TM_AUXT_MODE_ONE_SHOT);
    Tm_Init_RTC();

    Rad_Init();

    Tm_Start_RTC_Period(500);

    while (1)
    {
        if (Tm_RTC_Period_Completed())
        {
            rad_tx_param_t tx_param;
            rad_tx_result_t tx_result;

            uint8_t payload[254];
            for (size_t n = 0; n < sizeof(payload); n++)
                payload[n] = n+1;

            tx_param.payload = (uint8_t*)payload + 1;
            tx_param.payload_len = sizeof(payload) - 1;
            tx_param.synch = false;

            Rad_Ble5_Adv_Aux(&tx_param, &tx_result);
            GPIO_toggleDio(BRD_GREEN_LED);
            uint32_t rat_delta = Tm_Delta_Time32(tx_result.rat_timestamp_start, tx_result.rat_timestamp_end);
            PRINTF("RTC: %lu, RAT start: %lu, RAT end: %lu, RAT delta: %lu\r\n",
                   tx_result.rtc_timestamp, tx_result.rat_timestamp_start,
                   tx_result.rat_timestamp_end, rat_delta * RAD_RAT_NSEC_PER_TICK);
        }
    }

	return 0;
}

void Startup()
{
    // Power peripherals
    PRCMPowerDomainOn(PRCM_DOMAIN_PERIPH);
    while((PRCMPowerDomainStatus(PRCM_DOMAIN_PERIPH) != PRCM_DOMAIN_POWER_ON));

    // Power serial interfaces
    PRCMPowerDomainOn(PRCM_DOMAIN_SERIAL);
    while((PRCMPowerDomainStatus(PRCM_DOMAIN_SERIAL) != PRCM_DOMAIN_POWER_ON));

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

    // Configure outputs
    IOCPinTypeGpioOutput(BRD_RED_LED);
    IOCPinTypeGpioOutput(BRD_GREEN_LED);

    IOCPinTypeGpioOutput(BRD_DEBUG_PIN0);
    GPIO_clearDio(BRD_DEBUG_PIN0);
    IOCPinTypeGpioOutput(BRD_DEBUG_PIN1);
    GPIO_clearDio(BRD_DEBUG_PIN1);

    // Configure inputs
    IOCPinTypeGpioInput(BRD_BUTTON1);
    IOCPinTypeGpioInput(BRD_BUTTON2);
    IOCIOPortPullSet(BRD_BUTTON1, IOC_IOPULL_UP);
    IOCIOPortPullSet(BRD_BUTTON2, IOC_IOPULL_UP);

    // Configure peripheral IO pin
    IOCPinTypeUart(UART0_BASE, IOID_UNUSED, IOID_3, IOID_UNUSED, IOID_UNUSED); // UART, TX only
}
