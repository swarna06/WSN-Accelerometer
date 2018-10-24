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
    Tm_Init_Free_Running_Timer();

    Rad_Init();

    Tm_Start_RTC_Period(500);

    while (1)
    {
        if (Tm_RTC_Period_Completed())
        {
            // Test: Synchronization with RTC
            uint32_t start_time, end_time, cycle_time;

            // Wait for current RTC cycle to end
            HWREG(AON_RTC_BASE + AON_RTC_O_SYNC) = 1; // write to force synchronization on read
            while (HWREG(AON_RTC_BASE + AON_RTC_O_SYNC)) { }; // read to synchronize with next SCLK_LF edge

            // Measure time of 1 RTC cycle
            HWREG(AON_RTC_BASE + AON_RTC_O_SYNC) = 1; // write to force synchronization on read
            start_time = Tm_Get_Free_Running_Timer_Val();
            while (HWREG(AON_RTC_BASE + AON_RTC_O_SYNC)) { }; // read to synchronize with next SCLK_LF edge
            end_time = Tm_Get_Free_Running_Timer_Val();

            cycle_time = Tm_Delta_Time32(start_time, end_time);
            PRINTF("cycle time: %lu ns\r\n", cycle_time * TM_HFXOSC_NSEC_PER_TICK);

            GPIO_toggleDio(BRD_GREEN_LED);


//            uint8_t payload[36];
//            for (size_t n = 0; n < sizeof(payload); n++)
//                payload[n] = n+1;
//
//            Rad_Ble5_Adv_Aux(payload, sizeof(payload));
//            GPIO_toggleDio(BRD_GREEN_LED);

//            uint8_t buf[256];
//            size_t payload_len;
//            payload_len = Rad_Ble5_Scanner(buf, sizeof(buf));
//            PRINTF("payload: ");
//            for (size_t n = 0; n < payload_len; n++)
//                PRINTF("%02x ", buf[n]);
//            PRINTF("\r\n");

//            uint32_t rat_delta = Tm_Delta_Time32(tx_result.rat_timestamp_start, tx_result.rat_timestamp_end);
//            PRINTF("RTC: %lu, RAT start: %lu, RAT end: %lu, RAT delta: %lu\r\n",
//                   tx_result.rtc_timestamp, tx_result.rat_timestamp_start,
//                   tx_result.rat_timestamp_end, rat_delta * RAD_RAT_NSEC_PER_TICK);
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
