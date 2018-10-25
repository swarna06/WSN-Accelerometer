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

#include <driverlib/aon_rtc.h>

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
//        if (Tm_RTC_Period_Completed())
        if (!GPIO_readDio(BRD_BUTTON1))
        {
            GPIO_toggleDio(BRD_GREEN_LED); // heart beat

            Rad_Synch_Master();
//            Rad_Ble5_Adv_Aux(NULL, 0);

//            Tm_Delay_Microsec(1000*1000); // debounce 1 sec
            GPIO_toggleDio(BRD_GREEN_LED); // heart beat

//            // Test: Synchronization with RTC
//            uint32_t start_time, end_time, cycle_time;
//            uint32_t rtc_timestamp0, rtc_timestamp1, rtc_read_time;
//
//            // Wait for current RTC cycle to end
//            Tm_Synch_With_RTC();
//
//            // Measure time of 1 RTC cycle
//            HWREG(AON_RTC_BASE + AON_RTC_O_SYNC) = 1; // write to force synchronization on read
//            start_time = Tm_Get_Free_Running_Timer_Val(); // tic
//            HWREG(AON_RTC_BASE + AON_RTC_O_SYNC); // read to synchronize with next SCLK_LF edge
//            end_time = Tm_Get_Free_Running_Timer_Val(); // toc
//
//            cycle_time = Tm_Delta_Time32(start_time, end_time);
//
//            // Synchronize with RTC and read RTC counter
//            Tm_Synch_With_RTC();
//            rtc_timestamp0 = AONRTCCurrentCompareValueGet();
//            // Synchronize again and read RTC counter, measure time required to read the RTC counter
//            Tm_Synch_With_RTC();
//            start_time = Tm_Get_Free_Running_Timer_Val(); // tic
//            rtc_timestamp1 = AONRTCCurrentCompareValueGet();
//            end_time = Tm_Get_Free_Running_Timer_Val(); // toc
//
//            rtc_read_time = Tm_Delta_Time32(start_time, end_time);
//
//            PRINTF("cycle time: %lu ns, rtc_read_time: %lu ns, rtc_ticks: %lu\r\n",
//                   cycle_time * TM_HFXOSC_NSEC_PER_TICK, rtc_read_time * TM_HFXOSC_NSEC_PER_TICK,
//                   rtc_timestamp1 - rtc_timestamp0);
//
//            // Results: (from terminal)
//            // cycle time: 30702 ns, rtc_read_time: 945 ns, rtc_ticks: 2
//            //
//            // Conclusions
//            // 1. Synchronization sequence (write SYNCH, read SYNCH) takes a complete period of the RTC (~30 usec)
//            // 2. Reading the RTC does not block for an entire RTC cycle
//            // 3. The RTC counter clock increments two units with every RTC period (counts on both edges?)
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
