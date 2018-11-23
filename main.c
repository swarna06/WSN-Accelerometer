
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

void GPIO_Init();

int main(void)
{
    Pma_Init();
    GPIO_Init();
    Sep_Init();

    uint8_t count = 0;
    Brd_Led_Off(BRD_LED1);

    while (1)
    {
        const uint32_t WAKEUP_TIME_MS = 500;

        // ********************************
        // Active interval
        // ********************************
        Brd_Led_On(BRD_LED0);
        AONRTCEventClear(AON_RTC_CH0);
        AONRTCCompareValueSet(AON_RTC_CH0, Tm_Get_RTC_Time() + WAKEUP_TIME_MS*TM_RTC_TICKS_PER_MSEC);

        if (!(HWREG(UART0_BASE + UART_O_CTL) & UART_CTL_UARTEN))
        {
            Sep_Wakeup();
            PRINTF("count: %d\r\n", count++);
        }

        while (!AONRTCEventGet(AON_RTC_CH0)); // busy wait

        // ********************************
        // Sleep interval
        // ********************************
        Brd_Led_Off(BRD_LED0);
        Pma_MCU_Sleep(Tm_Get_RTC_Time() + WAKEUP_TIME_MS*TM_RTC_TICKS_PER_MSEC); // sleep
//        Pma_MCU_Sleep(0); // sleep forever
    }

    return 0;
}

void GPIO_Init()
{
    Pma_Power_On_Peripheral(PMA_PERIPH_GPIO);

    // Configure pins as 'standard' output
    IOCPinTypeGpioOutput(BRD_LED0);
    IOCPinTypeGpioOutput(BRD_LED1);

    // Set initial values
    Brd_Led_Off(BRD_LED0);
    Brd_Led_Off(BRD_LED1);
}



