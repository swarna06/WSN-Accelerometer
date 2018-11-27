
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
#include "power_management.h"
#include "profiling.h"

#include "printf.h"

PFL_DECLARE_PROFILING_VARIABLES();

void GPIO_Init();

int main(void)
{
    Pma_Init();
    GPIO_Init();
    Tm_Init();
    Sep_Init(); // FIXME disable to reduce power consumption
    Rfc_Init();
    Log_Init();

    #if (BRD_BOARD == BRD_LAUNCHPAD)
    Pfl_Init(); // FIXME disable to reduce power consumption
    #endif

    Rfc_Set_Tx_Power(RFC_TX_POW_0dBm);
    Rfc_BLE5_Set_PHY_Mode(RFC_PHY_MODE_2MBPS);
    Rfc_BLE5_Set_Channel(17);

    uint8_t count = 0;
    Brd_Led_Off(BRD_LED1);

    const uint32_t WAKEUP_TIME_MS = 100;
    rfc_tx_param_t tx_param;
    tx_param.buf = NULL; // empty packet
    tx_param.rat_start_time = 0; // transmit immediately

    uint32_t exec_time, wcet = 0;

    Tm_Start_Period(TM_PER_HEARTBEAT_ID, TM_PER_HEARTBEAT_VAL);

    while (1)
    {
        if (Tm_Period_Completed(TM_PER_HEARTBEAT_ID))
        {
            Brd_Led_Toggle(BRD_LED0);
            Log_Value("count: ", count++);
        }

        if (Tm_Sys_Tick())
            Tm_Process();

        Log_Process();

        Rfc_Process();

        continue;

        // ********************************
        // Active interval
        // ********************************
        Brd_Led_On(BRD_LED0);
        AONRTCEventClear(AON_RTC_CH0);
        AONRTCCompareValueSet(AON_RTC_CH0, Tm_Get_RTC_Time() + WAKEUP_TIME_MS*TM_RTC_TICKS_PER_MSEC);

        // Wake up RF core
        Pfl_Tic();

        Rfc_Wakeup();
        do
        {
            Rfc_Process();
            if (Tm_Sys_Tick())
                Tm_Process();
        } while (!Rfc_Ready());

        Pfl_Toc();
        exec_time = Pfl_Get_Exec_Time();
        wcet = Pfl_Get_WCET();

        // Wake up serial port and send message
        if (!(HWREG(UART0_BASE + UART_O_CTL) & UART_CTL_UARTEN))
        {
            Sep_Wakeup();
            PRINTF("count: %d\r\n", count++);
        }
        PRINTF("exec_time: %d ns, wcet: %d ns\r\n", Pfl_Ticks_To_Nanosec(exec_time), Pfl_Ticks_To_Nanosec(wcet));

        // Transmit packet
        Rfc_BLE5_Adv_Aux(&tx_param);
        do
        {
            Rfc_Process();
            if (Tm_Sys_Tick())
                Tm_Process();
        } while (!Rfc_Ready());

        // Blink LED using the timing module
        Tm_Start_Timeout(TM_TOUT_TEST_ID, 90);
        Tm_Start_Period(TM_PER_HEARTBEAT_ID, 30);
        Brd_Led_On(BRD_LED1);
        do
        {
            if (Tm_Sys_Tick())
                Tm_Process();

            if (Tm_Period_Completed(TM_PER_HEARTBEAT_ID))
                Brd_Led_Toggle(BRD_LED1);
        } while (!Tm_Timeout_Completed(TM_TOUT_TEST_ID));
        Brd_Led_Off(BRD_LED1);

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



