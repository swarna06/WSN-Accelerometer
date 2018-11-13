
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include <driverlib/prcm.h>
#include <driverlib/sys_ctrl.h>
#include <driverlib/ioc.h>

#include <driverlib/gpio.h>
#include <driverlib/timer.h>

#include <driverlib/aon_event.h>

#include "board.h"
#include "timing.h"
#include "serial_port.h"
#include "rf_core.h"
#include "host_interface.h"
#include "protocol.h"
#include "log.h"
#include "coordinator.h"

#include "printf.h"

const uint32_t TIMER_TICKS_PER_USEC = 48;
const uint32_t DELAY_TIME_USEC = 1000*500; // 100 ms

void Startup();
void GPIO_Init();

void RTC_Int_Handler()
{
    GPIO_toggleDio(BRD_LED0);  // heart beat
    Tm_Abs_Period_Update();
}

int main(void)
{
    Startup();
    GPIO_Init();
    Tm_Init();
    Sep_Init();
    Rfc_Init();
    Pro_Init();
    Log_Init();
    Hif_Init();
    Crd_Init();

    Tm_Start_Period(TM_PER_HEARTBEAT_ID, TM_PER_HEARTBEAT_VAL);
    Tm_Enable_Abs_Time_Per();

    AONEventMcuSet(AON_EVENT_MCU_EVENT0, AON_EVENT_RTC_CH1);
    IntRegister(INT_AON_PROG0, RTC_Int_Handler);
    IntEnable(INT_AON_PROG0);

    Rfc_BLE5_Set_PHY_Mode(RFC_PHY_MODE_2MBPS);

    while (1)
    {
        if (Tm_System_Tick())
            Tm_Update_Time_Events();

//        if (Tm_Period_Completed(TM_PER_HEARTBEAT_ID))
//            GPIO_toggleDio(BRD_LED1);  // heart beat

//        if (Tm_Abs_Time_Per_Completed())
//            GPIO_toggleDio(BRD_LED0);  // heart beat

        Rfc_Process();

        Pro_Process();

        if (Hif_Data_Received())
            Hif_Process();

        Log_Process();

        Crd_Process();
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

    // Configure pins as 'standard' output
    IOCPinTypeGpioOutput(BRD_LED0);
    IOCPinTypeGpioOutput(BRD_LED1);
}


