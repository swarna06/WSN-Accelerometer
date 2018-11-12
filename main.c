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
#include <driverlib/timer.h>

#include <driverlib/aon_event.h>

#include "board.h"
#include "timing.h"
#include "serial_port.h"
#include "rf_core.h"
#include "log.h"

#include "printf.h"

void Startup();
void GPIO_Init();

int main(void)
{
    Startup();
    GPIO_Init();
    Tm_Init();
    Sep_Init();
    Log_Init();
    Rfc_Init();

    Tm_Start_Period(TM_PER_HEARTBEAT_ID, TM_PER_HEARTBEAT_VAL);

    int state = -1;

    Rfc_Set_Tx_Power(RFC_TX_POW_MINUS_21dBm);
    Rfc_Set_BLE5_PHY_Mode(RFC_PHY_MODE_1MBPS);

    while (1)
    {
        if (Tm_System_Tick())
            Tm_Update_Time_Events();

        if (Tm_Period_Completed(TM_PER_HEARTBEAT_ID))
        {
            GPIO_toggleDio(BRD_LED1);  // heart beat

            if (Rfc_Ready())
            {
                uint8_t buf[] = "Hola!";
                rfc_tx_param_t tx_param;
                tx_param.buf = buf;
                tx_param.len = sizeof(buf);
                tx_param.rat_start_time = 0;
                Rfc_BLE5_Adv_Aux(&tx_param);

//                Log_Line("Rfc_Ready");
            }
        }

        if (state != rfc.state)
        {
            state = rfc.state;
            Log_Value("", state);
        }

        Rfc_Process(); // RF core FSM

        Log_Process();

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
    IOCPinTypeGpioOutput(BRD_LED0);
    IOCPinTypeGpioOutput(BRD_LED1);
}
