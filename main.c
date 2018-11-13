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

#include "host_interface.h"
#include "protocol.h"
#include "coordinator.h"

#include "printf.h"

void Startup();
void GPIO_Init();

void RTC_Int_Handler()
{
    GPIO_toggleDio(BRD_LED0);  // heart beat
    Tm_Abs_Period_Update();
}

extern pro_control_t prc;

int main(void)
{
    Startup();
    GPIO_Init();
    Tm_Init();
    Sep_Init();
    Log_Init();
    Rfc_Init();

    Pro_Init();
    Hif_Init();
    Crd_Init();

    Rfc_Set_Tx_Power(RFC_TX_POW_MINUS_21dBm);
    Rfc_BLE5_Set_PHY_Mode(RFC_PHY_MODE_2MBPS);
    Rfc_BLE5_Set_Channel(17);

    Log_String_Literal("\f"); // clear screen

    Tm_Start_Period(TM_PER_HEARTBEAT_ID, TM_PER_HEARTBEAT_VAL);
    Tm_Enable_Abs_Time_Per();

    AONEventMcuSet(AON_EVENT_MCU_EVENT0, AON_EVENT_RTC_CH1);
    IntRegister(INT_AON_PROG0, RTC_Int_Handler);
    IntEnable(INT_AON_PROG0);;

//    int state = -1;
//    int *state_p = &prc.state;
    bool op_in_progress = false;
    const int MODE_TX = 1;
    const int MODE_RX = 2;
    int mode = 0;

    while (1)
    {
        if (!GPIO_readDio(BRD_GPIO_IN0))
        {
            mode = MODE_TX;
            Log_Line("MODE_TX");
            break;
        }
        else if (!GPIO_readDio(BRD_GPIO_IN1))
        {
            mode = MODE_RX;
            Log_Line("MODE_RX");
            break;
        }
    }

    while (1)
    {
        if (Tm_System_Tick())
            Tm_Update_Time_Events();

        if (Tm_Period_Completed(TM_PER_HEARTBEAT_ID))
        {
            GPIO_toggleDio(BRD_LED1);  // heart beat

            if (!op_in_progress && Rfc_Ready())
            {
                op_in_progress = true;

                if (mode == MODE_TX)
                {
                    uint8_t buf[] = "Hola!";
                    rfc_tx_param_t tx_param;
                    tx_param.buf = buf;
                    tx_param.len = sizeof(buf);
                    tx_param.rat_start_time = 0;
                    Rfc_BLE5_Adv_Aux(&tx_param);

                    Log_Value("sizeof(buf)", sizeof(buf));
                }
                else
                {
                    Rfc_BLE5_Scanner(1000*1500);
                    op_in_progress = true;
                }
            }
        }

        //        if (Tm_Abs_Time_Per_Completed())
        //            GPIO_toggleDio(BRD_LED0);  // heart beat

//        if (state != *state_p)
//        {
//            state = *state_p;
//            Log_Value("", state);
//        }
//
        Rfc_Process();

        if (op_in_progress && Rfc_Ready())
        {
            op_in_progress = false;

            if (mode == MODE_TX)
            {
                Log_Line("Pkt sent");
            }
            else
            {
                uint8_t buf[256];
                rfc_rx_result_t rx_result;
                rx_result.buf = buf;
                rx_result.buf_len = sizeof(buf);

                Rfc_BLE5_Get_Scanner_Result(&rx_result);

                PRINTF("len: %d\r\n", rx_result.payload_len);
                PRINTF("buf: ");
                for (size_t n = 0; n < rx_result.payload_len; n++)
                {
                    PRINTF("%02x ", buf[n]);
                }
                PRINTF("\r\n");
            }
        }

//
//        Pro_Process();
//
//        if (Hif_Data_Received())
//            Hif_Process();
//
        Log_Process();
//
//        Crd_Process();
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

    // Configure inputs
    IOCPinTypeGpioInput(BRD_GPIO_IN0);
    IOCPinTypeGpioInput(BRD_GPIO_IN1);
    IOCIOPortPullSet(BRD_GPIO_IN0, IOC_IOPULL_UP);
    IOCIOPortPullSet(BRD_GPIO_IN1, IOC_IOPULL_UP);
}
