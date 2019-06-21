
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

//#define DRIVERLIB_NOROM // uncomment to disable ROM

#include <driverlib/prcm.h>
#include <driverlib/sys_ctrl.h>
#include <driverlib/ioc.h>
#include <driverlib/gpio.h>
#include <driverlib/timer.h>
#include <driverlib/aon_event.h>
#include <driverlib/uart.h>
#include <driverlib/ccfgread.h>
#include <inc/hw_ccfg.h>
#include <inc/hw_fcfg1.h>
#include <driverlib/trng.h>
#include <inc/hw_rfc_dbell.h>
#include <sensor_read.h>
#include "memory_test.h"

#include "board.h"
#include "timing.h"
#include "serial_port.h"
#include "rf_core.h"
#include "host_interface.h"
#include "protocol.h"
#include "log.h"
#include "power_management.h"
#include "profiling.h"
#include "configuration.h"
#include "printf.h"
#include "spi_bus.h"

void Sen_ISR();
void GPIO_Init();

// Global variables
volatile uint32_t pfl_tic, pfl_toc, pfl_wcet = 0, d_tm=0, p_tm=0; // time-stamp variables
volatile bool drdy_set = false,  pulse_flag = false;//flags for data ready, pulse and 1st sync pulse
volatile bool delay_ovr = false; //time keeper flag
int32_t abuf[5]; //sensor data buffer
static int32_t d_rdy=0;// data ready count

int main(void)
{
    int firstpulse_tm=0;
    bool firstpulse_flag=false;

/*----------------------------------------------
 *  Modules initialization
 * ---------------------------------------------
 */

    GPIO_Init();
    Tm_Init();
    Sep_Init();
    Log_Init();
    Pfl_Init();
    Spi_Init();
    Mem_Init();
    Sen_Init();

/*----------------------------------------------
 * Interrupt settings for external sync pulse train
 * ---------------------------------------------
 */
    IOCPinTypeGpioInput(BRD_GPIO_IN1);
    IOCIOPortPullSet(BRD_GPIO_IN1, IOC_IOPULL_UP);
    IOCIOIntSet(BRD_GPIO_IN1, IOC_INT_ENABLE, IOC_RISING_EDGE);
    IOCIOIntSet(BRD_SEN_INT1, IOC_INT_ENABLE, IOC_FALLING_EDGE);
    IOCIntRegister(Sen_ISR);
while (1)
{

  if (Tm_Sys_Tick())
      Tm_Process();
  Log_Process();

  if(!delay_ovr)
    {
      if(pulse_flag)
        {
            IOCIntDisable(BRD_GPIO_IN1);
            pulse_flag=false;
            IOCIntEnable(BRD_GPIO_IN1);
            if(!firstpulse_flag)
                {
                firstpulse_tm=p_tm;
                pfl_tic=firstpulse_tm;
                Pfl_Tic();  //start time for time keeping
                firstpulse_flag = true;
                Log_Value_Int(firstpulse_tm);Log_Line("1st pulse");
                }
            else
                {
                Log_Value_Int(p_tm);Log_Line("pulse");
                }

        }
        if(drdy_set)
        {
            IOCIntDisable(BRD_SEN_INT1);
            drdy_set=false;
            IOCIntEnable(BRD_SEN_INT1);
            Brd_Led_On(BRD_LED0);
            Sen_Read_Acc(abuf);  // Reading necessary to clear DRDY pulse
            Log_Value_Int(d_tm);Log_Line("");

        }

    }

  //Time keeper for 10 seconds
    Pfl_Toc();
    if(Pfl_Get_Exec_Time() >  10000000*48)
    {   delay_ovr=true;
        Brd_Led_Off(BRD_LED0);
    }
}

    return 0;
}

void Sen_ISR()
{
    int32_t t;
    t=Pfl_Get_Current_Time();

        if (IOCIntStatus(BRD_GPIO_IN1))  // Sync pulse interrupt
        {
            GPIO_toggleDio(BRD_GPIO_OUT2);
            p_tm=t;
            pulse_flag=true;
            IOCIntClear(BRD_GPIO_IN1);
        }

        if (IOCIntStatus(BRD_SEN_INT1)) // Data ready interrupt
        {
            d_tm=t;
            drdy_set = true;
            IOCIntClear(BRD_SEN_INT1);
        }
}

void GPIO_Init()
{
    Pma_Power_On_Peripheral(PMA_PERIPH_GPIO);

    // Configure pins as 'standard' output
    IOCPinTypeGpioOutput(BRD_LED0);
    IOCPinTypeGpioOutput(BRD_LED1);
    IOCPinTypeGpioOutput(BRD_GPIO_OUT2);
    IOCPinTypeGpioInput(BRD_SEN_INT1);
    IOCPinTypeGpioInput(BRD_GPIO_IN0);
    IOCIOPortPullSet(BRD_GPIO_IN0,IOC_IOPULL_UP);

    // Set initial values
    Brd_Led_Off(BRD_LED0);
    Brd_Led_Off(BRD_LED1);
    GPIO_clearDio(BRD_GPIO_OUT2);

}





