
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

// v0.1.7a
//#define DRIVERLIB_NOROM // uncomment to disable ROM

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
#include <inc/hw_rfc_dbell.h>

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

#include "sensor_test.h"
#include "spi_bus.h"

// Global profiling variables
volatile uint32_t pfl_tic, pfl_toc, pfl_wcet = 0;


void GPIO_Init();

int main(void)
{
    // Modules' initialization
    Pma_Init();
    GPIO_Init();
    Tm_Init();

    Sep_Init();
    Log_Init();
    Pfl_Init();

    Rfc_Init();
    Ptc_Init();

    Spi_Init();
    Sen_Init();
    // DEBUG TODO remove
    //Tm_Start_Period(TM_PER_HEARTBEAT_ID, TM_PER_HEARTBEAT_VAL);
    IOCPinTypeGpioInput(BRD_GPIO_IN0);
    IOCPinTypeGpioInput(BRD_GPIO_IN1);
    IOCIOPortPullSet(BRD_GPIO_IN0, IOC_IOPULL_UP);
    IOCIOPortPullSet(BRD_GPIO_IN1, IOC_IOPULL_UP);
    int16_t abuf[4];
    // Round-robin scheduling (circular execution, no priorities)
    while (1)
    {
        Log_Process();
      /*  if (Tm_Period_Completed(TM_PER_HEARTBEAT_ID))
                    {
                        Brd_Led_Toggle(BRD_LED0);
                        //Log_Line("heartbeat");
                        //Sen_Read_Acc_Test(abuf);
                    }
                    */

       if (Tm_Sys_Tick())
           Tm_Process();
        if (Pma_Batt_Volt_Meas_Ready())
            Pma_Process();


       Rfc_Process();

        if (Rfc_Ready())
            Ptc_Process();
        else if (Rfc_Error())
            Ptc_Handle_Error();

        // DEBUG
        // Print state of FSM
        #if (CFG_DEBUG_FSM_STATE == CFG_SETTING_ENABLED)
        static uint8_t fsm_state = (uint8_t)-1; // holds last assigned value (static)
        uint8_t new_fsm_state;
        if (fsm_state != (new_fsm_state = Ptc_Get_FSM_State()))
        {
            fsm_state = new_fsm_state;
            Log_Val_Hex32("s:", fsm_state);
        }
        #endif // #if (CFG_DEBUG_FSM_STATE == CFG_SETTING_ENABLED)
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



