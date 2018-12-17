
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

    // Enable debug output signals according to configuration
    Tm_Enable_LF_Clock_Output();
    Rfc_Enable_Output_Signals();

    // Start heart beat period
    Tm_Start_Period(TM_PER_HEARTBEAT_ID, TM_PER_HEARTBEAT_VAL);

    // Variable to keep track of the state of a module's FSM
    #if (CFG_DEBUG_FSM_STATE == CFG_SETTING_ENABLED)
    uint8_t fsm_state = (uint8_t)-1;
    uint8_t new_fsm_state = fsm_state;
    #endif // #if (CFG_DEBUG_FSM_STATE == CFG_SETTING_ENABLED)

    // Round-robin scheduling (circular execution, no priorities)
    while (1)
    {
        if (Tm_Period_Completed(TM_PER_HEARTBEAT_ID))
        {
//            Brd_Led_Toggle(BRD_LED1); // heart beat
        }

        if (Tm_Sys_Tick())
            Tm_Process();

        Log_Process();

        Rfc_Process();

        #if (CFG_DEBUG_FSM_STATE == CFG_SETTING_ENABLED)
        if (fsm_state != (new_fsm_state = Ptc_Get_FSM_State()))
        {
            fsm_state = new_fsm_state;
            Log_Val_Hex32("s:", fsm_state);
        }
        #endif // #if (CFG_DEBUG_FSM_STATE == CFG_SETTING_ENABLED)

        if (Rfc_Ready())
            Ptc_Process();
        else if (Rfc_Error())
            Ptc_Handle_Error();
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



