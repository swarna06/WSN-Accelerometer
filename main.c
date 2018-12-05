
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

//#define DRIVERLIB_NOROM
#define SET_CCFG_IEEE_BLE_0

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
    Pma_Init();
    GPIO_Init();
    Tm_Init();

    Sep_Init(); // FIXME disable to reduce power consumption
    Log_Init();
    #if (BRD_BOARD == BRD_LAUNCHPAD)
    Pfl_Init(); // FIXME disable to reduce power consumption
    #endif

    Rfc_Init();
    Ptc_Init();

    Brd_Led_Off(BRD_LED1);

    Tm_Start_Period(TM_PER_HEARTBEAT_ID, TM_PER_HEARTBEAT_VAL);

    uint8_t fsm_state = (uint8_t)-1;
    uint8_t new_fsm_state = fsm_state;

    // Enable debug output signals according to configuration
#if (CFG_DEBUG_LF_OSC_OUT == CFG_SETTING_ENABLED)
    Tm_Enable_LF_Clock_Output();
#endif
#if (CFG_DEBUG_RADIO_OUT == CFG_SETTING_ENABLED)
    Rfc_Enable_Output_Signals();
#endif

    while (1)
    {
        if (Tm_Period_Completed(TM_PER_HEARTBEAT_ID))
        {
//            Brd_Led_Toggle(BRD_LED1);
        }

        if (Tm_Sys_Tick())
            Tm_Process();

        Log_Process();

        Rfc_Process();

        if (fsm_state != (new_fsm_state = Ptc_Get_FSM_State()))
        {
            fsm_state = new_fsm_state;
            Log_Val_Hex32("s:", fsm_state);
        }

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



