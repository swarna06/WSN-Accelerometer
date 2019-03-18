
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

// v0.1.8
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
//#include "rf_core.h"
//#include "protocol.h"
#include "cp_engine.h"
#include "radio.h"
#include "log.h"
#include "power_management.h"
#include "profiling.h"
#include "configuration.h"

#include "misc.h"

// Global profiling variables
volatile uint32_t pfl_tic, pfl_toc, pfl_wcet = 0;

void rat_isr()
{
    Brd_Led_Toggle(BRD_LED1);

    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG) = ~RFC_DBELL_RFHWIFG_RATCH5;

    uint32_t radio_time = Rad_Get_Radio_Time();
    Log_Val_Uint32("event_time(us):", Rad_RAT_Ticks_To_Microsec(radio_time));
}

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

//    Rfc_Init();
//    Ptc_Init();

    Cpe_Init();
    Rad_Init();

    #if (CFG_DEBUG_RFC_ERR_BUTTON == CFG_SETTING_ENABLED)
    // DEBUG TODO remove
    IOCPinTypeGpioInput(BRD_GPIO_IN0);
    IOCPinTypeGpioInput(BRD_GPIO_IN1);
    IOCIOPortPullSet(BRD_GPIO_IN0, IOC_IOPULL_UP);
    IOCIOPortPullSet(BRD_GPIO_IN1, IOC_IOPULL_UP);
    #endif // #if (CFG_DEBUG_RFC_ERR_BUTTON == CFG_SETTING_ENABLED)

    Tm_Start_Period(TM_PER_HEARTBEAT_ID, 1000);
    Rad_Turn_On_Radio();

    uint32_t secondary_ble_addr_l = HWREG(CCFG_BASE + CCFG_O_IEEE_BLE_0);
    uint8_t dev_id = (uint8_t)secondary_ble_addr_l;
    Log_Val_Uint32("dev_id:", dev_id);

    bool wait_rat_event = false;
    bool rat_output_conf = false;

    IOCPortConfigureSet(BRD_LED1, IOC_PORT_RFC_GPO0, IOC_STD_OUTPUT);

    // Round-robin scheduling (circular execution, no priorities)
    while (1)
    {
        if (Tm_Period_Completed(TM_PER_HEARTBEAT_ID))
        {
            Brd_Led_Toggle(BRD_LED0);

            if (Rad_Radio_Is_On())
            {
                if (dev_id == 0) // sink ?
                {
                    if (rat_output_conf == true)
                    {
                        uint32_t radio_time = Rad_Get_Radio_Time();

                        Log_Val_Uint32("curr_time(us):", Rad_RAT_Ticks_To_Microsec(radio_time));

                        Rad_Set_RAT_Cmp_Val(radio_time + 100000, NULL);
                        wait_rat_event = true;
                    }
                    else
                    {
                        Rad_Set_RAT_Output();
                        rat_output_conf = true;
                    }
                }
            }
        }

        if (wait_rat_event == true && HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG) & RFC_DBELL_RFHWIFG_RATCH5)
        {
            HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG) = ~RFC_DBELL_RFHWIFG_RATCH5;

            uint32_t radio_time = Rad_Get_Radio_Time();
            Log_Val_Uint32("event_time(us):", Rad_RAT_Ticks_To_Microsec(radio_time));

            wait_rat_event = false;
        }

        if (Tm_Sys_Tick())
            Tm_Process();

        Log_Process();

        if (Pma_Batt_Volt_Meas_Ready())
            Pma_Process();

//        Rfc_Process();
//
//        if (Rfc_Ready())
//            Ptc_Process();
//        else if (Rfc_Error())
//            Ptc_Handle_Error();

        Cpe_Process();

        Rad_Process();

        // DEBUG
        // Print state of FSM
        #if (CFG_DEBUG_FSM_STATE == CFG_SETTING_ENABLED)
        static uint8_t fsm_state = (uint8_t)-1; // holds last assigned value (static)
        uint8_t new_fsm_state;
        if (fsm_state != (new_fsm_state = Cpe_Get_FSM_State()))
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



