
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

#include "sync_test.h"

// Global profiling variables
volatile uint32_t pfl_tic, pfl_toc, pfl_wcet = 0;

void GPIO_Init();

void Sync_Testbed_Isr();
void Sync_Testbed();

int main(void)
{
    // Modules' initialization
    Pma_Init();
    GPIO_Init();
    Tm_Init();

    Sep_Init();
    Log_Init();
    Pfl_Init();

    Sync_Testbed();

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

    Tm_Start_Period(TM_PER_HEARTBEAT_ID, 200);

    Sts_Init();

    // Round-robin scheduling (circular execution, no priorities)
    while (1)
    {
        if (Tm_Period_Completed(TM_PER_HEARTBEAT_ID))
        {
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

        Sts_Process();

        // DEBUG
        // Print state of FSM
        #if (CFG_DEBUG_FSM_STATE == CFG_SETTING_ENABLED)
        static uint8_t fsm_state = (uint8_t)-1; // holds last assigned value (static)
        uint8_t new_fsm_state;
        if (fsm_state != (new_fsm_state = Sts_Get_FSM_State()))
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

#if (BRD_BOARD != BRD_SENSOR_NODE_V1)

#define NODE_NUM    4

const uint32_t gpio_pin[NODE_NUM] = {19, 18, 8, 9};

volatile uint32_t timestamps[NODE_NUM];
volatile size_t idx = 0;
volatile bool done = false;

void Sync_Testbed_Isr()
{
    uint32_t curr_time = Pfl_Get_Current_Time();

    for (size_t i = 0; i < NODE_NUM; i++)
    {
        if (IOCIntStatus(gpio_pin[i]))
        {
            timestamps[idx] = curr_time;
            IOCIntClear(gpio_pin[i]);
            IOCIntDisable(gpio_pin[i]);

            idx++;
            if (idx >= NODE_NUM)
            {
                done = true;
                idx = 0;
                return;
            }
        }
    }

//    if (IOCIntStatus(BRD_GPIO_IN0))
//        Log_Line("BRD_GPIO_IN0");
//
//    if (IOCIntStatus(BRD_GPIO_IN1))
//        Log_Line("BRD_GPIO_IN1");
//
//    IOCIntClear(BRD_GPIO_IN0);
//    IOCIntClear(BRD_GPIO_IN1);
//    Log_Val_Uint32("curr_time: ", curr_time);
}

#define TOUT_SYNC                   (5*1000 + 100)    // 5 seconds
#define TOUT_SIGNALS                (1000 + 100)        // 10 milliseconds

#define MAX_SYNC_ERR_NANOSEC        (1000 * 50)

#define SYT_S_WAIT_SYNC_TOUT        0
#define SYT_S_WAIT_MAX_SYNC_ERR     1

void Sync_Testbed()
{
    Brd_Led_On(BRD_LED0); // start beacons
    Brd_Led_On(BRD_LED1);

    IOCPinTypeGpioInput(BRD_GPIO_IN0);
    IOCPinTypeGpioInput(BRD_GPIO_IN1);
    IOCIOPortPullSet(BRD_GPIO_IN0, IOC_IOPULL_UP);
    IOCIOPortPullSet(BRD_GPIO_IN1, IOC_IOPULL_UP);

    for (size_t i = 0; i < NODE_NUM; i++)
        IOCPinTypeGpioInput(gpio_pin[i]);

    for (size_t i = 0; i < NODE_NUM; i++)
        IOCIOIntSet(gpio_pin[i], IOC_INT_ENABLE, IOC_RISING_EDGE);

    IOCIntRegister(Sync_Testbed_Isr);

    // Wait until button is pressed
    Log_Line("Waiting for button...");
    while (GPIO_readDio(BRD_GPIO_IN0)) {Log_Process();};
    Log_Line("Test started");

    // Start heartbeat period and sync timeout
    uint8_t state = SYT_S_WAIT_SYNC_TOUT;
    Tm_Start_Period(TM_PER_HEARTBEAT_ID, 1000);
    Tm_Start_Timeout(TM_TOUT_TEST_ID, TOUT_SYNC);

    bool first = true;
    uint32_t rtc_start_time = 0;

    while (1)
    {
        if (Tm_Sys_Tick())
            Tm_Process();

        if (Tm_Period_Completed(TM_PER_HEARTBEAT_ID))
        {
            Brd_Led_Toggle(BRD_LED1);
        }

        Log_Process();

        switch (state)
        {
        case SYT_S_WAIT_SYNC_TOUT:

            // If timeout completed enable interrupts, stop beacons and go to next step
            if (Tm_Timeout_Completed(TM_TOUT_TEST_ID))
            {
                done = false;
                first = true;

                // re-enable interrupts
                for (size_t i = 0; i < NODE_NUM; i++)
                {
                    IOCIntClear(gpio_pin[i]);
                    IOCIntEnable(gpio_pin[i]);
                }

                Brd_Led_Off(BRD_LED0); // stop beacons
                Tm_Start_Timeout(TM_TOUT_TEST_ID, TOUT_SIGNALS);
                Log_Line("--------------------");
                Log_Line("Round started!");
                Log_Line("");

                state = SYT_S_WAIT_MAX_SYNC_ERR;
            }
            break;

        case SYT_S_WAIT_MAX_SYNC_ERR:

            if (done == true)
            {
                if (first == true)
                {
                    first = false;
                    rtc_start_time = Tm_Get_RTC_Time();
                    Log_Val_Uint32("rtc_start_time: ", rtc_start_time);
                }

                done = false;
                Tm_Start_Timeout(TM_TOUT_TEST_ID, TOUT_SIGNALS);

                // print results
                uint32_t sync_err = Pfl_Delta_Time32(timestamps[0], timestamps[NODE_NUM-1]);
                sync_err = Pfl_Ticks_To_Nanosec(sync_err);
                Log_Val_Uint32("sync_err[ns]: ", sync_err);

                // Check if max error was exceeded and act accordingly
                if (sync_err < MAX_SYNC_ERR_NANOSEC)
                {
                    // re-enable interrupts
                    for (size_t i = 0; i < NODE_NUM; i++)
                    {
                        IOCIntClear(gpio_pin[i]);
                        IOCIntEnable(gpio_pin[i]);
                    }
                }
                else
                {
                    uint32_t rtc_end_time = Tm_Get_RTC_Time();
                    uint32_t rtc_delta = Pfl_Delta_Time32(rtc_start_time, rtc_end_time);
                    Log_Line("");
                    Log_Val_Uint32("rtc_end_time: ", rtc_end_time);
                    Log_Val_Uint32("rtc_delta: ", rtc_delta);

                    Brd_Led_On(BRD_LED0); // re-start beacons
                    Tm_Start_Timeout(TM_TOUT_TEST_ID, TOUT_SYNC);
                    Log_Line("Round finished!");

                    state = SYT_S_WAIT_SYNC_TOUT;
                }
            }
            else if (Tm_Timeout_Completed(TM_TOUT_TEST_ID))
            {
                assertion(!"signals timeout");
            }

            break;

        default:
            assertion(!"invalid state");
        }
    }
}

#endif
