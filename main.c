
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

// v0.1.7a
//#define DRIVERLIB_NOROM // uncomment to disable ROM

#include <driverlib/prcm.h>
#include <driverlib/sys_ctrl.h>
#include <driverlib/ioc.h>

//#include <ti/devices/DeviceFamily.h>
//#include DeviceFamily_constructPath(driverlib/ioc.h)
//#include <ti/boards/CC2640R2_LAUNCHXL/CC2640R2_LAUNCHXL.h>
//#include <ti/drivers/GPIO.h>
//#include <ti/drivers/gpio/GPIOCC26XX.h>

#include <driverlib/gpio.h>
#include <driverlib/timer.h>

#include <driverlib/aon_event.h>

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


// Global profiling variables
volatile uint32_t pfl_tic, pfl_toc, pfl_wcet = 0;
uint32_t str=0; //for profiling
void GPIO_Init();
void Sen_ISR(uint_least8_t index);
static int32_t d_rdy=0;
int main(void)
{
    int32_t abuf[4];
    uint8_t r_abuf[4], num[4] ={1, 2, 3, 4}, data;
    uint32_t addr=0;
    bool sync_given = false;
    bool delay_ovr = false;
    uint32_t exec_time,curtime, wcet = 0,st=0,en=0;


    // Modules' initialization
    //Pma_Init();

    GPIO_Init();
    Tm_Init();

    Sep_Init();
    Log_Init();
    Pfl_Init();

    //Rfc_Init();
    //Ptc_Init();

    Spi_Init();
    Sen_Init();
    Mem_Init();

    #if (CFG_DEBUG_RFC_ERR_BUTTON == CFG_SETTING_ENABLED)
    // DEBUG TODO remove
    IOCPinTypeGpioInput(BRD_GPIO_IN0);
    IOCPinTypeGpioInput(BRD_GPIO_IN1);
    IOCIOPortPullSet(BRD_GPIO_IN0, IOC_IOPULL_UP);
    IOCIOPortPullSet(BRD_GPIO_IN1, IOC_IOPULL_UP);
    #endif // #if (CFG_DEBUG_RFC_ERR_BUTTON == CFG_SETTING_ENABLED)

    Tm_Start_Period(TM_PER_HEARTBEAT_ID, TM_PER_HEARTBEAT_VAL);
    Tm_Start_Timeout(TM_TOUT_TEST_ID,TM_TOUT_TEST_VAL);
    Tm_Start_Timeout(TM_TOUT_SYNC_ID,TM_TOUT_SYNC_VAL);

   //IntEnable(INT_AON_GPIO_EDGE);
   //IntRegister(INT_AON_GPIO_EDGE, Sen_ISR);
   //IOCIOIntSet(9,IOC_INT_ENABLE,IOC_FALLING_EDGE);
   //IOCPortConfigureSet(IOID_9,IOC_PORT_GPIO,IOC_FALLING_EDGE|IOC_INT_ENABLE);
    str = Pfl_Ticks_To_Microsec(Pfl_Get_Current_Time());
    GPIO_setDio(BRD_GPIO_OUT1);  //To enable start of test connect to ref sensor via BNC cable
    Brd_Led_On(BRD_LED0);
   /* GPIO_init();
    GPIO_write(GPIOCC26XX_DIO_06,1);
    GPIO_setConfig(GPIOCC26XX_DIO_09, GPIO_CFG_IN_PU|GPIO_CFG_IN_INT_FALLING);
    GPIO_setCallback(GPIOCC26XX_DIO_09, &Sen_ISR);
    GPIO_enableInt(GPIOCC26XX_DIO_09);*/
    // Round-robin scheduling (circular execution, no priorities)

    while (1)
    {

        if (Tm_Sys_Tick())
            Tm_Process();

        Log_Process();

 /* if (Pma_Batt_Volt_Meas_Ready())
          Pma_Process();

    Rfc_Process();

    if (Rfc_Ready())
          Ptc_Process()
    else if (Rfc_Error())
          Ptc_Handle_Error(); */



          //  Pfl_Tic();
          /*  Pfl_Toc();
              exec_time = Pfl_Get_Exec_Time_Microsec();
              Log_Value_Int(exec_time);Log_Line(" ");*/

              //  Log_Value_Int(d_rdy);Log_Line(" ");
               if(Tm_Period_Completed(TM_PER_HEARTBEAT_ID))
                   {
                   //Mem_Test();
                 //  Sen_Read_Acc(abuf);
                /*   Log_Value_Int(abuf[0]);Log_String_Literal(",");
                   Log_Value_Int(abuf[1]);Log_String_Literal(",");
                   Log_Value_Int(abuf[2]);Log_String_Literal(",");
                   Log_Value_Int(abuf[3]);
                   Log_Line(" ");*/
                   //to convert uint32 to uint8
                  /* uint32_t value;
                   uint32_t result[4];

                   result[0] = (value & 0x000000ff);
                   result[1] = (value & 0x0000ff00) >> 8;
                   result[2] = (value & 0x00ff0000) >> 16;
                   result[3] = (value & 0xff000000) >> 24;*/
                   Mem_Write(addr,num,4); //addr starts at 00000h till 7FFFFh
                   Mem_Read(addr,r_abuf,4);
                   Log_Value_Hex(addr); Log_String_Literal(",");
                   for(int j =0; j<4; j++)
                   {
                       Log_Value_Int(r_abuf[j]);Log_String_Literal(",");
                   }
                   Log_Line(" ");
                   addr++;
                   }
  //*********************LEVEL TRIGGER***************************

      /*  if((Pfl_Ticks_To_Microsec(Pfl_Get_Current_Time()) - str < 1000000) && !delay_ovr)
        { // Mem_Test();
        if(!GPIO_readDio(9))
                {
                    Sen_Read_Acc(abuf);
                    d_rdy++;
                    Log_Value_Int(Pfl_Ticks_To_Microsec(Pfl_Get_Current_Time()));Log_String_Literal(",");
                    //Log_Value_Int(d_rdy);Log_Line(" ");
                    Log_Value_Int(abuf[1]);Log_String_Literal(",");
                    Log_Value_Int(abuf[2]);Log_String_Literal(",");
                    Log_Value_Int(abuf[3]);Log_Line("");
                 /* if(d_rdy==1)
                    {
                        st = Pfl_Ticks_To_Microsec(Pfl_Get_Current_Time());
                        Brd_Led_Toggle(BRD_LED0);
                    }
                    else if(d_rdy == 1000)
                    {
                        Log_Value_Int(Pfl_Ticks_To_Microsec(Pfl_Get_Current_Time()) - st);
                        Brd_Led_Toggle(BRD_LED0);
                    }
                }
        }
        else
            {
                delay_ovr = true;
                Brd_Led_Off(BRD_LED0);
            }
/*

  //*******************EDGE TRIGGER*****************************

  /* if((Pfl_Ticks_To_Microsec(Pfl_Get_Current_Time()) - str < 5000000) && !delay_ovr)
        {
           Sen_Read_Acc(abuf);
           Log_Value_Int(d_rdy);Log_Line(" ");
        }
   else
               delay_ovr = true;
        if(IOCIntStatus(9))
        {
            Sen_Read_Acc(abuf);
          //  d_rdy++;
           // Log_Value_Int(d_rdy);Log_Line("");
            if(d_rdy==1)
            {
                Log_Value_Int(Pfl_Ticks_To_Microsec(Pfl_Get_Current_Time()));Log_Line("");
                Brd_Led_Toggle(BRD_LED0);
            }
            else if(d_rdy == 1000)
            {
                Log_Value_Int(Pfl_Ticks_To_Microsec(Pfl_Get_Current_Time()));
                Brd_Led_Toggle(BRD_LED0);
            }
            //Delay of 100 us
            //st = Pfl_Ticks_To_Microsec(Pfl_Get_Current_Time());
            //while(Pfl_Ticks_To_Microsec(Pfl_Get_Current_Time()) - st < 100)
            //  ;

        }
        */

    #if (CFG_DEBUG_EXT_SYNC == CFG_SETTING_ENABLED)
        if(Tm_Timeout_Completed(TM_TOUT_SYNC_ID))
        {
            if(!sync_given)
                {
                    GPIO_setDio(BRD_GPIO_OUT2);
                    sync_given = true;
                    Log_Value_Int(Pfl_Ticks_To_Microsec(Pfl_Get_Current_Time()));Log_Line("");
                }
        }
    #endif // #if (CFG_DEBUG_EXT_SYNC == CFG_SETTING_ENABLED)
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
    IOCPinTypeGpioOutput(BRD_GPIO_OUT1);
    IOCPinTypeGpioOutput(BRD_GPIO_OUT2);
    IOCPinTypeGpioInput(9);
  //  IOCIOPortPullSet(9, IOC_IOPULL_DOWN);

    // Set initial values
    Brd_Led_Off(BRD_LED0);
    Brd_Led_Off(BRD_LED1);
    GPIO_clearDio(BRD_GPIO_OUT1);
    GPIO_clearDio(BRD_GPIO_OUT2);

}

void Sen_ISR(uint_least8_t index)
{
    d_rdy++;
}

