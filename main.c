
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


#define MAX_ADDR    0x7ffff

// Global profiling variables
volatile uint32_t pfl_tic, pfl_toc, pfl_wcet = 0;
uint32_t str=0; //for profiling
void Sen_ISR();
void GPIO_Init();
static int32_t d_rdy=0;
int main(void)
{
    int32_t abuf[4],d_tm=0, i=0;
    uint8_t r_abuf[16], num[4] ={1, 2, 3, 4}, dummy[16]={0};
    uint32_t addr=1,r_addr=1;
    bool sync_given = false;
    bool delay_ovr = false,memread=false;
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

    /*
     * Interrupt settings
     */
        IOCPinTypeGpioInput(BRD_GPIO_IN0);
        IOCIOPortPullSet(BRD_GPIO_IN0, IOC_IOPULL_UP);
        IOCIOIntSet(BRD_GPIO_IN0, IOC_INT_ENABLE, IOC_FALLING_EDGE);
        //IOCIntRegister(Sen_ISR);

  //  IOCPinTypeGpioInput(BRD_SEN_INT1);
  //  IOCIOIntSet(BRD_SEN_INT1,IOC_INT_ENABLE,IOC_FALLING_EDGE);
  //  IOCIntRegister(Sen_ISR);
   //IntEnable(INT_AON_GPIO_EDGE);
   //IntRegister(INT_AON_GPIO_EDGE, Sen_ISR);
   //IOCIOIntSet(9,IOC_INT_ENABLE,IOC_FALLING_EDGE);
   //IOCPortConfigureSet(IOID_9,IOC_PORT_GPIO,IOC_FALLING_EDGE|IOC_INT_ENABLE);

Log_Line("Enter 'r' to retrieve data:");
GPIO_setDio(BRD_GPIO_OUT1);  //To enable start of test connect to ref sensor via BNC cable

Brd_Led_On(BRD_LED1);
//Delay of 2,000,000 us = 2 sec
st = Pfl_Ticks_To_Microsec(Pfl_Get_Current_Time());
while(Pfl_Ticks_To_Microsec(Pfl_Get_Current_Time()) - st < 2000000)
{Log_Process();} ;
Brd_Led_Off(BRD_LED1);


    Tm_Start_Period(TM_PER_HEARTBEAT_ID, TM_PER_HEARTBEAT_VAL);
    Log_Value_Uint(Mem_Flag_Read());Log_String_Literal("\r\n");
    if(!Mem_Flag_Read())
    {
        Log_Line("Waiting for button...");
            while (GPIO_readDio(BRD_GPIO_IN0)) {Log_Process();};
            Log_Line("Test started");
    }
    str = Pfl_Ticks_To_Microsec(Pfl_Get_Current_Time());

    // Round-robin scheduling (circular execution, no priorities)

    while (1)
    {

        if (Tm_Sys_Tick())
            Tm_Process();

        Log_Process();
        //****************** MEMORY WRITE *****************************

#if (CFG_DEBUG_MEM_WRITE == CFG_SETTING_ENABLED)
 if(!Mem_Flag_Read()&& !delay_ovr)
 {
       if((Pfl_Ticks_To_Microsec(Pfl_Get_Current_Time()) - str < 1000000) )
            {
                if(!GPIO_readDio(BRD_SEN_INT1))
                {
                    Brd_Led_On(BRD_LED0);
                    Sen_Read_Acc(abuf);
                    d_rdy++;
                    d_tm=Pfl_Ticks_To_Microsec(Pfl_Get_Current_Time());
                    uint8_t data[] = {d_tm>>24,d_tm>>16,d_tm>>8,d_tm,((abuf[1]>>24)), ((abuf[1]>>16)), ((abuf[1]>>8)),abuf[1],((abuf[2]>>24)), ((abuf[2]>>16)), ((abuf[2]>>8)),abuf[2],((abuf[3]>>24)), ((abuf[3]>>16)), ((abuf[3]>>8)),abuf[3]};
                    Mem_Write(addr,data,sizeof(data)); //addr starts at 00001h till 7FFFFh
                    addr+=16;
                }
            }
        else
            {
                delay_ovr = true;
                Brd_Led_Off(BRD_LED0);
                Mem_Flag_Set();
            }
 }
#endif // #if (CFG_DEBUG_MEM_WRITE == CFG_SETTING_ENABLED)

//**************UART *****************
       if(UARTCharsAvail(UART0_BASE))
       {
           i = UARTCharGetNonBlocking(UART0_BASE);
           if(i=='r'&& Mem_Flag_Read())
           {
              Log_Line("Retrieving data from memory!");
            //  Brd_Led_On(BRD_LED0);
              r_addr=1;
              memread=true;
              Log_Value_Uint(Mem_Flag_Read());Log_String_Literal("\r\n");
           }
           else if(i=='c')
           {
               Log_Line("Clearing mem flag to start data collection for next run!");
               Mem_Flag_Reset();
               Log_Value_Uint(Mem_Flag_Read());
               for(int k=1; k<MAX_ADDR; k+=16)
                       {
                           Mem_Write(k,dummy,sizeof(dummy));
                       }
           }
           else if(i=='f')
           {
               Log_Line("mem flag");Log_Value_Uint(Mem_Flag_Read());
           }
       }

       if(memread)
       {
           if(Tm_Period_Completed(TM_PER_HEARTBEAT_ID)&&r_addr<MAX_ADDR)
                                 {
                                  Mem_Read(r_addr,r_abuf,sizeof(r_abuf));
                                  for(int j =0; j<sizeof(r_abuf);j+=4)
                                      {
                                         int32_t b = (int32_t)r_abuf[j];
                                         b=(b<<8)|(int32_t)r_abuf[j+1];
                                         b=(b<<8)|(int32_t)r_abuf[j+2];
                                         b=(b<<8)|(int32_t)r_abuf[j+3];
                                         if(b!=0)
                                             {
                                             Log_Value_Int(b);Log_String_Literal(",");
                                             }
                                         else
                                             {
                                             memread=false;
                                             Log_Line("over!");
                                             }
                                      }

                                      Log_Line(" ");
                                 r_addr+=16;
                                 }
       }

 /* if (Pma_Batt_Volt_Meas_Ready())
          Pma_Process();

    Rfc_Process();

    if (Rfc_Ready())
          Ptc_Process()
    else if (Rfc_Error())
          Ptc_Handle_Error(); */




  //*********************LEVEL TRIGGER-UART LOG***************************

 /*       if((Pfl_Ticks_To_Microsec(Pfl_Get_Current_Time()) - str < 10000000) && !delay_ovr )
        { // Mem_Test();
            if(!GPIO_readDio(BRD_SEN_INT1))
                {
                    Sen_Read_Acc(abuf);
                    d_rdy++;
                    Log_Value_Int(Pfl_Ticks_To_Microsec(Pfl_Get_Current_Time()));Log_String_Literal(",");
                    //Log_Value_Int(d_rdy);Log_Line(" ");
                    Log_Value_Int(abuf[1]);Log_String_Literal(",");
                    Log_Value_Int(abuf[2]);Log_String_Literal(",");
                    Log_Value_Int(abuf[3]);Log_Line("");
                /*  if(d_rdy==1)
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

void Sen_ISR()
{
        if (IOCIntStatus(BRD_GPIO_IN0))
          Log_Line("BRD_GPIO_IN0");
        IOCIntClear(BRD_GPIO_IN0);
        IntDisable(BRD_GPIO_IN0);
}


