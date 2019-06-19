
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
volatile uint32_t pfl_tic, pfl_toc, pfl_wcet = 0, d_tm=0;
uint32_t str=0;
volatile bool drdy_set = false,delay_ovr = false;
void Sen_ISR();
void DRDY_ISR();
void GPIO_Init();
static int32_t d_rdy=0;
int main(void)
{
    int32_t abuf[4],i=0;
    uint8_t r_abuf[16], num[4] ={1, 2, 3, 4}, dummy[16]={0};
    bool sync_given = false,  memread=false;
    uint32_t exec_time,curtime, wcet = 0,st=0,en=0, addr=1,r_addr=1;

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
    Mem_Init();


    #if (CFG_DEBUG_RFC_ERR_BUTTON == CFG_SETTING_ENABLED)
    // DEBUG TODO remove
    IOCPinTypeGpioInput(BRD_GPIO_IN0);
    IOCPinTypeGpioInput(BRD_GPIO_IN1);
    IOCIOPortPullSet(BRD_GPIO_IN0, IOC_IOPULL_UP);
    IOCIOPortPullSet(BRD_GPIO_IN1, IOC_IOPULL_UP);
    #endif // #if (CFG_DEBUG_RFC_ERR_BUTTON == CFG_SETTING_ENABLED)

/*
 * Interrupt settings for ext sync pulse train
 */
        IOCPinTypeGpioInput(BRD_GPIO_IN1);
        IOCIOPortPullSet(BRD_GPIO_IN1, IOC_IOPULL_UP);
        IOCIOIntSet(BRD_GPIO_IN1, IOC_INT_ENABLE, IOC_RISING_EDGE);
#if (CFG_DEBUG_EXT_SYNC == CFG_SETTING_ENABLED)
        IOCIOIntSet(BRD_SEN_INT1, IOC_INT_ENABLE, IOC_FALLING_EDGE);
#endif
        IOCIntRegister(Sen_ISR);

#if (CFG_DEBUG_MEM_WRITE == CFG_SETTING_DISABLED)

    Log_Line("Waiting for button...");
    while (GPIO_readDio(BRD_GPIO_IN0)) {Log_Process();};
    Log_Line("Test starting in 2 secs");
    //Delay of 2,000,000 us = 2 sec
    Brd_Led_On(BRD_LED1);
    st = Pfl_Ticks_To_Microsec(Pfl_Get_Current_Time());
    while(Pfl_Ticks_To_Microsec(Pfl_Get_Current_Time()) - st < 2000000)
    {Log_Process();} ;
    Brd_Led_Off(BRD_LED1);

#endif // #if (CFG_DEBUG_MEM_WRITE == CFG_SETTING_DISABLED)

#if (CFG_DEBUG_MEM_WRITE == CFG_SETTING_ENABLED)
    Tm_Start_Period(TM_PER_HEARTBEAT_ID, TM_PER_HEARTBEAT_VAL);
    Log_Value_Uint(Mem_Flag_Read());Log_String_Literal("\r\n");
    if(!Mem_Flag_Read())
        {
            Log_Line("Waiting for button...");
            while (GPIO_readDio(BRD_GPIO_IN0)) {Log_Process();};
            Log_Line("Test started");
        }
    else
        Log_Line("Enter 'r' to retrieve data");
#endif // #if (CFG_DEBUG_MEM_WRITE == CFG_SETTING_ENABLED)
    str = Pfl_Ticks_To_Microsec(Pfl_Get_Current_Time());
    Sen_Init();

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
       if((Pfl_Ticks_To_Microsec(Pfl_Get_Current_Time()) - str < 10000000) )
            {
               if(drdy_set)
                {
                   IOCIntDisable(BRD_SEN_INT1);
                   drdy_set=false;
                    IOCIntClear(BRD_SEN_INT1);
                    IOCIntEnable(BRD_SEN_INT1);
                    Brd_Led_On(BRD_LED0);
                    Sen_Read_Acc(abuf);
                  /*  #if (CFG_DEBUG_EXT_SYNC == CFG_SETTING_ENABLED)
                    if(!sync_given)
                    {
                        GPIO_setDio(BRD_GPIO_OUT2);
                        sync_given = true;
                    }
                    #endif*/
                    d_rdy++;
                  //d_tm=Pfl_Ticks_To_Microsec(Pfl_Get_Current_Time())-str;
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


//************** UART *****************
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
               for(int k=1; k<MAX_ADDR; k+=16)
                       {
                           Mem_Write(k,dummy,sizeof(dummy));
                       }
               Log_Value_Uint(Mem_Flag_Read());
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
#endif // #if (CFG_DEBUG_MEM_WRITE == CFG_SETTING_ENABLED)

 /* if (Pma_Batt_Volt_Meas_Ready())
          Pma_Process();

    Rfc_Process();

    if (Rfc_Ready())
          Ptc_Process()
    else if (Rfc_Error())
          Ptc_Handle_Error(); */




  //*********************LEVEL TRIGGER-UART LOG***************************
#if (CFG_DEBUG_MEM_WRITE == CFG_SETTING_DISABLED)
        if((Pfl_Ticks_To_Microsec(Pfl_Get_Current_Time()) - str < 10000000) && !delay_ovr )
        {
           // if(!GPIO_readDio(BRD_SEN_INT1))
                if(drdy_set)
                {
                    IOCIntDisable(BRD_SEN_INT1);
                    drdy_set=false;
                    IOCIntClear(BRD_SEN_INT1);
                    IOCIntEnable(BRD_SEN_INT1);
                    Sen_Read_Acc(abuf);
                    Brd_Led_On(BRD_LED0);
                  /*  #if (CFG_DEBUG_EXT_SYNC == CFG_SETTING_ENABLED)
                    if(!sync_given)
                    {
                        GPIO_setDio(BRD_GPIO_OUT2);
                        sync_given = true;
                    }
                    #endif*/
                   // Log_Value_Int(Pfl_Ticks_To_Microsec(Pfl_Get_Current_Time()) - str);Log_String_Literal(",");
                    Log_Value_Int(d_tm);Log_String_Literal(",");
                    Log_Value_Int(abuf[1]);Log_String_Literal(",");
                    Log_Value_Int(abuf[2]);Log_String_Literal(",");
                    Log_Value_Int(abuf[3]);Log_Line("");
                }
        }

        else
            {
                delay_ovr = true;
                Brd_Led_Off(BRD_LED0);

            }

#endif // #if (CFG_DEBUG_MEM_WRITE == CFG_SETTING_DISABLED)

    }

    return 0;
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

void Sen_ISR()
{
    d_tm=Pfl_Ticks_To_Microsec(Pfl_Get_Current_Time())-str;

        if (IOCIntStatus(BRD_GPIO_IN1))
        {
            GPIO_toggleDio(BRD_GPIO_OUT2);

        }
#if (CFG_DEBUG_EXT_SYNC == CFG_SETTING_ENABLED)
        else if (IOCIntStatus(BRD_SEN_INT1))
        {
            drdy_set = true;
        }
#endif
    IOCIntClear(BRD_GPIO_IN1);
    IOCIntClear(BRD_SEN_INT1);


}



