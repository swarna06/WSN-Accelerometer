/*
 * sensor_test.c
 *
 *  Created on: Aug 20, 2018
 *      Author: alvaro
 */

#include <driverlib/ioc.h>

#include <driverlib/timer.h>
#include <driverlib/sys_ctrl.h>
#include <driverlib/event.h>
#include <driverlib/aon_rtc.h>
#include <driverlib/aux_timer.h>
#include <driverlib/aon_event.h>
#include <inc/hw_ccfg.h>
#include <sensor_read.h>

//#include <stdio.h>

#include "spi_bus.h"
#include "timing.h"
#include "board.h"
#include "log.h"
#include "configuration.h"
#include "printf.h"

static void Sen_HW_Clock_Setup(uint32_t timer_base)
{
    const uint32_t IOID = BRD_SEN_INT2;   //DIO_10 Pin on chip
    const uint32_t TIMER_LOAD_VAL = 46; //46 = ~1.024 MHz    48MHz/46 = ~1.024MHz

    // Set configuration parameters according to the timer number
    uint32_t port_id = 0, subscriber = 0, event_source = 0, periph_timer = 0;
    if (timer_base == GPT0_BASE)
    {
        port_id = IOC_PORT_MCU_PORT_EVENT0;
        subscriber = EVENT_O_GPT0ACAPTSEL;
        event_source = EVENT_GPT0ACAPTSEL_EV_PORT_EVENT0;
        periph_timer = PRCM_PERIPH_TIMER0;
    }
    else if (timer_base == GPT2_BASE)
    {
        port_id = IOC_PORT_MCU_PORT_EVENT4;
        subscriber = EVENT_O_GPT2ACAPTSEL;
        event_source = EVENT_GPT2ACAPTSEL_EV_PORT_EVENT4;
        periph_timer = PRCM_PERIPH_TIMER2;
    }

    // Map timer event to GPIO and register CPU event
    IOCPortConfigureSet(IOID, port_id, IOC_STD_OUTPUT);
    EventRegister(subscriber, event_source);

    // Enable timer peripheral
    PRCMPeripheralRunEnable(periph_timer);
    PRCMLoadSet();
    while(!PRCMLoadGet());

    // Configure and enable timer according to steps in the datasheet
    // 1. Ensure the timer is disabled (clear the TnEN bit) before making any changes.
    TimerDisable(timer_base, TIMER_BOTH);
    // 2. Write the GPTM Configuration Register (GPT:CFG) with a value of 0x0000 0004.
    HWREG(timer_base + GPT_O_CFG) = 0x00000004;
    // 3. In the GPTM Timer Mode Register (GPT:TnMR), write the TnCMR field to 0x1 and write the TnMR field to 0x2.
    HWREG(timer_base + GPT_O_TAMR) |= 0b1010;
    // 4. Configure the output state of the PWM signal (whether or not it is inverted) in the GPTM Control Register (GPT:CTL) TnPWML field.
    // 5. If a prescaler is to be used, write the prescale value to the GPTM Timer n Prescale Register (GPT:TnPR).
//    TimerPrescaleSet(timer_base, TIMER_A, 255); // xxx
//    TimerPrescaleMatchSet(timer_base, TIMER_A, 0); // xxx
    // 6. If PWM interrupts are used, configure the interrupt condition in the GPT:CTL TnEVENT register field, and enable the interrupts by setting the GPT:TnMR TnPWMIE register bit.
//    HWREG(timer_base + GPT_O_CTL) |= 0xC;
    HWREG(timer_base + GPT_O_CTL) |= 0x0; // positive edge
    HWREG(timer_base + GPT_O_TAMR) |= 0x200;
    // 7. Load the timer start value into the GPTM Timer n Interval Load Register (GPT:TnILR).
    TimerLoadSet(timer_base, TIMER_A, TIMER_LOAD_VAL);
    // 8. Load the GPTM Timer n Match Register (GPT:TnMATCHR) with the match value.
    TimerMatchSet(timer_base, TIMER_A, TIMER_LOAD_VAL/2); // to get 50% duty cycle
    // 9. Set the GPTM Control Register (GPT:CTL) TnEN bit to enable the timer and begin generation of the output PWM signal.
    TimerEnable(timer_base, TIMER_A);

}


static void Sen_Single_Byte_Read(uint8_t addr, uint8_t *dest)
{
    addr = (addr << 1) | 0x01; // read operation

    Spi_Transaction(&addr, sizeof(addr),
                    dest, sizeof(*dest),
                    SEN_SPI_CS_PIN);
}

static void Sen_Single_Byte_Write(uint8_t addr, uint8_t val)
{
    addr = (addr << 1) & (~0x01); // write operation
    const uint8_t frame[] = {addr, val};

    Spi_Transaction(frame, sizeof(frame),
                    NULL, 0,
                    SEN_SPI_CS_PIN);
}

void Sen_Init()
{
    // Configure chip select (CS) pin
    Spi_Init_CS_Pin(SEN_SPI_CS_PIN);
    const uint8_t RESET_CODE = 0x52;
    Sen_Single_Byte_Write(RESET, RESET_CODE); // reset sensor

    #if (CFG_DEBUG_EXT_SYNC == CFG_SETTING_ENABLED)
    Sen_Single_Byte_Write(SYNC, 0x05); //Set full external synchronization 00000101 with ext clk
    Sen_HW_Clock_Setup(GPT2_BASE);     //External Clock generation 1.024MHz INT2 pin
    #endif // #if (CFG_DEBUG_EXT_SYNC == CFG_SETTING_ENABLED)


    Sen_Single_Byte_Write(FILTER, 0x02); // set ODR 1000Hz
    Sen_Single_Byte_Write(RANGE, 0x02); //Set Range +/-2g
    Sen_Single_Byte_Write(INT_MAP, 0x01); // Map DATA_RDY interrupt enable on INT1 pin *** connect to DIO_9 pin of board
    Sen_Single_Byte_Write(POWER_CTL, 0x00); // start measurement
}


void Sen_Read_Acc(int32_t* abuf)
{
        uint8_t status = 0,power_ctl = 0, xdata1 =0,xdata2=0,xdata3=0,ydata1 =0,ydata2=0,ydata3=0,zdata1 =0,zdata2=0,zdata3=0,fifo_data[10] = {0};
        int16_t temp1=0,temp2=0,temp = 0;
        int32_t xdata = 0, ydata = 0, zdata = 0, fifo_acc[3]={0};
        static int dry=0;

//*************************REGISTER ACCESS*******************************

        Sen_Single_Byte_Read(STATUS, &status);

//**************************DATA ACCESS**********************************

        Sen_Single_Byte_Read(0x80|TEMP2 + 0x80, (int8_t*)&temp2);
        Sen_Single_Byte_Read(0x80|TEMP1 + 0x80, (int8_t*)&temp1);
        temp = temp2<<8|temp1;

        if (status & 1 == 1)
          {

                Sen_Single_Byte_Read(XDATA3,(int8_t*)&xdata3);
                Sen_Single_Byte_Read(XDATA2, (int8_t*)&xdata2);
                Sen_Single_Byte_Read(XDATA1, (int8_t*)&xdata1);
                xdata =(int)xdata3<<12|(int)xdata2<<4|(int)xdata1>>4;
                if(xdata & (1 << 20 - 1))
                    xdata = xdata - (1 << 20);

                Sen_Single_Byte_Read(YDATA3, (int8_t*)&ydata3);
                Sen_Single_Byte_Read(YDATA2, (int8_t*)&ydata2);
                Sen_Single_Byte_Read(YDATA1, (int8_t*)&ydata1);
                ydata = (int)ydata3<<12|(int)ydata2<<4|(int)ydata1>>4;
                if(ydata & (1 << 20 - 1))
                    ydata = ydata - (1 << 20);

                Sen_Single_Byte_Read(ZDATA3, (int8_t*)&zdata3);
                Sen_Single_Byte_Read(ZDATA2, (int8_t*)&zdata2);
                Sen_Single_Byte_Read(ZDATA1, (int8_t*)&zdata1);
                zdata = (int)zdata3<<12|(int)zdata2<<4|(int)zdata1>>4;
                if(zdata & (1 << 20 - 1))
                    zdata = zdata - (1 << 20);
                        abuf[0]= (int)temp;
                        abuf[1]= (int)xdata;
                        abuf[2]= (int)ydata;
                        abuf[3]= (int)zdata;
                        //Log_Value_Int(ydata);Log_Line("");

           }
        else
            for(int y=0; y<4; y++)
                abuf[y]=0;

//--------------------FIFO ACCESS--------------------------
   /*     if (status & 1 == 1)
          {
             for(int j=0; j<9; j++)
                {
                    Sen_Single_Byte_Read(FIFO_DATA, &fifo_data[j]);
                }


        for (int z = 0 ; z < 3; z++)
                {
                   fifo_acc[z] = (((int)fifo_data[z*3]<<12) | ((int)fifo_data[z*3 +1]<<4) | ((int)fifo_data[z*3 +2]>>4) );
                   if(fifo_acc[z] & (1 << 20 - 1))
                       fifo_acc[z] = fifo_acc[z] - (1 << 20);
                }

      Log_Value_Int(xdata);Log_String_Literal(",");
      Log_Value_Int(ydata);Log_String_Literal(",");
      Log_Value_Int(zdata);Log_String_Literal(",");
      Log_Value_Int(fifo_acc[0]);Log_String_Literal(",");
      Log_Value_Int(fifo_acc[1]);Log_String_Literal(",");
      Log_Value_Int(fifo_acc[2]);
      Log_Line(" ");
        }*/
}
