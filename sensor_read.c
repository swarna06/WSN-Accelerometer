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

#include "printf.h"

static void Sen_HW_Clock_Setup(uint32_t timer_base)
{
    const uint32_t IOID = 10;   //DIO Pin on chip
    const uint32_t TIMER_LOAD_VAL = 46; //46 = ~1.024 MHz

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
    // Clock settings
    Sen_HW_Clock_Setup(GPT2_BASE);

    const uint8_t ADDR_FILTER = 0x28;
    const uint8_t ADDR_POWER_CTL = 0x2D;
    const uint8_t ADDR_RESET = 0x2F;
    const uint8_t ADDR_RANGE = 0x2C;
    const uint8_t RESET_CODE = 0x52;
    const uint8_t ADDR_EXTSYNC = 0x2B;

    Sen_Single_Byte_Write(ADDR_RESET, RESET_CODE); // reset sensor
    Sen_Single_Byte_Write(ADDR_FILTER, 0x02); // set ODR 1000Hz
    Sen_Single_Byte_Write(ADDR_RANGE, 0x01);  //Set Range +/-2g
    Sen_Single_Byte_Write(ADDR_EXTSYNC, 0x05); //Set full external synchronization 00000101
    Sen_Single_Byte_Write(ADDR_POWER_CTL, 0x04); // start measurement
}


void Sen_Read_Acc(int32_t* abuf)
{
        uint8_t addr, status = 0,power_ctl = 0, xdata1 =0,xdata2=0,xdata3=0,ydata1 =0,ydata2=0,ydata3=0,zdata1 =0,zdata2=0,zdata3=0;
        int16_t temp1=0,temp2=0,temp = 0;
        int32_t xdata = 0, ydata = 0, zdata = 0;

    //-----------REGISTER ACCESS-------------------------

        addr = 0x04; // Status
        Sen_Single_Byte_Read(addr, &status);

//--------------------------DATA ACCESS---------------------------------

        addr = 0x80|0x06; // TEMP2
        Sen_Single_Byte_Read(addr + 0x80, (int8_t*)&temp2);

        addr = 0x80|0x07; // TEMP1
        Sen_Single_Byte_Read(addr + 0x80, (int8_t*)&temp1);
        temp = temp2<<8|temp1;

        if (status & 1 == 1)
          {
            for(int y=0; y<5;y++)
               abuf[y]=1;

                addr = 0x08; // XDATA3
                Sen_Single_Byte_Read(addr , (int8_t*)&xdata3);

                addr = 0x09; // XDATA2
                Sen_Single_Byte_Read(addr, (int8_t*)&xdata2);

                addr = 0x0A; // XDATA1
                Sen_Single_Byte_Read(addr, (int8_t*)&xdata1);

                xdata =xdata3<<12|xdata2<<4|xdata1>>4;
                if(xdata & (1 << 20 - 1))
                    xdata = xdata - (1 << 20);

                addr = 0x0B; // YDATA3
                Sen_Single_Byte_Read(addr , (int8_t*)&ydata3);

                addr = 0x0C; // YDATA2
                Sen_Single_Byte_Read(addr, (int8_t*)&ydata2);

                addr = 0x0D; // YDATA1
                Sen_Single_Byte_Read(addr, (int8_t*)&ydata1);

                ydata = (int)ydata3<<12|(int)ydata2<<4|(int)ydata1>>4;
                if(ydata & (1 << 20 - 1))
                    ydata = ydata - (1 << 20);

                addr = 0x0E; // ZDATA3
                Sen_Single_Byte_Read(addr , (int8_t*)&zdata3);

                addr = 0x0F; // ZDATA2
                Sen_Single_Byte_Read(addr, (int8_t*)&zdata2);

                addr = 0x10; // ZDATA1
                Sen_Single_Byte_Read(addr, (int8_t*)&zdata1);

                zdata = (int)zdata3<<12|(int)zdata2<<4|(int)zdata1>>4;
                if(zdata & (1 << 20 - 1))
                    zdata = zdata - (1 << 20);

                     /* abuf[1]= (int)xdata;
                        abuf[2]= (int)ydata;
                        abuf[3]= (int)zdata; */
           }
        else
            for(int y=0; y<5;y++)
                abuf[y]=0;
}