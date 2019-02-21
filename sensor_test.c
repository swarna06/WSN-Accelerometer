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

//#include <stdio.h>

#include "sensor_test.h"
#include "spi_bus.h"
#include "timing.h"
#include "board.h"
#include "log.h"

#include "printf.h"

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

    const uint8_t ADDR_FILTER = 0x28;
    const uint8_t ADDR_POWER_CTL = 0x2D;
    const uint8_t ADDR_RESET = 0x2F;

    const uint8_t RESET_CODE = 0x52;

    Sen_Single_Byte_Write(ADDR_RESET, RESET_CODE); // reset sensor
    Sen_Single_Byte_Write(ADDR_FILTER, 0x03); // set ODR
    Sen_Single_Byte_Write(ADDR_POWER_CTL, 0x00); // start measurement
}

void Sen_Read_Acc_Test(int16_t* abuf)
{
//    while(1)
    {
        uint8_t addr;
        uint8_t devid_ad = 0, devid_ms = 0, partid = 0, revid = 0, status = 0,xdata1 =0,xdata2=0,xdata3=0,ydata1 =0,ydata2=0,ydata3=0,zdata1 =0,zdata2=0,zdata3=0;
        uint8_t fifo_entries = 0;
        int16_t temp1=0,temp2=0,temp = 0;
        int16_t xdata = 0;
        int16_t  ydata = 0, zdata = 0;
        uint8_t power_ctl = 0;

        addr = 0x00; // DEVID_AD
        Sen_Single_Byte_Read(addr, &devid_ad);

        addr = 0x01; // DEVID_MST
        Sen_Single_Byte_Read(addr, &devid_ms);

        addr = 0x02; // PARTID
        Sen_Single_Byte_Read(addr, &partid);

        addr = 0x03; // REVID
        Sen_Single_Byte_Read(addr, &revid);

        addr = 0x04; // Status
        Sen_Single_Byte_Read(addr, &status);

        addr = 0x05; // FIFO_ENTRIES
        Sen_Single_Byte_Read(addr, &fifo_entries);

        addr = 0x80|0x06; // TEMP2
        Sen_Single_Byte_Read(addr + 0x80, (int8_t*)&temp2);

        addr = 0x80|0x07; // TEMP1
        Sen_Single_Byte_Read(addr + 0x80, (int8_t*)&temp1);

        temp = temp2<<8|temp1;

        addr = 0x08; // XDATA3
        Sen_Single_Byte_Read(addr , (int8_t*)&xdata3);

        addr = 0x09; // XDATA2
        Sen_Single_Byte_Read(addr, (int8_t*)&xdata2);

        addr = 0x0A; // XDATA1
        Sen_Single_Byte_Read(addr, (int8_t*)&xdata1);

      //  xdata =xdata3<<12|xdata2<<4|xdata1>>4;
        xdata = xdata3<<8|xdata2;
        addr = 0x0B; // YDATA3
        Sen_Single_Byte_Read(addr , (int8_t*)&ydata3);

        addr = 0x0C; // YDATA2
        Sen_Single_Byte_Read(addr, (int8_t*)&ydata2);

        addr = 0x0D; // YDATA1
        Sen_Single_Byte_Read(addr, (int8_t*)&ydata1);

      //  ydata = (int)ydata3<<12|(int)ydata2<<4|(int)ydata1>>4;
       // if(ydata & (1 << 20 - 1))
           // ydata = ydata - (1 << 20);
        ydata = ydata3<<8|ydata2;

        addr = 0x0E; // ZDATA3
        Sen_Single_Byte_Read(addr , (int8_t*)&zdata3);

        addr = 0x0F; // ZDATA2
        Sen_Single_Byte_Read(addr, (int8_t*)&zdata2);

        addr = 0x10; // ZDATA1
        Sen_Single_Byte_Read(addr, (int8_t*)&zdata1);

      //  zdata = (int)zdata3<<12|(int)zdata2<<4|(int)zdata1>>4;
      //  if(zdata & (1 << 20 - 1))
           // zdata = zdata - (1 << 20);
        zdata = zdata3<<8|zdata2;
        addr = 0x2D; // POWER_CTL
        Sen_Single_Byte_Read(addr, &power_ctl);
        abuf[0]= (int)temp;
        abuf[1]= (int)xdata;
        abuf[2]= (int)ydata;
        abuf[3]= (int)zdata;


 /*     Log_Value_Int(temp);Log_String_Literal(",");
        Log_Value_Int(xdata);Log_String_Literal(",");
        Log_Value_Int(ydata);Log_String_Literal(",");
        Log_Value_Int(zdata);
        Log_Line(" ");*/
    }
}
