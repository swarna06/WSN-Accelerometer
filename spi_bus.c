/*
 * spi_bus.c
 *
 *  Created on: Aug 31, 2018
 *      Author: alvaro
 */

#include <stdint.h>
//#include <stdio.h>

#include <driverlib/ioc.h>
#include <driverlib/gpio.h>
#include <driverlib/prcm.h>
#include <driverlib/ssi.h>
#include <driverlib/sys_ctrl.h>

#include "spi_bus.h"
//#include "myassert.h"
#include "board.h"
#include "log.h"
#include "printf.h"

static void Spi_HW_Setup()
{

    // Power serial interfaces
    PRCMPowerDomainOn(PRCM_DOMAIN_SERIAL); // TODO Should go to Startup code; also needed for I2C and SPI !
    while((PRCMPowerDomainStatus(PRCM_DOMAIN_SERIAL) != PRCM_DOMAIN_POWER_ON));

    // Power SPI
    PRCMPeripheralRunEnable(PRCM_PERIPH_SSI0);
    PRCMLoadSet();
    while(!PRCMLoadGet());

    // Map GPIOs to SPI signals
    // IOCPinTypeSsiMaster(ui32Base, ui32Rx, ui32Tx, ui32Fss, ui32Clk)
//    IOCPinTypeSsiMaster(SSI0_BASE, IOID_23, IOID_24, IOID_UNUSED, IOID_26);
    IOCPinTypeSsiMaster(SSI0_BASE, BRD_SPI_MISO, BRD_SPI_MOSI, IOID_UNUSED, BRD_SPI_CLK);

    // Configure and enable SPI
    // Master mode, CPOL = 0, CPHA = 0
    // xxx which clock should be used ? (not baud rate)
    // note: The peripheral clock is not necessarily the same as the processor clock.
    // The frequency of the peripheral clock is set by the system control.
    SSIConfigSetExpClk(SSI0_BASE, SysCtrlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER,
                       SPI_DATA_RATE, SPI_DATA_WIDTH);
    SSIEnable(SSI0_BASE);
}

void Spi_Init()
{
    Spi_HW_Setup();

   /* myprintf("Sen_HW_Setup: Ok, DATA_RATE: %u, DATA_WIDTH: %u\r\n",
             SPI_DATA_RATE, SPI_DATA_WIDTH);*/
    Log_Line("Sen HW Setup: OK");
}

void Spi_Init_CS_Pin(uint8_t pin)
{
    // CS (chip select) pin configured as output with initial state high
    GPIO_setDio(pin);
    IOCPinTypeGpioOutput(pin);
}

uint32_t Spi_Flush_Fifo()
{
    uint32_t dummy = 0; // this variable is used to force the compiler to read the register
    do
    {
        if (HWREG(SSI0_BASE + SSI_O_SR) & SSI_SR_RNE)
            dummy = HWREG(SSI0_BASE + SSI_O_DR); // read byte
    } while ((HWREG(SSI0_BASE + SSI_O_SR) & SSI_SR_BSY));
    return dummy; // value is returned to avoid warning
}

void Spi_Send(const uint8_t *src, const size_t src_count)
{
    if (src == NULL)
        return;

    for (size_t n = 0; n < src_count; n++)
    {
        while (!(HWREG(SSI0_BASE + SSI_O_SR) & SSI_SR_TNF)); // wait if TX FIFO is full
        HWREG(SSI0_BASE + SSI_O_DR) = src[n]; // send byte
    }
}

void Spi_Receive(uint8_t *dest, const size_t dest_cont)
{
    if (dest == NULL)
        return;

    size_t m = 0;

    // Receive the data
    for (size_t n = 0; n < dest_cont; n++)
    {
        while (!(HWREG(SSI0_BASE + SSI_O_SR) & SSI_SR_TNF)); // wait if TX FIFO is full
        HWREG(SSI0_BASE + SSI_O_DR) = 0; // send dummy byte

        if (HWREG(SSI0_BASE + SSI_O_SR) & SSI_SR_RNE) // RX FIFO not empty ?
        {
            dest[m] = HWREG(SSI0_BASE + SSI_O_DR); // pop value from FIFO
            m++;
        }
    }

    // Read the RX FIFO
    for (; m < dest_cont; m++)
    {
        while (!(HWREG(SSI0_BASE + SSI_O_SR) & SSI_SR_RNE)); // wait for data
        dest[m] = (uint8_t)HWREG(SSI0_BASE + SSI_O_DR);
    }
}

void Spi_Transaction(const uint8_t *src,
                     const size_t src_count,
                     uint8_t *dest,
                     const size_t dest_count,
                     const uint32_t cs_pin)
{
    // FIXME include guard time for consecutive transactions
    Spi_Assert_CS(cs_pin);
    Spi_Send(src, src_count);
    Spi_Flush_Fifo();
    Spi_Receive(dest, dest_count);
    Spi_Deassert_CS(cs_pin);
}


