/*
 * serial_port.c
 *
 *  Created on: 15 oct. 2018
 *      Author: Alvaro
 */

#include <stdlib.h>

#include <driverlib/ioc.h>
#include <driverlib/gpio.h>
#include <driverlib/prcm.h>
#include <driverlib/timer.h>
#include <driverlib/uart.h>
#include <driverlib/sys_ctrl.h>

#include "board.h"
#include "serial_port.h"

void Sep_Init()
{
    // Power serial interfaces
    PRCMPowerDomainOn(PRCM_DOMAIN_SERIAL); // TODO Should go to Startup code; also needed for I2C and SPI !
    while((PRCMPowerDomainStatus(PRCM_DOMAIN_SERIAL) != PRCM_DOMAIN_POWER_ON));

    // Power UART
    PRCMPeripheralRunEnable(PRCM_PERIPH_UART0);
    PRCMLoadSet();
    while(!PRCMLoadGet());

    // Map GPIOs to UART signals
    IOCPinTypeUart(UART0_BASE, BRD_UART_RX, BRD_UART_TX, IOID_UNUSED, IOID_UNUSED);

    // Configure and enable UART
    UARTConfigSetExpClk(UART0_BASE, SysCtrlClockGet(), SEP_BAUD_RATE,
                        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    UARTFIFOEnable(UART0_BASE); // enable UART FIFOs
    UARTEnable(UART0_BASE);
}

