/*
 * serial_port.c
 *
 *  Created on: 21 oct. 2018
 *      Author: Alvaro
 */

#include <driverlib/prcm.h>
#include <driverlib/sys_ctrl.h>
#include <driverlib/uart.h>

void Sep_UART_Init()
{
    // Power UART
    PRCMPeripheralRunEnable(PRCM_PERIPH_UART0);
    PRCMLoadSet();
    while(!PRCMLoadGet());

    // Configure and enable UART
    UARTConfigSetExpClk(UART0_BASE, SysCtrlClockGet(), 1000000,
                        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    UARTFIFOEnable(UART0_BASE);
    UARTEnable(UART0_BASE);

    // Clear screen
    UARTCharPut(UART0_BASE, '\f');
}

