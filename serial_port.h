/*
 * serial_port.h
 *
 *  Created on: 15 oct. 2018
 *      Author: Alvaro
 */

#ifndef SERIAL_PORT_H_
#define SERIAL_PORT_H_

#define SEP_BAUD_RATE       921600

void Sep_Init();

void Sep_Wakeup();

bool Sep_UART_Busy();

#endif /* SERIAL_PORT_H_ */
