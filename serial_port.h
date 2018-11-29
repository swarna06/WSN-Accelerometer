/*
 * serial_port.h
 *
 *  Created on: 15 oct. 2018
 *      Author: Alvaro
 */

#ifndef SERIAL_PORT_H_
#define SERIAL_PORT_H_

#define SEP_BAUD_RATE       115200

void Sep_Init();

void Sep_Wakeup();

bool Sep_UART_Idle();

#endif /* SERIAL_PORT_H_ */
