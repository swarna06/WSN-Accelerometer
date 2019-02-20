/*
 * board.h
 *
 *  Created on: Jul 9, 2018
 *      Author: alvaro
 */

#ifndef BOARD_H_
#define BOARD_H_

#include <driverlib/gpio.h>

#define BRD_LAUNCHPAD               1
#define BRD_SENSOR_NODE_V1          2

#define BRD_BOARD                   BRD_LAUNCHPAD
//#define BRD_BOARD                   BRD_SENSOR_NODE_V1

#define Brd_Led_Toggle(l)           GPIO_toggleDio(l)

#if (BRD_BOARD == BRD_LAUNCHPAD)

    #define BRD_LED0                6
    #define BRD_LED1                7

    #define BRD_GPIO_OUT0           21

    #define BRD_GPIO_IN0            13
    #define BRD_GPIO_IN1            14

    #define BRD_UART_TX             3
    #define BRD_UART_RX             2

    #define BRD_SLEEP_PIN           BRD_LED0
    #define BRD_LF_OSC_PIN          21
    #define BRD_RFC_TXOUT_PIN       22
    #define BRD_RFC_RXOUT_PIN       23
    #define BRD_RTC_OUT_PIN         BRD_LED1

    #define BRD_SPI_CLK     15
    #define BRD_SPI_MOSI    25
    #define BRD_SPI_MISO    26

    #define BRD_MEM_CS      1
    #define BRD_SEN_CS      0   //using this

    // LEDs are active high
    #define Brd_Led_On(l)           GPIO_setDio(l)
    #define Brd_Led_Off(l)          GPIO_clearDio(l)

#elif (BRD_BOARD == BRD_SENSOR_NODE_V1)

    #define BRD_LED0                13
    #define BRD_LED1                14

    #define BRD_GPIO_OUT0           11

    #define BRD_GPIO_IN0
    #define BRD_GPIO_IN1

    #define BRD_UART_TX             4
    #define BRD_UART_RX             5

    // LEDs are active low
    #define Brd_Led_On(l)           GPIO_clearDio(l)
    #define Brd_Led_Off(l)          GPIO_setDio(l)

#endif  // #if (BOARD_USED == BOARD_LAUNCHPAD)

#endif /* BOARD_H_ */
