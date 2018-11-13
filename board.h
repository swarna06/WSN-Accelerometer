/*
 * board.h
 *
 *  Created on: Jul 9, 2018
 *      Author: alvaro
 */

#ifndef BOARD_H_
#define BOARD_H_

#define BOARD_LAUNCHPAD     0
#define BOARD_ES            1
#define BOARD_SENSORTAG     2

#define BOARD_USED  BOARD_LAUNCHPAD

#ifndef BOARD_USED
    #define BOARD_USED BOARD_LAUNCHPAD
#endif  // BOARD_USED

#if (BOARD_USED == BOARD_LAUNCHPAD)

    #define BRD_LED0            6
    #define BRD_LED1            7

    #define BRD_DEBUG_PIN0      21
    #define BRD_DEBUG_PIN1      22

    #define BRD_GPIO_IN0         13
    #define BRD_GPIO_IN1         14

    #define BRD_UART_TX         3
    #define BRD_UART_RX         2

#elif (BOARD_USED == BOARD_ES)

    #define BRD_LED0            13
    #define BRD_LED1            14

    #define BRD_DEBUG_PIN0      11
    #define BRD_DEBUG_PIN1      12

    #define BRD_GPIO_IN0
    #define BRD_GPIO_IN1

    #define BRD_UART_TX         4

#endif  // #if (BOARD_USED == BOARD_LAUNCHPAD)

#endif /* BOARD_H_ */
