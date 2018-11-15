/*
 * log.c
 *
 *  Created on: Nov 1, 2018
 *      Author: alvaro
 */

#include <driverlib/uart.h>

#include "log.h"
#include "queue.h"

static uint8_t value_str[16];
static uint8_t log_queue_buf[LOG_BUF_LEN];
static queue_t log_queue;

/**
 * C++ version 0.4 char* style "itoa":
 * Written by Lukás Chmela
 * Released under GPLv3.
 *
 * Modified by Alvaro
 * Non-standard itoa version, adapted for the log module
 */
static size_t myitoa(int value, char* result)
{
    char* ptr = result, *ptr1 = result, tmp_char;
    int tmp_value;
    size_t count = 0;

    do {
        tmp_value = value;
        value /= LOG_VAL_BASE;
        *ptr++ = "FEDCBA9876543210123456789ABCDEF" [15 + (tmp_value - value * LOG_VAL_BASE)];
        count++;
    } while ( value );

    // Apply negative sign
    if (tmp_value < 0)
    {
        *ptr++ = '-';
        count++;
    }
    *ptr-- = '\0';
    while(ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
    }
    return count;
}

void Log_Init()
{
    Queue_Init(&log_queue, log_queue_buf, sizeof(log_queue_buf));
}

void Log_Process()
{
    if (!(HWREG(UART0_BASE + UART_O_FR) & UART_FR_TXFF)) // UART TX FIFO is not full?
    {
        uint8_t datum;
        if (Queue_Remove(&log_queue, &datum, 1) == 1)
            HWREG(UART0_BASE + UART_O_DR) = datum;
        else
            return; // Queue is empty
    }
}

void Log(void* data, size_t data_len)
{
    Queue_Add(&log_queue, data, data_len);
}

void Log_Value_Hex(uint32_t val)
{
    size_t len = myitoa(val, (char*)value_str);
    Queue_Add(&log_queue, value_str, len);
}

// Function to avoid calling Log() when length of string literal is 0 (excluding '\0')
inline void Log_Dummy(void* data, size_t data_len)
{
    (void)0; // do nothing
}


