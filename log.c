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

static size_t itoa_udec(uint32_t value, char* result);
static size_t itoa_dec(int32_t value, char* result);
static size_t itoa_hex(uint32_t value, char* result);

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

void Log_Value_Uint(uint32_t val)
{
    size_t len = itoa_udec(val, (char*)value_str);
    Queue_Add(&log_queue, value_str, len);
}

void Log_Value_Int(int32_t val)
{
    size_t len = itoa_dec(val, (char*)value_str);
    Queue_Add(&log_queue, value_str, len);
}

void Log_Value_Hex(uint32_t val)
{
    size_t len = itoa_hex(val, (char*)value_str);
    Queue_Add(&log_queue, value_str, len);
}

/**
 * C++ version 0.4 char* style "itoa":
 * Written by Lukás Chmela
 * Released under GPLv3.
 *
 * Modified by Alvaro
 * Non-standard itoa version, adapted for the log module
 */
static size_t itoa_udec(uint32_t value, char* result)
{
    char* ptr = result, *ptr1 = result, tmp_char;
    uint32_t tmp_value;
    size_t count = 0;

    do {
        tmp_value = value;
        value /= LOG_DEC_BASE;
        *ptr++ = "9876543210123456789" [(LOG_DEC_BASE-1) + (tmp_value - value * LOG_DEC_BASE)];
        count++;
    } while ( value );

//    *ptr-- = '\0';
    ptr--;
    while(ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
    }
    return count;
}

static size_t itoa_dec(int32_t value, char* result)
{
    char* ptr = result, *ptr1 = result, tmp_char;
    int32_t tmp_value;
    size_t count = 0;

    do {
        tmp_value = value;
        value /= LOG_DEC_BASE;
        *ptr++ = "9876543210123456789" [(LOG_DEC_BASE-1) + (tmp_value - value * LOG_DEC_BASE)];
        count++;
    } while ( value );

    // Apply negative sign
    if (tmp_value < 0)
    {
        *ptr++ = '-';
        count++;
    }
//    *ptr-- = '\0';
    ptr--;
    while(ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
    }
    return count;
}

/**
 * C++ version 0.4 char* style "itoa":
 * Written by Lukás Chmela
 * Released under GPLv3.
 *
 * Modified by Alvaro
 * Non-standard itoa version, adapted for the log module
 */
static size_t itoa_hex(uint32_t value, char* result)
{
    char* ptr = result, *ptr1 = result, tmp_char;
    uint32_t tmp_value;
    size_t count = 0;

    do {
        tmp_value = value;
        value /= LOG_HEX_BASE;
        *ptr++ = "FEDCBA9876543210123456789ABCDEF" [(LOG_HEX_BASE-1) + (tmp_value - value * LOG_HEX_BASE)];
        count++;
    } while ( value );

//    *ptr-- = '\0';
    ptr--;
    while(ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
    }
    return count;
}
