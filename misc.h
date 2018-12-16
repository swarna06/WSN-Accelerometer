/*
 * misc.h
 *
 *  Created on: 21 oct. 2018
 *      Author: Alvaro
 */

#ifndef MISC_H_
#define MISC_H_

#include <stdlib.h>

#include "printf.h"

// Assert macro
#define ENABLE_ASSERT

#define TO_STR_HELPER(l)    #l
#define TO_STR(l)           TO_STR_HELPER(l)

#if defined (ENABLE_ASSERT)
    #define assertion(__e)  if (__e) (void)0;   \
                            else \
                            { \
                                PRINTF("\r\nAssertion violation: file " __FILE__ ", line " TO_STR(__LINE__) ": " #__e "\r\n"); \
                                exit(0); \
                            }
#else
    #define assertion(__e)
#endif

// Macro for checking if a pointer is memory aligned
#define is_aligned(POINTER, BYTE_COUNT) \
        (((uintptr_t)(const void *)(POINTER)) % (BYTE_COUNT) == 0)

// Pseudo-random number generation
extern uint16_t lfsr;

inline void Lfsr_Seed(uint16_t seed)
{
    lfsr = seed;
}

inline uint16_t Lfsr_Fibonacci()
{
    uint16_t bit = ((lfsr >> 0) ^ (lfsr >> 2) ^ (lfsr >> 3) ^ (lfsr >> 5) ) & 1;
    lfsr =  (lfsr >> 1) | (bit << 15);
    return lfsr;
}

#endif /* MISC_H_ */
