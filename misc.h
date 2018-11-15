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
                                PRINTF("Assertion violation: file " __FILE__ ", line " TO_STR(__LINE__) ": " #__e "\r\n"); \
                                exit(0); \
                            }
#else
    #define assertion(__e)
#endif

// Macro for checking if a pointer is memory aligned
#define is_aligned(POINTER, BYTE_COUNT) \
        (((uintptr_t)(const void *)(POINTER)) % (BYTE_COUNT) == 0)


#endif /* MISC_H_ */
