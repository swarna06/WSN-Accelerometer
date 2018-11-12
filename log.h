/*
 * log.h
 *
 *  Created on: Nov 1, 2018
 *      Author: alvaro
 */

#ifndef LOG_H_
#define LOG_H_

#include <stdlib.h>

#define LOG_VAL_BASE                16

// Macros for logging
#define Log_String_Literal(s)       (sizeof(s) > 1 ? Log : Log_Dummy)(s, sizeof(s) - 1) // excludes '\0'
#define Log_Line(l)                 Log_String_Literal(l "\r\n")
#define Log_Value(s,v)              { \
                                        Log_String_Literal(s); \
                                        Log_Value_Hex(v); \
                                        Log_String_Literal("\r\n"); \
                                    }
#define Log_Char(s,c)               { \
                                        Log_String_Literal(s); \
                                        Log(&c, 1); \
                                        Log_String_Literal("\r\n"); \
                                    }


// Log buffer length
#define LOG_BUF_LEN                 1024 // bytes

void Log_Init();

void Log_Process();

void Log(void* data, size_t data_len);

void Log_Value_Hex(uint32_t val);

void Log_Dummy(void* data, size_t data_len);

#endif /* LOG_H_ */
