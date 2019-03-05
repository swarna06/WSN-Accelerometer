/*
 * log.h
 *
 *  Created on: Nov 1, 2018
 *      Author: alvaro
 */

#ifndef LOG_H_
#define LOG_H_

#include <stdlib.h>

// Numeral systems base
#define LOG_DEC_BASE                10
#define LOG_HEX_BASE                16

// Macros for logging
//#define Log_String_Literal(s)       (sizeof(s) > 1 ? Log : Log_Dummy)(s, sizeof(s) - 1) // excludes '\0'
#define Log_String_Literal(s)       if (sizeof(s) > 1) Log(s, sizeof(s) - 1); // excludes '\0'
#define Log_Line(l)                 Log_String_Literal(l "\r\n")
#define Log_Val_Uint32(s,v)         { \
                                        Log_String_Literal(s); \
                                        Log_Value_Uint(v); \
                                        Log_String_Literal("\r\n"); \
                                    }
#define Log_Val_Int32(s,v)          { \
                                        Log_String_Literal(s); \
                                        Log_Value_Int(v); \
                                        Log_String_Literal("\r\n"); \
                                    }
#define Log_Val_Hex32(s,v)          { \
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

void Log_Value_Uint(uint32_t val);

void Log_Value_Int(int32_t val);

void Log_Value_Hex(uint32_t val);

#endif /* LOG_H_ */
