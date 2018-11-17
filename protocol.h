/*
 * protocol.h
 *
 *  Created on: Nov 1, 2018
 *      Author: alvaro
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include <stdint.h>
#include <stdbool.h>

#include "rf_core.h"

// Communication buffer length
#define PRO_COMM_BUF_LEN            256

// Synchronization result number of fields and length (bytes)
#define PRO_SYNC_RES_FIELD_NUM      3
#define PRO_SYNC_RESULT_LEN         (sizeof(uint32_t) * PRO_SYNC_RES_FIELD_NUM)

// Delays and timeouts
#define PRO_DELAY_RAT_SYNC_PKT_TX       1024 // delay of start of transmission of synchronization packet
#define PRO_TOUT_SYNC_RESP_USEC         (100*1000) // 100 ms
#define PRO_TOUT_SYNC_START_USEC        (1000*1000) // 1 s
#define PRO_DELAY_RAT_SYNC_RESP_TX      4092*4 // delay of start of transmission of synchronization response

// Maximum consecutive errors
#define PRO_MAX_ERROR_COUNT         1

// Operation modes
#define PRO_MODE_SLV                0 // default
#define PRO_MODE_MSTR               1

// Flags
#define PRO_F_SYNC_DONE             0x01

// Common FSM states
typedef enum
{
    PRO_S_IDLE = 0,
    PRO_S_WAIT_RF_CORE_INIT,
    PRO_S_WAIT_RAT_SYNC,
} pro_common_state_t;

// Sensor node protocol FSM states
typedef enum
{
    PRO_S_SLV_START_RX = 0x10,
    PRO_S_SLV_WAIT_SYNC_PKT,
    PRO_S_SLV_WAIT_RESP_SENT,
    PRO_S_SLV_WAIT_SYNC_RESULT,
} pro_sensor_state_t;

// Sink node protocol FSM states
typedef enum
{
    PRO_S_MSTR_SEND_SYNC_PKT = 0x80,
    PRO_S_MSTR_WAIT_SYNC_PKT_SENT,
    PRO_S_MSTR_WAIT_SYNC_RESP,
    PRO_S_MSTR_WAIT_SYNC_RESULT_SENT,
} pro_sink_state_t;

typedef struct
{
    uint32_t rtc_t0;
    uint32_t rat_t0,
             rat_t1,
             rat_t2,
             rat_t3,
             rat_t4;
} timestamps_t;

// Structure to hold the state of the protocol module
typedef struct
{
    bool mode;
    uint8_t flags;
    int state; // TODO use proper data type (pro_state_t)
    uint8_t error_count;

    uint8_t* comm_buf_p;
    rfc_tx_param_t tx_param;
    rfc_rx_result_t rx_result;

    timestamps_t ts;
} pro_control_t;

void Pro_Init();

void Pro_Common_FSM();

void Pro_Sensor_Node_FSM();

void Pro_Sink_Node_FSM();

void (*Pro_Process)();

void Pro_Handle_Error();

bool Pro_Set_Mode(bool mode);

bool Pro_Start_Synch();

bool Pro_Ready();

bool Pro_Sync_Done();

#endif /* PROTOCOL_H_ */
