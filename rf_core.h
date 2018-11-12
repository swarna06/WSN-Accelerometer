/*
 * rf_core.h
 *
 *  Created on: Oct 29, 2018
 *      Author: alvaro
 */

#ifndef RF_CORE_H_
#define RF_CORE_H_

#include <stdint.h>
#include <stdbool.h>

#include <driverlib/rf_common_cmd.h>
#include <inc/hw_rfc_rat.h>

#include "misc.h"

// Timeout values (milliseconds)
#define RFC_TOUT_DEFAULT                0
#define RFC_TOUT_BOOT_MSEC              100
#define RFC_TOUT_CPE_READY_MSEC         100
#define RFC_TOUT_CPE_ACK_MSEC           100 // xxx
#define RFC_TOUT_MAX_OP_TIME_MSEC       1000 // 1 second xxx
#define RFC_TOUT_TX_MSEC                100 // Maximum TX time
#define RFC_TOUT_RX_MSEC                100 // Minimum RX time

// RX buffer length
#define RFC_RX_BUF_LEN                  512

// Error codes
#define RFC_ERR_NONE                    0

// CPE command done interrupt flags
#define RFC_M_CPE_COMMAND_DONE          (RFC_DBELL_RFCPEIFG_LAST_COMMAND_DONE | \
                                         RFC_DBELL_RFCPEIFG_COMMAND_DONE)

// CPE TX interrupt flags
#define RFC_M_CPE_TX_INT_FLAGS          (RFC_DBELL_RFCPEIFG_TX_BUFFER_CHANGED | \
                                        RFC_DBELL_RFCPEIFG_TX_ENTRY_DONE | \
                                        RFC_DBELL_RFCPEIFG_TX_RETRANS | \
                                        RFC_DBELL_RFCPEIFG_TX_CTRL_ACK_ACK | \
                                        RFC_DBELL_RFCPEIFG_TX_CTRL_ACK | \
                                        RFC_DBELL_RFCPEIFG_TX_CTRL | \
                                        RFC_DBELL_RFCPEIFG_TX_ACK | \
                                        RFC_DBELL_RFCPEIFG_TX_DONE)
// CPE TX interrupt flags
#define RFC_M_CPE_RX_INT_FLAGS          (RFC_DBELL_RFCPEIFG_RX_ABORTED | \
                                         RFC_DBELL_RFCPEIFG_RX_N_DATA_WRITTEN | \
                                         RFC_DBELL_RFCPEIFG_RX_DATA_WRITTEN | \
                                         RFC_DBELL_RFCPEIFG_RX_ENTRY_DONE | \
                                         RFC_DBELL_RFCPEIFG_RX_BUF_FULL | \
                                         RFC_DBELL_RFCPEIFG_RX_CTRL_ACK | \
                                         RFC_DBELL_RFCPEIFG_RX_CTRL | \
                                         RFC_DBELL_RFCPEIFG_RX_EMPTY | \
                                         RFC_DBELL_RFCPEIFG_RX_IGNORED | \
                                         RFC_DBELL_RFCPEIFG_RX_NOK | \
                                         RFC_DBELL_RFCPEIFG_RX_OK)

// Macro functions for interfacing with the Command and Packet Engine (CPE)
#define Rfc_Get_CPE_Int_Flags()         (HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG))
#define Rfc_Clear_CPE_Int_Flags(msk)    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = ~msk;
#define Rfc_CPE_Ready()                 (HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDR) == 0)
#define Rfc_Send_To_CPE(op)             HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDR) = (uint32_t)op;
#define Rfc_CPE_Ack()                   (HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG))
#define Rfc_Get_CPE_CMDSTA()            (HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDSTA))

// Macro functions for interfacing with the RAdio Timer (RAT)
#define Rfc_Get_RAT_Time()              HWREG(RFC_RAT_BASE + RFC_RAT_O_RATCNT)

// RAT time conversion
#define RAD_RAT_NSEC_PER_TICK           250
#define RAD_RAT_USEC_PER_TICK           (RAD_RAT_NSEC_PER_TICK * 1000)
#define RAD_RAT_MSEC_PER_TICK           (RAD_RAT_USEC_PER_TICK * 1000)

#define RAD_RAT_TICKS_PER_USEC          (1000 / RAD_RAT_NSEC_PER_TICK)
#define RAD_RAT_TICKS_PER_MSEC          (RAD_RAT_TICKS_PER_USEC * 1000)

#define RAD_RAT_TICKS_PER_RTC_TICK      (122)

// Radio operation status field error flag
#define RFC_F_RADIO_OP_STATUS_ERR       0x0800

// Macros for making more readable the state machine TODO move to inline functions ?
#define Rfc_Initialized()               (rfc.flags & RFC_F_INITIALIZED)

#define Rfc_Set_Flags_On_Success(f)     (rfc.set_flags_on_success |= f)

#define Rfc_Start_Immediate_Cmd(c)      {\
                                            rfc.immediate_cmd_p = (rfc_command_t*)c; \
                                            rfc.state = RFC_S_WAIT_CPE_READY; \
                                            Tm_Start_Timeout(TM_RFC_TOUT_ID, RFC_TOUT_CPE_READY_MSEC); \
                                        }

#define Rfc_Start_Direct_Cmd(c)         Rfc_Start_Immediate_Cmd(CMDR_DIR_CMD(c));
                                        // NOTE: direct commands are immediate commands without parameters

#define Rfc_Start_Radio_Op(op, to)      {\
                                            rfc.radio_op_p = (rfc_radioOp_t*)op; \
                                            rfc.radio_op_p->status = IDLE; \
                                            rfc.error = 0; \
                                            rfc.state = RFC_S_WAIT_CPE_READY; \
                                            Tm_Start_Timeout(TM_RFC_TOUT_ID, RFC_TOUT_CPE_READY_MSEC); \
                                            rfc.op_timeout = to; \
                                        }

#define Rfc_On_Success_Do(op)             { \
                                            rfc.flags |= rfc.set_flags_on_success; \
                                            rfc.set_flags_on_success = 0; \
                                            op = NULL; \
                                        }

// Transmission power codes (taken from SmartRF)
typedef enum
{
    RFC_TX_POW_PLUS_5dBm = 0x9330,
    RFC_TX_POW_PLUS_4dBm = 0x9324,
    RFC_TX_POW_PLUS_3dBm = 0x5A1C,
    RFC_TX_POW_PLUS_2dBm = 0x4E18,
    RFC_TX_POW_PLUS_1dBm = 0x4214,
    RFC_TX_POW_0dBm = 0x3161,
    RFC_TX_POW_MINUS_3dBm = 0x2558,
    RFC_TX_POW_MINUS_6dBm = 0x1D52,
    RFC_TX_POW_MINUS_9dBm = 0x194E,
    RFC_TX_POW_MINUS_12dBm = 0x144B,
    RFC_TX_POW_MINUS_15dBm = 0x0CCB,
    RFC_TX_POW_MINUS_18dBm = 0x0CC9,
    RFC_TX_POW_MINUS_21dBm = 0x0CC7,
} rfc_tx_power_t;

// BLE5 PHY modes
typedef enum
{
    RFC_PHY_MODE_2MBPS = 0,
    RFC_PHY_MODE_1MBPS,
    RFC_PHY_MODE_500KBPS,
    RFC_PHY_MODE_125KBPS,
    RFC_PHY_MODES_NUM,
} rfc_ble5_mode;

#define RFC_PHY_MAIN_MODE_1MBPS     0
#define RFC_PHY_MAIN_MODE_2MBPS     1
#define RFC_PHY_MAIN_MODE_CODED     2
#define RFC_PHY_CODING_NONE         0
#define RFC_PHY_CODING_500KBPS      2
#define RFC_PHY_CODING_125KBPS      8

// BLE5 channels
#define RFC_BLE5_BASE_FREQ          2402 // frequency channel 37 (channel offset = 0)
#define RFC_BLE5_BASE_CH            0x66 // id channel 37 (channel offset = 0)
#define RFC_BLE5_BASE_WHITE_INIT    0x40 // whitening initial value for channel 0

// RF Core FSM states
typedef enum
{
    RFC_S_IDLE = 0x00,
    RFC_S_PROCESS_PROP_TX_RESULT,
    RFC_S_PROCESS_PROP_RX_RESULT,

    RFC_S_WAIT_RFC_BOOT = 0x10,
    RFC_S_EXEC_RADIO_SETUP,
    RFC_S_EXEC_FS,
    RFC_S_EXEC_START_RAT,
    RFC_S_EXEC_SYNC_START_RAT,

    RFC_S_WAIT_CPE_READY = 0x20,
    RFC_S_WAIT_CPE_ACK,
    RFC_S_WAIT_RADIO_OP_EXECUTION,

} rfc_state_t;

// Flags
#define RFC_F_INITIALIZED           0x01

// RF Core control structure
typedef struct
{
    int state, next_state;
    uint8_t flags, set_flags_on_success;
    volatile rfc_command_t* immediate_cmd_p;
    volatile rfc_radioOp_t* radio_op_p;
    uint32_t radio_op_cpe_err_flags;
    uint16_t op_timeout;
    uint8_t error;
} rfc_control_t;

extern rfc_control_t rfc;

// TX parameter structure
typedef struct
{
    size_t len;
    uint8_t* buf;
    uint32_t rat_start_time;
} rfc_tx_param_t;

// RX result structure
typedef struct
{
    size_t payload_len, buf_len;
    void* buf;
    uint32_t rat_timestamp;
    int8_t rssi_db;
    uint8_t err_flags;
} rfc_rx_result_t;

// RX result flags
#define RFC_F_RX_CRC_ERR        0x01
#define RFC_F_RX_TOUT_ERR       0x02

// Macros for printing error information
#define Rfc_Print_CPE_Err() \
        PRINTF("ERR: state: %d, RFCPEIFG: %p, CMDSTA: %p\r\n", \
               rfc.state, (void*)HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG), \
               (void*)HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDSTA))

#define Rfc_Print_Timeout_Err() \
        PRINTF("ERR: timeout\r\n")

#define Rfc_Print_Radio_Op_Err(op_p) \
        PRINTF("ERR: state: %d, commandNo: %04X, CMDSTA: %p, status: %04X, RFCPEIFG: %p\r\n", \
               rfc.state, (op_p)->commandNo, (void*)HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDSTA), \
               (op_p)->status, (void*)HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG))

#define Rfc_Print_Direct_Cmd_Err(id) \
        PRINTF("ERR: state: %d, commandNo: %04X, CMDSTA: %p, RFCPEIFG: %p\r\n", \
               rfc.state, (uint32_t)id, (void*)HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDSTA), \
               (void*)HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG))

void Rfc_Init();

void Rfc_Process();

void Rfc_Set_Tx_Power(rfc_tx_power_t tx_power);

void Rfc_Set_BLE5_PHY_Mode(uint8_t ble5_mode);

void Rfc_BLE5_Set_Channel(uint8_t channel);

bool Rfc_BLE5_Adv_Aux(rfc_tx_param_t* tx_param_p);

bool Rfc_Prop_Rx(uint32_t timeout_usec);

void Rfc_Get_Prop_Rx_Results(rfc_rx_result_t* dest);

bool Rfc_Synchronize_RAT();

bool Rfc_Ready();

uint8_t Rfc_Error();

#endif /* RF_CORE_H_ */