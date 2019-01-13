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

#include "configuration.h"

// ********************************
// Timing
// ********************************
// Timeout values (milliseconds)
#define RFC_TOUT_DEFAULT                0
#define RFC_TOUT_BOOT_MSEC              100
#define RFC_TOUT_CPE_READY_MSEC         100
#define RFC_TOUT_CPE_ACK_MSEC           100 // xxx
#define RFC_TOUT_MAX_OP_TIME_MSEC       1000 // 1 second xxx
#define RFC_TOUT_TX_MSEC                100 // maximum TX time
#define RFC_TOUT_RX_MSEC                100 // minimum RX time

// RF Core wake up time
#define RFC_WAKEUP_TIME_USEC            1000
#define RFC_WAKEUP_TIME_MSEC            1

// Macro functions for interfacing with the RAdio Timer (RAT)
#define Rfc_Get_RAT_Time()              HWREG(RFC_RAT_BASE + RFC_RAT_O_RATCNT)

// RAT time conversion
#define RFC_RAT_NSEC_PER_TICK           250
#define RFC_RAT_USEC_PER_TICK           (RFC_RAT_NSEC_PER_TICK * 1000)
#define RFC_RAT_MSEC_PER_TICK           (RFC_RAT_USEC_PER_TICK * 1000)

#define RFC_RAT_TICKS_PER_USEC          (1000 / RFC_RAT_NSEC_PER_TICK)
#define RFC_RAT_TICKS_PER_MSEC          (RFC_RAT_TICKS_PER_USEC * 1000)

#define RFC_RAT_TICKS_PER_RTC_TICK      (122)

// RAT output modes
#define RFC_RAT_OUTPUT_PULSE            (0b000)
#define RFC_RAT_OUTPUT_TOGGLE           (0b011)
#define RFC_RAT_OUTPUT_SET              (0b001)
#define RFC_RAT_OUTPUT_CLEAR            (0b010)
#define RFC_RAT_OUTPUT_ALWAYS_1         (0b101)
#define RFC_RAT_OUTPUT_ALWAYS_0         (0b100)

// ********************************
// RF Core interface
// ********************************
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

// Radio operation status field error flag
#define RFC_F_RADIO_OP_STATUS_ERR       0x0800

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

// ********************************
// BLE 5
// ********************************
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
#define RFC_PHY_CODING_500KBPS      1
#define RFC_PHY_CODING_125KBPS      0

// BLE5 frequency channel base values
#define RFC_BLE5_BASE_FREQ          2402 // frequency channel 37 (channel offset = 0)
#define RFC_BLE5_BASE_CH            0x66 // id channel 37 (channel offset = 0)
#define RFC_BLE5_BASE_WHITE_INIT    0x40 // whitening initial value for channel 0

// ********************************
// Transmission and reception data structures and constants
// ********************************
// Maximum payload length
#define RFC_MAX_PAYLOAD_LEN             254

// RX result flags
#define RFC_F_RX_CRC_ERR                0x01
#define RFC_F_RX_TOUT_ERR               0x02

// RX buffer
#define RFC_RX_BUF_LEN                  512
#define RFC_RX_BUF_PAYLOAD_LEN_IDX      1
#define RFC_RX_BUF_PAYLOAD_OFFSET       3

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

// ********************************
// Error handling
// ********************************
// Error codes
typedef enum
{
    RFC_ERR_NONE = 0,
    RFC_ERR_BOOT_FAILED,
    RFC_ERR_OPERATION_FAILED,
    RFC_ERR_INTERNAL,
    RFC_ERR_SYNTH_NO_LOCK,
    RFC_ERR_TIMEOUT,
} rfc_error_code_t;

// Error structure
typedef struct
{
    uint8_t code;
    uint8_t fsm_state;
    uint16_t cmd_num;
    uint16_t cmd_status;
    uint32_t CMDSTA; // copy of register value
    uint32_t RFHWIFG; // copy of register value
    uint32_t RFCPEIFG; // copy of register value
} rfc_error_t;

// ********************************
// State machine
// ********************************
// Control structure flags
#define RFC_F_INITIALIZED           0x01

// RF Core FSM states
typedef enum
{
    RFC_S_IDLE = 0x00,

    RFC_S_WAIT_RFC_BOOT = 0x10,
    RFC_S_EXEC_RADIO_SETUP,
    RFC_S_EXEC_FS,
    RFC_S_EXEC_START_RAT,
    RFC_S_EXEC_SYNC_START_RAT,

    RFC_S_WAIT_CPE_READY = 0x20,
    RFC_S_WAIT_CPE_ACK,
    RFC_S_WAIT_RADIO_OP_EXECUTION,

    RFC_S_WAIT_ERR_ACTION = 0x30,

} rfc_state_t;

// RF Core control structure
typedef struct
{
    int state, next_state;
    uint8_t flags, set_flags_on_success;
    volatile rfc_command_t* immediate_cmd_p;
    volatile rfc_radioOp_t* radio_op_p;
    uint32_t radio_op_cpe_err_flags;
    uint16_t op_timeout;
    rfc_error_t error;
} rfc_control_t;

// Macros for making more readable the state machine TODO move to inline functions ?
#define Rfc_Initialized()               (rfc.flags & RFC_F_INITIALIZED)

#define Rfc_Set_Flags_On_Success(f)     (rfc.set_flags_on_success |= f)

#define Rfc_Start_Immediate_Cmd(c)      {\
                                            rfc.immediate_cmd_p = (rfc_command_t*)c; \
                                            rfc.error.code = 0; \
                                            rfc.state = RFC_S_WAIT_CPE_READY; \
                                            Tm_Start_Timeout(TM_RFC_TOUT_ID, RFC_TOUT_CPE_READY_MSEC); \
                                        }

#define Rfc_Start_Direct_Cmd(c)         Rfc_Start_Immediate_Cmd(CMDR_DIR_CMD(c));
                                        // NOTE: direct commands are immediate commands without parameters

/*//#define Rfc_Start_Radio_Op(op, to)      {\
//                                            rfc.radio_op_p = (rfc_radioOp_t*)op; \
//                                            rfc.radio_op_p->status = IDLE; \
//                                            rfc.error.code = 0; \
//                                            rfc.state = RFC_S_WAIT_CPE_READY; \
//                                            Tm_Start_Timeout(TM_RFC_TOUT_ID, RFC_TOUT_CPE_READY_MSEC); \
//                                            rfc.op_timeout = to; \
//                                        } */

#define Rfc_On_Success_Do(op)             { \
                                            rfc.flags |= rfc.set_flags_on_success; \
                                            rfc.set_flags_on_success = 0; \
                                            op = NULL; \
                                        }

// ********************************
// Public functions
// ********************************
void Rfc_Init();

void Rfc_Wakeup();

void Rfc_Process();

uint8_t Rfc_Get_FSM_State();

void Rfc_Set_Tx_Power(rfc_tx_power_t tx_power);

void Rfc_BLE5_Set_PHY_Mode(uint8_t ble5_mode);

void Rfc_BLE5_Set_Channel(uint8_t channel);

bool Rfc_BLE5_Adv_Aux(rfc_tx_param_t* tx_param_p);

bool Rfc_BLE5_Scanner(uint32_t rat_start_time, uint32_t timeout_usec);

void Rfc_BLE5_Get_Scanner_Result(rfc_rx_result_t* dest);

bool Rfc_Synchronize_RAT();

bool Rfc_Set_RAT_Compare(uint8_t rat_ch, uint32_t rat_compare_time);

bool Rfc_Set_RAT_Output(uint8_t rat_ch, uint8_t output_sel, uint8_t output_mode);

bool Rfc_Ready();

uint8_t Rfc_Error();

void Rfc_Enable_Output_Signals();

#endif /* RF_CORE_H_ */
