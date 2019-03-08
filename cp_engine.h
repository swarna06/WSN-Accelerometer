/*
 * cp_engine.h
 *
 * Command and Packet Engine (CPE)
 *
 *  Created on: Mar 5, 2019
 *      Author: alvaro
 */

#ifndef CP_ENGINE_H_
#define CP_ENGINE_H_

#include <stdint.h>
#include <stdbool.h>

#include <driverlib/rfc.h>
#include <inc/hw_rfc_rat.h>

// ********************************
// Timing
// ********************************
// Timeout values (milliseconds)
enum
{
    CPE_TOUT_DEFAULT_MSEC = 10,
    CPE_TOUT_POWER_ON_MSEC = CPE_TOUT_DEFAULT_MSEC,
    CPE_TOUT_BOOT_MSEC = CPE_TOUT_DEFAULT_MSEC,
    CPE_TOUT_CPE_READY_MSEC = CPE_TOUT_DEFAULT_MSEC,
    CPE_TOUT_CPE_ACK_MSEC = CPE_TOUT_DEFAULT_MSEC,
    CPE_TOUT_RADIO_OP_MSEC = CPE_TOUT_DEFAULT_MSEC,
};

// Get radio timer (RAT) counter value
#define Cpe_Get_RAT_Time()              HWREG(RFC_RAT_BASE + RFC_RAT_O_RATCNT)

// ********************************
// RF Core interface
// ********************************
// CPE command done interrupt flags
#define CPE_M_CPE_COMMAND_DONE          (RFC_DBELL_RFCPEIFG_LAST_COMMAND_DONE | \
                                         RFC_DBELL_RFCPEIFG_COMMAND_DONE)

// CPE TX interrupt flags
#define CPE_M_CPE_TX_INT_FLAGS          (RFC_DBELL_RFCPEIFG_TX_BUFFER_CHANGED | \
                                        RFC_DBELL_RFCPEIFG_TX_ENTRY_DONE | \
                                        RFC_DBELL_RFCPEIFG_TX_RETRANS | \
                                        RFC_DBELL_RFCPEIFG_TX_CTRL_ACK_ACK | \
                                        RFC_DBELL_RFCPEIFG_TX_CTRL_ACK | \
                                        RFC_DBELL_RFCPEIFG_TX_CTRL | \
                                        RFC_DBELL_RFCPEIFG_TX_ACK | \
                                        RFC_DBELL_RFCPEIFG_TX_DONE)

// CPE TX interrupt flags
#define CPE_M_CPE_RX_INT_FLAGS          (RFC_DBELL_RFCPEIFG_RX_ABORTED | \
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

// Error interrupt flags
#define CPE_M_CPE_RF_CORE_ERR           (RFC_DBELL_RFCPEIFG_INTERNAL_ERROR | \
                                         RFC_DBELL_RFCPEIFG_SYNTH_NO_LOCK)
#define CPE_M_CPE_RX_ERR_FLAGS          (RFC_DBELL_RFCPEIFG_RX_NOK)

// Macro functions for interfacing with the Command and Packet Engine (CPE)
#define Cpe_Get_CPE_Int_Flags()         (HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG))
#define Cpe_Clear_CPE_Int_Flags(msk)    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = ~msk;
#define Cpe_CPE_Ready()                 (HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDR) == 0)
#define Cpe_Set_CMD_Reg(op)             HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDR) = (uint32_t)op;
#define Cpe_Clear_CPE_Ack()             HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG) = 0x0;
#define Cpe_CPE_Ack()                   (HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG))
#define Cpe_Get_CPE_CMDSTA()            (HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDSTA))

#define Cpe_Abort_Cmd_Execution()       Cpe_Set_CMD_Reg(CMDR_DIR_CMD(CMD_ABORT))

// Radio operation status field error flag
#define CPE_F_RADIO_OP_STATUS_ERR       0x0800

// ********************************
// Error handling
// ********************************
// Error codes
enum
{
    CPE_ERR_NONE = 0,
    CPE_ERR_POWER_ON_TOUT,
    CPE_ERR_BOOT_FAILED,
    CPE_ERR_RFC_UNRESPONSIVE,
    CPE_ERR_CMD_PARSING,
    CPE_ERR_RADIO_OP_FAILED,
    CPE_ERR_RFC_INTERNAL,
    CPE_ERR_SYNTH_NO_LOCK,
};

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
} rfd_error_t;

// ********************************
// State machine and control structure
// ********************************

// FSM states
typedef enum
{
    CPE_S_IDLE = 0,

    CPE_S_WAIT_POW_DOMAIN_ON,
    CPE_S_WAIT_RFC_BOOT,

    CPE_S_WAIT_CPE_READY,
    CPE_S_WAIT_CPE_ACK,
    CPE_S_WAIT_RADIO_OP_DONE,

    CPE_S_WAIT_ERR_CLEARED,

    CPE_STATE_NUM
} rfd_state_t;

// Flags
typedef enum
{
    CPE_F_RFC_HAS_BOOTED = 0x01,
    CPE_F_CMD_IS_RADIO_OP = 0x02,
    CPE_F_CMD_CHAIN = 0x04,
} rfd_flags_t;

// Control structure (module's state)
typedef struct
{
    rfd_state_t state;
    rfd_flags_t flags;
    rfd_error_t error;

    volatile rfc_command_t* cmd_p;
    uint16_t radio_op_tout_ms;
} rfd_control_t;

// ********************************
// Public functions
// ********************************

void Cpe_Init();

void Cpe_Process();

rfd_state_t Cpe_Get_FSM_State();

bool Cpe_Turn_On_RFC();

bool Cpe_Turn_Off_RFC();

bool Cpe_RFC_Is_On();

bool Cpe_Start_Direct_Cmd(volatile uint16_t command);

bool Cpe_Start_Immediate_Cmd(volatile rfc_command_t* command);

bool Cpe_Start_Radio_Op(volatile rfc_radioOp_t* radio_op, uint16_t timeout_ms);

bool Cpe_Start_Radio_Op_Chain(volatile rfc_radioOp_t* radio_op, uint16_t timeout_ms);

bool Cpe_Ready();

bool Cpe_Error_Occurred();

uint8_t Cpe_Get_Err_Code();


#endif /* CP_ENGINE_H_ */
