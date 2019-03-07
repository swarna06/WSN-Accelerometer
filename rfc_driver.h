/*
 * rfc_driver.h
 *
 *  Created on: Mar 5, 2019
 *      Author: alvaro
 */

#ifndef RFC_DRIVER_H_
#define RFC_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>

#include <driverlib/rf_common_cmd.h>
#include <driverlib/rf_mailbox.h>
#include <inc/hw_rfc_rat.h>
#include <inc/hw_rfc_dbell.h>

// ********************************
// Timing
// ********************************
// Timeout values (milliseconds)
enum
{
    RDV_TOUT_DEFAULT_MSEC = 10,
    RDV_TOUT_POWER_ON_MSEC = RDV_TOUT_DEFAULT_MSEC,
    RDV_TOUT_BOOT_MSEC = RDV_TOUT_DEFAULT_MSEC,
    RDV_TOUT_CPE_READY_MSEC = RDV_TOUT_DEFAULT_MSEC,
    RDV_TOUT_CPE_ACK_MSEC = RDV_TOUT_DEFAULT_MSEC,
    RDV_TOUT_RADIO_OP_MSEC = RDV_TOUT_DEFAULT_MSEC,
};

// ********************************
// RF Core interface
// ********************************
// CPE command done interrupt flags
#define RDV_M_CPE_COMMAND_DONE          (RFC_DBELL_RFCPEIFG_LAST_COMMAND_DONE | \
                                         RFC_DBELL_RFCPEIFG_COMMAND_DONE)

// CPE TX interrupt flags
#define RDV_M_CPE_TX_INT_FLAGS          (RFC_DBELL_RFCPEIFG_TX_BUFFER_CHANGED | \
                                        RFC_DBELL_RFCPEIFG_TX_ENTRY_DONE | \
                                        RFC_DBELL_RFCPEIFG_TX_RETRANS | \
                                        RFC_DBELL_RFCPEIFG_TX_CTRL_ACK_ACK | \
                                        RFC_DBELL_RFCPEIFG_TX_CTRL_ACK | \
                                        RFC_DBELL_RFCPEIFG_TX_CTRL | \
                                        RFC_DBELL_RFCPEIFG_TX_ACK | \
                                        RFC_DBELL_RFCPEIFG_TX_DONE)

// CPE TX interrupt flags
#define RDV_M_CPE_RX_INT_FLAGS          (RFC_DBELL_RFCPEIFG_RX_ABORTED | \
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
#define RDV_M_CPE_RF_CORE_ERR           (RFC_DBELL_RFCPEIFG_INTERNAL_ERROR | \
                                         RFC_DBELL_RFCPEIFG_SYNTH_NO_LOCK)
#define RDV_M_CPE_RX_ERR_FLAGS          (RFC_DBELL_RFCPEIFG_RX_NOK)

// Macro functions for interfacing with the Command and Packet Engine (CPE)
#define Rdv_Get_CPE_Int_Flags()         (HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG))
#define Rdv_Clear_CPE_Int_Flags(msk)    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = ~msk;
#define Rdv_CPE_Ready()                 (HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDR) == 0)
#define Rdv_Set_CMD_Reg(op)             HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDR) = (uint32_t)op;
#define Rdv_Clear_CPE_Ack()             HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG) = 0x0;
#define Rdv_CPE_Ack()                   (HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG))
#define Rdv_Get_CPE_CMDSTA()            (HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDSTA))

#define Rdv_Abort_Cmd_Execution()       Rdv_Set_CMD_Reg(CMDR_DIR_CMD(CMD_ABORT))

// Radio operation status field error flag
#define RDV_F_RADIO_OP_STATUS_ERR       0x0800

// ********************************
// Error handling
// ********************************
// Error codes
enum
{
    RDV_ERR_NONE = 0,
    RDV_ERR_POWER_ON_TOUT,
    RDV_ERR_BOOT_FAILED,
    RDV_ERR_RFC_UNRESPONSIVE,
    RDV_ERR_CMD_PARSING,
    RDV_ERR_RADIO_OP_FAILED,
    RDV_ERR_RFC_INTERNAL,
    RDV_ERR_SYNTH_NO_LOCK,
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

// Data types
typedef enum
{
    RDV_S_IDLE = 0,

    RDV_S_WAIT_POW_DOMAIN_ON,
    RDV_S_WAIT_RFC_BOOT,

    RDV_S_WAIT_CPE_READY,
    RDV_S_WAIT_CPE_ACK,
    RDV_S_WAIT_RADIO_OP_DONE,

    RDV_S_WAIT_ERR_CLEARED,

    RDV_STATE_NUM
} rfd_state_t;

typedef enum
{
    RDV_F_RFC_HAS_BOOTED = 0x01,
    RDV_F_CMD_IS_RADIO_OP = 0x02,
    RDV_F_CMD_CHAIN = 0x04,
} rfd_flags_t;

typedef struct
{
    rfd_state_t state;
    rfd_flags_t flags;
    rfd_error_t error;

    volatile rfc_command_t* cmd_p;
    uint16_t radio_op_tout_ms;
} rfd_control_t;

void Rdv_Init();

void Rdv_Process();

rfd_state_t Rdv_Get_FSM_State();

bool Rdv_Turn_On();

bool Rdv_Turn_Off();

bool Rdv_Is_On();

bool Rdv_Start_Direct_Cmd(uint16_t command);

bool Rdv_Start_Immediate_Cmd(rfc_command_t* command);

bool Rdv_Start_Radio_Op(rfc_radioOp_t* radio_op, uint16_t timeout_ms);

bool Rdv_Start_Radio_Op_Chain(rfc_radioOp_t* radio_op, uint16_t timeout_ms);

bool Rdv_Ready();

bool Rdv_Error_Occurred();

uint8_t Rdv_Get_Err_Code();


#endif /* RFC_DRIVER_H_ */
