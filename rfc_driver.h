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
    RFD_TOUT_DEFAULT_MSEC = 10,
    RFD_TOUT_POWER_ON_MSEC = RFD_TOUT_DEFAULT_MSEC,
    RFD_TOUT_BOOT_MSEC = RFD_TOUT_DEFAULT_MSEC,

};

// ********************************
// RF Core interface
// ********************************
// CPE command done interrupt flags
#define RFD_M_CPE_COMMAND_DONE          (RFC_DBELL_RFCPEIFG_LAST_COMMAND_DONE | \
                                         RFC_DBELL_RFCPEIFG_COMMAND_DONE)

// CPE TX interrupt flags
#define RFD_M_CPE_TX_INT_FLAGS          (RFC_DBELL_RFCPEIFG_TX_BUFFER_CHANGED | \
                                        RFC_DBELL_RFCPEIFG_TX_ENTRY_DONE | \
                                        RFC_DBELL_RFCPEIFG_TX_RETRANS | \
                                        RFC_DBELL_RFCPEIFG_TX_CTRL_ACK_ACK | \
                                        RFC_DBELL_RFCPEIFG_TX_CTRL_ACK | \
                                        RFC_DBELL_RFCPEIFG_TX_CTRL | \
                                        RFC_DBELL_RFCPEIFG_TX_ACK | \
                                        RFC_DBELL_RFCPEIFG_TX_DONE)

// CPE TX interrupt flags
#define RFD_M_CPE_RX_INT_FLAGS          (RFC_DBELL_RFCPEIFG_RX_ABORTED | \
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
#define RFD_M_CPE_RF_CORE_ERR           (RFC_DBELL_RFCPEIFG_INTERNAL_ERROR | \
                                         RFC_DBELL_RFCPEIFG_SYNTH_NO_LOCK)
#define RFD_M_CPE_RX_ERR_FLAGS          (RFC_DBELL_RFCPEIFG_RX_NOK)

// Macro functions for interfacing with the Command and Packet Engine (CPE)
#define Rfd_Get_CPE_Int_Flags()         (HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG))
#define Rfd_Clear_CPE_Int_Flags(msk)    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = ~msk;
#define Rfd_CPE_Ready()                 (HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDR) == 0)
#define Rfd_Send_To_CPE(op)             HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDR) = (uint32_t)op;
#define Rfd_CPE_Ack()                   (HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG))
#define Rfd_Get_CPE_CMDSTA()            (HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDSTA))

#define Rfd_Abort_Cmd_Execution()       Rfd_Send_To_CPE(CMDR_DIR_CMD(CMD_ABORT))

// ********************************
// Error handling
// ********************************
// Error codes
enum
{
    RFD_ERR_NONE = 0,
    RFD_ERR_POW_ON_TOUT,
    RFD_ERR_BOOT_FAILED,
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
    RFD_S_IDLE = 0,

    RFD_S_WAIT_POW_DOMAIN_ON,
    RFD_S_WAIT_RFC_BOOT,

    RFD_S_WAIT_ERR_CLEARED,

    RFD_STATE_NUM
} rfd_state_t;

typedef enum
{
    RFD_F_RFC_HAS_BOOTED = 0x01,
} rfd_flags_t;

typedef struct
{
    rfd_state_t state;
    rfd_flags_t flags;
    rfd_error_t error;

    volatile rfc_command_t* cmd_p;
} rfd_control_t;

void Rfd_Init();

void Rfd_Process();

rfd_state_t Rfd_Get_FSM_State();

bool Rfd_Turn_On();

bool Rfd_Turn_Off();

bool Rfd_Is_On();

bool Rfd_Start_Direct_Cmd(uint16_t cmd);

bool Rfd_Start_Immediate_Cmd(rfc_command_t* cmd);

bool Rfd_Start_Radio_Op(rfc_radioOp_t* radio_op);

bool Rfd_Start_Radio_Op_Chain(rfc_radioOp_t* radio_op);

bool Rfd_Abort();

bool Rfd_Ready();

bool Rfd_Error_Occurred();

uint8_t Rfd_Get_Error(rfd_error_t error);


#endif /* RFC_DRIVER_H_ */
