/*
 * cp_engine.c
 *
 * Command and Packet Engine (CPE)
 *
 *  Created on: Mar 5, 2019
 *      Author: alvaro
 */

#include <stdint.h>
#include <stdbool.h>

#include <driverlib/prcm.h>
#include <driverlib/rfc.h>

#include "cp_engine.h"
#include "misc.h"
#include "timing.h"
#include "log.h"

// ********************************
// Static (private) function declarations
// ********************************

// State procedures
static void Cpe_S_Idle();

static void Cpe_S_Wait_Power_On();
static void Cpe_S_Wait_Boot();

static void Cpe_S_Wait_CPE_Ready();
static void Cpe_S_Wait_CPE_Ack();
static void Cpe_S_Wait_Radio_Op_Done();

static void Cpe_S_Wait_Err_Cleared();

// Other static functions
static bool Cpe_Send_To_CPE(volatile rfc_command_t* command);
static void Cpe_Start_Cmd_Execution(volatile  rfc_command_t* command);
static void Cpe_Hardware_Init();
static void Cpe_Handle_Error(uint8_t err_code);

// ********************************
// Static (private) variables
// ********************************

// Control structure
static rfd_control_t rdc;

// State procedures array
static void (*rfd_state_proc_ptr[CPE_STATE_NUM])() =
{
     [CPE_S_IDLE] = Cpe_S_Idle,

     [CPE_S_WAIT_POW_DOMAIN_ON] = Cpe_S_Wait_Power_On,
     [CPE_S_WAIT_RFC_BOOT] = Cpe_S_Wait_Boot,

     [CPE_S_WAIT_CPE_READY] = Cpe_S_Wait_CPE_Ready,
     [CPE_S_WAIT_CPE_ACK] = Cpe_S_Wait_CPE_Ack,
     [CPE_S_WAIT_RADIO_OP_DONE] = Cpe_S_Wait_Radio_Op_Done,

     [CPE_S_WAIT_ERR_CLEARED] = Cpe_S_Wait_Err_Cleared
};

// ********************************
// Non-static (public) functions
// ********************************

void Cpe_Init()
{
    // Hardware initialization
    Cpe_Hardware_Init();

    // Control structure initialization
    rdc.flags = 0;
    rdc.state = CPE_S_IDLE;
    rdc.error.code = 0;
}

inline void Cpe_Process()
{
    assertion(rdc.state < CPE_STATE_NUM);
    assertion(rfd_state_proc_ptr[rdc.state] != NULL);
    rfd_state_proc_ptr[rdc.state](); // execute state procedure
}

inline rfd_state_t Cpe_Get_FSM_State()
{
    return rdc.state;
}

bool Cpe_Turn_On_RFC()
{
    if (rdc.state != CPE_S_IDLE) return false;

    if (Cpe_RFC_Is_On() == false)
    {
        PRCMPowerDomainOn(PRCM_DOMAIN_RFCORE);
        Tm_Start_Timeout(TM_CPE_TOUT_ID, CPE_TOUT_POWER_ON_MSEC);
        rdc.state = CPE_S_WAIT_POW_DOMAIN_ON;
        return true;
    }
    else
        return false;
}

bool Cpe_Turn_Off_RFC()
{
    if (rdc.error.code == CPE_ERR_NONE && rdc.state != CPE_S_IDLE)
        return false;

    if (Cpe_RFC_Is_On() == true)
    {
        PRCMPowerDomainOff(PRCM_DOMAIN_RFCORE);

        // Clear interrupt flags
        HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = 0;
        HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG) = 0;

        rdc.flags &= ~CPE_F_RFC_HAS_BOOTED;
        return true;
    }
    else
        return false;
}

inline bool Cpe_RFC_Is_On()
{
    return (PRCMPowerDomainStatus(PRCM_DOMAIN_RFCORE) == PRCM_DOMAIN_POWER_ON);
}

bool Cpe_Start_Direct_Cmd(uint16_t command)
{
    // A direct command is a special case of an immediate command
    return Cpe_Start_Immediate_Cmd((rfc_command_t*)CMDR_DIR_CMD(command));
}

bool Cpe_Start_Immediate_Cmd(volatile rfc_command_t* command)
{
    if (!Cpe_Ready()) return false;

    rdc.flags &= ~CPE_F_CMD_IS_RADIO_OP;
    Cpe_Start_Cmd_Execution(command);
    return true;
}

bool Cpe_Start_Radio_Op(volatile rfc_radioOp_t* radio_op, uint16_t timeout_ms)
{
    if (!Cpe_Ready()) return false;

    rdc.flags |= CPE_F_CMD_IS_RADIO_OP;
    rdc.radio_op_tout_ms = timeout_ms;  // 0 means wait forever

    radio_op->status = IDLE;
    Cpe_Start_Cmd_Execution((rfc_command_t*)radio_op);
    return true;
}

bool Cpe_Start_Radio_Op_Chain(volatile rfc_radioOp_t* radio_op, uint16_t timeout_ms)
{
    if (Cpe_Start_Radio_Op(radio_op, timeout_ms) == true) // success ?
    {
        rdc.flags |= CPE_F_CMD_CHAIN;
        return true;
    }
    else
        return false;
}

bool Cpe_Ready()
{
    if (rdc.state == CPE_S_IDLE && rdc.flags & CPE_F_RFC_HAS_BOOTED)
        return true;
    else
        return false;
}

bool Cpe_Error_Occurred()
{
    return (rdc.error.code != CPE_ERR_NONE);
}

uint8_t Cpe_Get_Err_Code()
{
    uint8_t err_code = rdc.error.code;
    rdc.error.code = CPE_ERR_NONE;
    return err_code;
}

// ********************************
// FSM procedures (static)
// ********************************

static void Cpe_S_Idle()
{
    // do nothing
}

static void Cpe_S_Wait_Power_On()
{
    // Wait until RFC power domain is turned on
    if (Cpe_RFC_Is_On())
    {
        // Enable clock domain
        PRCMDomainEnable(PRCM_DOMAIN_RFCORE);
        PRCMLoadSet();
        while(!PRCMLoadGet());

        // Enable RF core clock
        RFCClockEnable();

        Tm_Start_Timeout(TM_CPE_TOUT_ID, CPE_TOUT_BOOT_MSEC);
        rdc.state = CPE_S_WAIT_RFC_BOOT;
    }
    else if (Tm_Timeout_Completed(TM_CPE_TOUT_ID))
    {
        Cpe_Handle_Error(CPE_ERR_POWER_ON_TOUT);
        rdc.state = CPE_S_WAIT_ERR_CLEARED;
    }
}

static void Cpe_S_Wait_Boot()
{
    uint32_t cpe_int_flags = Cpe_Get_CPE_Int_Flags();

    if (cpe_int_flags & RFC_DBELL_RFCPEIFG_BOOT_DONE)
    {
        rdc.flags |= CPE_F_RFC_HAS_BOOTED;
        rdc.state = CPE_S_IDLE;
    }
    else if (cpe_int_flags & RFC_DBELL_RFCPEIFG_INTERNAL_ERROR ||
             Tm_Timeout_Completed(TM_CPE_TOUT_ID))
    {
        Cpe_Handle_Error(CPE_ERR_BOOT_FAILED);
        rdc.state = CPE_S_WAIT_ERR_CLEARED;
    }
}

static void Cpe_S_Wait_CPE_Ready()
{
    if (Cpe_Send_To_CPE(rdc.cmd_p) == true) // command accepted ?
    {
        Tm_Start_Timeout(TM_CPE_TOUT_ID, CPE_TOUT_CPE_ACK_MSEC);
        rdc.state = CPE_S_WAIT_CPE_ACK;
    }
    else if (Tm_Timeout_Completed(TM_CPE_TOUT_ID))
    {
        Cpe_Handle_Error(CPE_ERR_RFC_UNRESPONSIVE);
        rdc.state = CPE_S_WAIT_ERR_CLEARED;
    }
}

static void Cpe_S_Wait_CPE_Ack()
{
    if (Cpe_CPE_Ack() == true) // command acknowledged by the RFC ?
    {
        uint32_t cpe_cmd_sta = Cpe_Get_CPE_CMDSTA();
        if (cpe_cmd_sta != CMDSTA_Done) // error ?
        {
            Cpe_Handle_Error(CPE_ERR_CMD_PARSING);
            rdc.state = CPE_S_WAIT_ERR_CLEARED;
            return;
        }

        if (rdc.flags & CPE_F_CMD_IS_RADIO_OP) // radio operation ?
        {
            rdc.state = CPE_S_WAIT_RADIO_OP_DONE;
            Tm_Start_Timeout(TM_CPE_TOUT_ID, rdc.radio_op_tout_ms);
        }
        else // Immediate command has finished successfully
            rdc.state = CPE_S_IDLE;
    }
    else if (Tm_Timeout_Completed(TM_CPE_TOUT_ID))
    {
        Cpe_Handle_Error(CPE_ERR_RFC_UNRESPONSIVE);
        rdc.state = CPE_S_WAIT_ERR_CLEARED;
    }
}

static void Cpe_S_Wait_Radio_Op_Done()
{
    rfc_radioOp_t* radio_op_p = (rfc_radioOp_t*)rdc.cmd_p;
    uint32_t cpe_int_flags = Cpe_Get_CPE_Int_Flags();

    // Determine if radio operation (single or chain) has finished
    bool command_done = false;
    if (cpe_int_flags & CPE_M_CPE_COMMAND_DONE)
    {
        if (!(rdc.flags & CPE_F_CMD_CHAIN)) // single radio operation ?
            command_done = true;
        else if (cpe_int_flags & RFC_DBELL_RFCPEIFG_LAST_COMMAND_DONE) // last command done ?
            command_done = true;
    }

    if (command_done == true)
    {
        if (rdc.flags & CPE_F_CMD_CHAIN) // comand chain ?
            rdc.state = CPE_S_IDLE; // do not check for errors
        else if (!(radio_op_p->status & CPE_F_RADIO_OP_STATUS_ERR)) // success ?
            rdc.state = CPE_S_IDLE;
        else
        {
            Cpe_Handle_Error(CPE_ERR_RADIO_OP_FAILED);
            rdc.state = CPE_S_WAIT_ERR_CLEARED;
        }
    }
    else if (cpe_int_flags & RFC_DBELL_RFCPEIFG_RX_NOK) // CRC error ?
    {
        // Sometimes the RX_NOK flag is set but not the COMMAND_DONE flag
        Cpe_Abort_Cmd_Execution();
        radio_op_p->status = DONE_RXERR; // CRC error
        rdc.state = CPE_S_IDLE;
    }
    else if (cpe_int_flags & CPE_M_CPE_RF_CORE_ERR) // internal or synth-no-lock error ?
    {
        uint8_t err_code = CPE_ERR_RFC_INTERNAL;
        if (cpe_int_flags & RFC_DBELL_RFCPEIFG_SYNTH_NO_LOCK)
            err_code = CPE_ERR_SYNTH_NO_LOCK;

        Cpe_Handle_Error(err_code);
        rdc.state = CPE_S_WAIT_ERR_CLEARED;
    }
    else if (rdc.radio_op_tout_ms && Tm_Timeout_Completed(TM_CPE_TOUT_ID))
    {
        Cpe_Handle_Error(CPE_ERR_RFC_UNRESPONSIVE);
        rdc.state = CPE_S_WAIT_ERR_CLEARED;
    }
}

static void Cpe_S_Wait_Err_Cleared()
{
    // Wait until error code is cleared
    // The error code is cleared by calling Cpe_Get_Err_Code()
    if (rdc.error.code != CPE_ERR_NONE)
        rdc.state = CPE_S_IDLE;
}

// ********************************
// Other static (local/private) functions
// ********************************

static bool Cpe_Send_To_CPE(volatile rfc_command_t* command)
{
    if (Cpe_CPE_Ready()) // RF Core is ready ?
    {
        // Clear interrupt flags and start command execution
        Cpe_Clear_CPE_Int_Flags(CPE_M_CPE_COMMAND_DONE |
                                    CPE_M_CPE_TX_INT_FLAGS |
                                    CPE_M_CPE_RX_INT_FLAGS);
        Cpe_Clear_CPE_Ack();
        Cpe_Set_CMD_Reg(command);
        return true;
    }
    else
        return false;
}

static void Cpe_Start_Cmd_Execution(volatile rfc_command_t* command)
{
    rdc.cmd_p = command;

    if (Cpe_Send_To_CPE(rdc.cmd_p) == true) // command accepted ?
    {
        Tm_Start_Timeout(TM_CPE_TOUT_ID, CPE_TOUT_CPE_ACK_MSEC);
        rdc.state = CPE_S_WAIT_CPE_ACK;
    }
    else
    {
        Tm_Start_Timeout(TM_CPE_TOUT_ID, CPE_TOUT_CPE_ACK_MSEC);
        rdc.state = CPE_TOUT_CPE_READY_MSEC;
    }
}

static void Cpe_Hardware_Init()
{
    // Clear interrupt flags
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = 0;
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG) = 0;

    // Set RF mode - note: the RF core must be off before setting the mode
    HWREG(PRCM_BASE + PRCM_O_RFCMODESEL) = PRCM_RFCMODESEL_CURR_MODE1; // BLE
}

static void Cpe_Handle_Error(uint8_t err_code)
{
    Cpe_Abort_Cmd_Execution(); // abort execution of current command (if any is running)

    rdc.error.code = err_code; // error code must be set before calling Cpe_Turn_Off()
    Cpe_Turn_Off_RFC();

    // Log error
    Log_Line("RFC err:");
    Log_Val_Hex32("\tcode: ", rdc.error.code);
}
