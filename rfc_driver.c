/*
 * rfc_driver.c
 *
 *  Created on: Mar 5, 2019
 *      Author: alvaro
 */

#include <stdint.h>
#include <stdbool.h>

#include <driverlib/prcm.h>
#include <driverlib/sys_ctrl.h>

#include <driverlib/rfc.h>
#include <driverlib/rf_mailbox.h>
#include <driverlib/rf_common_cmd.h>
#include <driverlib/rf_data_entry.h>
#include <inc/hw_rfc_rat.h>
#include <driverlib/rf_ble_mailbox.h>
#include <driverlib/rf_ble_cmd.h>

#include "rfc_driver.h"
#include "misc.h"
#include "timing.h"
#include "log.h"

// ********************************
// Static (private) function declarations
// ********************************

// State procedures
void Rdv_S_Idle();

void Rdv_S_Wait_Power_On();
void Rdv_S_Wait_Boot();

void Rdv_S_Wait_CPE_Ready();
void Rdv_S_Wait_CPE_Ack();
void Rdv_S_Wait_Radio_Op_Done();

void Rdv_S_Wait_Err_Cleared();

// Other static functions
static bool Rdv_Send_To_CPE(volatile rfc_command_t* command);
static void Rdv_Start_Cmd_Execution(rfc_command_t* command);
static void Rdv_Hardware_Init();
static void Rdv_Handle_Error(uint8_t err_code);

// ********************************
// Static (private) variables
// ********************************

// Control structure
rfd_control_t rdc;

// State procedures array
void (*rfd_state_proc_ptr[RDV_STATE_NUM])() =
{
     [RDV_S_IDLE] = Rdv_S_Idle,

     [RDV_S_WAIT_POW_DOMAIN_ON] = Rdv_S_Wait_Power_On,
     [RDV_S_WAIT_RFC_BOOT] = Rdv_S_Wait_Boot,

     [RDV_S_WAIT_CPE_READY] = Rdv_S_Wait_CPE_Ready,
     [RDV_S_WAIT_CPE_ACK] = Rdv_S_Wait_CPE_Ack,
     [RDV_S_WAIT_RADIO_OP_DONE] = Rdv_S_Wait_Radio_Op_Done,

     [RDV_S_WAIT_ERR_CLEARED] = Rdv_S_Wait_Err_Cleared
};

// ********************************
// Non-static (public) functions
// ********************************

void Rdv_Init()
{
    // Hardware initialization
    Rdv_Hardware_Init();

    // Control structure initialization
    rdc.flags = 0;
    rdc.state = RDV_S_IDLE;
}

inline void Rdv_Process()
{
    assertion(rdc.state < RDV_STATE_NUM);
    assertion(rfd_state_proc_ptr[rdc.state] != NULL);
    rfd_state_proc_ptr[rdc.state](); // execute state procedure
}

inline rfd_state_t Rdv_Get_FSM_State()
{
    return rdc.state;
}

bool Rdv_Turn_On()
{
    if (rdc.state != RDV_S_IDLE)
        return false;

    if (Rdv_Is_On() == false)
    {
        PRCMPowerDomainOn(PRCM_DOMAIN_RFCORE);
        Tm_Start_Timeout(TM_RF_DRIVER_TOUT_ID, RDV_TOUT_POWER_ON_MSEC);
        rdc.state = RDV_S_WAIT_POW_DOMAIN_ON;
        return true;
    }
    else
        return false;
}

bool Rdv_Turn_Off()
{
    if (rdc.error.code == RDV_ERR_NONE && rdc.state != RDV_S_IDLE)
        return false;

    if (Rdv_Is_On() == true)
    {
        PRCMPowerDomainOff(PRCM_DOMAIN_RFCORE);

        // Clear interrupt flags
        HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = 0;
        HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG) = 0;

        rdc.flags &= ~RDV_F_RFC_HAS_BOOTED;
        return true;
    }
    else
        return false;
}

inline bool Rdv_Is_On()
{
    return (PRCMPowerDomainStatus(PRCM_DOMAIN_RFCORE) == PRCM_DOMAIN_POWER_ON);
}

bool Rdv_Start_Direct_Cmd(uint16_t command)
{
    // A direct command is a special case of an immediate command
    return Rdv_Start_Immediate_Cmd((rfc_command_t*)CMDR_DIR_CMD(command));
}

bool Rdv_Start_Immediate_Cmd(rfc_command_t* command)
{
    if (!Rdv_Ready())
        return false;

    rdc.flags &= ~RDV_F_CMD_IS_RADIO_OP;
    Rdv_Start_Cmd_Execution(command);
    return true;
}

bool Rdv_Start_Radio_Op(rfc_radioOp_t* radio_op)
{
    if (!Rdv_Ready())
        return false;

    rdc.flags |= RDV_F_CMD_IS_RADIO_OP;
    Rdv_Start_Cmd_Execution((rfc_command_t*)radio_op);
    return true;
}

bool Rdv_Ready()
{
    if (rdc.state == RDV_S_IDLE && rdc.flags & RDV_F_RFC_HAS_BOOTED)
        return true;
    else
        return false;
}

bool Rdv_Error_Occurred()
{
    return (rdc.error.code != RDV_ERR_NONE);
}

uint8_t Rdv_Get_Err_Code()
{
    uint8_t err_code = rdc.error.code;
    rdc.error.code = RDV_ERR_NONE;

    return err_code;
}

// ********************************
// FSM procedures (static)
// ********************************

void Rdv_S_Idle()
{

}

void Rdv_S_Wait_Power_On()
{
    // Wait until RFC power domain is turned on
    if (Rdv_Is_On())
    {
        // Enable clock domain
        PRCMDomainEnable(PRCM_DOMAIN_RFCORE);
        PRCMLoadSet();
        while(!PRCMLoadGet());

        // Enable RF core clock
        RFCClockEnable();

        Tm_Start_Timeout(TM_RF_DRIVER_TOUT_ID, RDV_TOUT_BOOT_MSEC);
        rdc.state = RDV_S_WAIT_RFC_BOOT;
    }
    else if (Tm_Timeout_Completed(TM_RF_DRIVER_TOUT_ID))
    {
        Rdv_Handle_Error(RDV_ERR_POWER_ON_TOUT);
        rdc.state = RDV_S_WAIT_ERR_CLEARED;
    }
}

void Rdv_S_Wait_Boot()
{
    uint32_t cpe_int_flags = Rdv_Get_CPE_Int_Flags();

    if (cpe_int_flags & RFC_DBELL_RFCPEIFG_BOOT_DONE)
    {
        rdc.flags |= RDV_F_RFC_HAS_BOOTED;
        rdc.state = RDV_S_IDLE;
    }
    else if (cpe_int_flags & RFC_DBELL_RFCPEIFG_INTERNAL_ERROR ||
             Tm_Timeout_Completed(TM_RF_DRIVER_TOUT_ID))
    {
        Rdv_Handle_Error(RDV_ERR_BOOT_FAILED);
        rdc.state = RDV_S_WAIT_ERR_CLEARED;
    }
}

void Rdv_S_Wait_CPE_Ready()
{
    if (Rdv_Send_To_CPE(rdc.cmd_p) == true) // command accepted ?
    {
        Tm_Start_Timeout(TM_RF_DRIVER_TOUT_ID, RDV_TOUT_CPE_ACK_MSEC);
        rdc.state = RDV_S_WAIT_CPE_ACK;
    }
    else if (Tm_Timeout_Completed(TM_RF_DRIVER_TOUT_ID))
    {
        Rdv_Handle_Error(RDV_ERR_RFC_UNRESPONSIVE);
        rdc.state = RDV_S_WAIT_ERR_CLEARED;
    }
}

void Rdv_S_Wait_CPE_Ack()
{
    if (Rdv_CPE_Ack() == true) // command acknowledged by the RFC ?
    {
        uint32_t cpe_cmd_sta = Rdv_Get_CPE_CMDSTA();
        if (cpe_cmd_sta != CMDSTA_Done) // error ?
        {
            Rdv_Handle_Error(RDV_ERR_CMD_PARSING);
            rdc.state = RDV_S_WAIT_ERR_CLEARED;
            return;
        }

        if (rdc.flags & RDV_F_CMD_IS_RADIO_OP) // radio operation ?
        {
            rdc.state = RDV_S_WAIT_RADIO_OP_DONE;
            Tm_Start_Timeout(TM_RF_DRIVER_TOUT_ID, RDV_TOUT_RADIO_OP_MSEC); // xxx should this be variable
        }
        else // Immediate command has finished successfully
            rdc.state = RDV_S_WAIT_ERR_CLEARED;
    }
    else if (Tm_Timeout_Completed(TM_RF_DRIVER_TOUT_ID))
    {
        Rdv_Handle_Error(RDV_ERR_RFC_UNRESPONSIVE);
        rdc.state = RDV_S_WAIT_ERR_CLEARED;
    }
}

void Rdv_S_Wait_Radio_Op_Done()
{

}

void Rdv_S_Wait_Err_Cleared()
{
    // Wait until error code is cleared
    // The error code is cleared by calling Rdv_Get_Err_Code()
    if (rdc.error.code != RDV_ERR_NONE)
        rdc.state = RDV_S_IDLE;
}

// ********************************
// Other static (local/private) functions
// ********************************

static bool Rdv_Send_To_CPE(volatile rfc_command_t* command)
{
    if (Rdv_CPE_Ready()) // RF Core is ready ?
    {
        Rdv_Clear_CPE_Ack();
        Rdv_Set_CMD_Reg(command);
        return true;
    }
    else
        return false;
}

static void Rdv_Start_Cmd_Execution(rfc_command_t* command)
{
    rdc.cmd_p = command;

    if (Rdv_Send_To_CPE(rdc.cmd_p) == true) // command accepted ?
    {
        Tm_Start_Timeout(TM_RF_DRIVER_TOUT_ID, RDV_TOUT_CPE_ACK_MSEC);
        rdc.state = RDV_S_WAIT_CPE_ACK;
    }
    else
    {
        Tm_Start_Timeout(TM_RF_DRIVER_TOUT_ID, RDV_TOUT_CPE_ACK_MSEC);
        rdc.state = RDV_TOUT_CPE_READY_MSEC;
    }
}

static void Rdv_Hardware_Init()
{
    // Clear interrupt flags
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = 0;
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG) = 0;

    // Set RF mode - note: the RF core must be off before setting the mode
    HWREG(PRCM_BASE + PRCM_O_RFCMODESEL) = PRCM_RFCMODESEL_CURR_MODE1; // BLE
}

static void Rdv_Handle_Error(uint8_t err_code)
{
    Rdv_Abort_Cmd_Execution(); // abort execution of current command (if any is running)

    rdc.error.code = err_code;

    Rdv_Turn_Off();

    // Log error
    Log_Line("RFC err:");
    Log_Val_Hex32("\tcode: ", rdc.error.code);
}
