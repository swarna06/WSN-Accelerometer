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
void Rfd_S_Idle();
void Rfd_S_Wait_Power_On();
void Rfd_S_Wait_Boot();

void Rfd_S_Wait_Err_Cleared();

// Other static functions
static void Rfd_Hardware_Init();
static void Rfd_Handle_Error(uint8_t err_code);

// ********************************
// Static (private) variables
// ********************************

// Control structure
rfd_control_t rfc;

// State procedures array
void (*rfd_state_proc_ptr[RFD_STATE_NUM])() =
{
     [RFD_S_IDLE] = Rfd_S_Idle,
     [RFD_S_WAIT_POW_DOMAIN_ON] = Rfd_S_Wait_Power_On,
     [RFD_S_WAIT_RFC_BOOT] = Rfd_S_Wait_Boot,

     [RFD_S_WAIT_ERR_CLEARED] = Rfd_S_Wait_Err_Cleared
};

// ********************************
// Non-static (public) functions
// ********************************

void Rfd_Init()
{
    // Hardware initialization
    Rfd_Hardware_Init();

    // Control structure initialization
    rfc.flags = 0;
    rfc.state = RFD_S_IDLE;
}

inline void Rfd_Process()
{
    assertion(rfc.state < RFD_STATE_NUM);
    assertion(rfd_state_proc_ptr[rfc.state] != NULL);
    rfd_state_proc_ptr[rfc.state](); // execute state procedure
}

inline rfd_state_t Rfd_Get_FSM_State()
{
    return rfc.state;
}

bool Rfd_Turn_On()
{
    if (rfc.state != RFD_S_IDLE)
        return false;

    if (Rfd_Is_On() == false)
    {
        PRCMPowerDomainOn(PRCM_DOMAIN_RFCORE);
        Tm_Start_Timeout(TM_RF_DRIVER_TOUT_ID, RFD_TOUT_POWER_ON_MSEC);
        rfc.state = RFD_S_WAIT_POW_DOMAIN_ON;
        return true;
    }
    else
        return false;
}

bool Rfd_Turn_Off()
{
    if (rfc.error.code == RFD_ERR_NONE && rfc.state != RFD_S_IDLE)
        return false;

    if (Rfd_Is_On() == true)
    {
        PRCMPowerDomainOff(PRCM_DOMAIN_RFCORE);

        // Clear interrupt flags
        HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = 0;
        HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG) = 0;

        rfc.flags &= ~RFD_F_RFC_HAS_BOOTED;
        return true;
    }
    else
        return false;
}

inline bool Rfd_Is_On()
{
    return (PRCMPowerDomainStatus(PRCM_DOMAIN_RFCORE) == PRCM_DOMAIN_POWER_ON);
}

bool Rfd_Ready()
{
    if (rfc.state == RFD_S_IDLE && rfc.flags & RFD_F_RFC_HAS_BOOTED)
        return true;
    else
        return false;
}

bool Rfd_Error_Occurred()
{
    return (rfc.error.code != RFD_ERR_NONE);
}

uint8_t Rfd_Get_Err_Code()
{
    uint8_t err_code = rfc.error.code;
    rfc.error.code = RFD_ERR_NONE;

    return err_code;
}

// ********************************
// FSM procedures (static)
// ********************************

void Rfd_S_Idle()
{

}

void Rfd_S_Wait_Power_On()
{
    // Wait until RFC power domain is turned on
    if (Rfd_Is_On())
    {
        // Enable clock domain
        PRCMDomainEnable(PRCM_DOMAIN_RFCORE);
        PRCMLoadSet();
        while(!PRCMLoadGet());

        // Enable RF core clock
        RFCClockEnable();

        Tm_Start_Timeout(TM_RF_DRIVER_TOUT_ID, RFD_TOUT_BOOT_MSEC);
        rfc.state = RFD_S_WAIT_RFC_BOOT;
    }
    else if (Tm_Timeout_Completed(TM_RF_DRIVER_TOUT_ID))
    {
        Rfd_Handle_Error(RFD_ERR_POW_ON_TOUT);
        rfc.state = RFD_S_WAIT_ERR_CLEARED;
    }
}

void Rfd_S_Wait_Boot()
{
    uint32_t cpe_int_flags = Rfd_Get_CPE_Int_Flags();

    if (cpe_int_flags & RFC_DBELL_RFCPEIFG_BOOT_DONE)
    {
        rfc.flags |= RFD_F_RFC_HAS_BOOTED;
        rfc.state = RFD_S_IDLE;
    }
    else if (cpe_int_flags & RFC_DBELL_RFCPEIFG_INTERNAL_ERROR ||
             Tm_Timeout_Completed(TM_RF_DRIVER_TOUT_ID))
    {
        Rfd_Handle_Error(RFD_ERR_BOOT_FAILED);
        rfc.state = RFD_S_WAIT_ERR_CLEARED;
    }
}


void Rfd_S_Wait_Err_Cleared()
{
    // Wait until error code is cleared
    // The error code is cleared by calling Rfd_Get_Err_Code()
    if (rfc.error.code != RFD_ERR_NONE)
        rfc.state = RFD_S_IDLE;
}

// ********************************
// Other static (local/private) functions
// ********************************

static void Rfd_Hardware_Init()
{
    // Clear interrupt flags
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = 0;
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG) = 0;

    // Set RF mode - note: the RF core must be off before setting the mode
    HWREG(PRCM_BASE + PRCM_O_RFCMODESEL) = PRCM_RFCMODESEL_CURR_MODE1; // BLE
}

static void Rfd_Handle_Error(uint8_t err_code)
{
    Rfd_Abort_Cmd_Execution(); // abort execution of current command (if any is running)

    rfc.error.code = err_code;

    Rfd_Turn_Off();

    // Log error
    Log_Line("RFC err:");
    Log_Val_Hex32("\tcode: ", rfc.error.code);
}
