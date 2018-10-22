/*
 * radio.c
 *
 *  Created on: 21 oct. 2018
 *      Author: Alvaro
 */

#include <stdint.h>

#include <driverlib/prcm.h>
#include <driverlib/sys_ctrl.h>

#include <driverlib/rfc.h>
#include <driverlib/rf_mailbox.h>
#include <driverlib/rf_common_cmd.h>
#include <driverlib/rf_ble_mailbox.h>
#include <driverlib/rf_ble_cmd.h>
#include <driverlib/rf_data_entry.h>
#include <inc/hw_rfc_rat.h>

#include "radio.h"
#include "smartrf_settings.h"

#include "misc.h"
#include "printf.h"

// Pointers to SmartRF command structures
static volatile rfc_CMD_BLE5_RADIO_SETUP_t* cmd_ble5_radio_setup = &RF_cmdBle5RadioSetup;
static volatile rfc_CMD_FS_t* cmd_fs = &RF_cmdFs;

// Local functions
static int Rad_Execute_Radio_Op(volatile rfc_radioOp_t* radio_op_p);
static int Rad_Execute_Direct_Cmd(uint16_t cmd_id);

static void Rad_Boot_RFC()
{
    // Turn off RF core
    PRCMPowerDomainOff(PRCM_DOMAIN_RFCORE);
    while (PRCMPowerDomainStatus(PRCM_DOMAIN_RFCORE) == PRCM_DOMAIN_POWER_ON);

    // Clear interrupt flags
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = 0;

    // Set RF mode (Bluetooth)
    HWREG(PRCM_BASE + PRCM_O_RFCMODESEL) = RF_MODE_BLE;

    // Turn on power domains and enable clock domain
    PRCMPowerDomainOn(PRCM_DOMAIN_RFCORE);
    while (PRCMPowerDomainStatus(PRCM_DOMAIN_RFCORE) != PRCM_DOMAIN_POWER_ON);
    PRCMDomainEnable(PRCM_DOMAIN_RFCORE);
    PRCMLoadSet();
    while(!PRCMLoadGet());

    // Enable RF core clock
    RFCClockEnable();
    PRCMLoadSet();
    while(!PRCMLoadGet());

    // Wait until the RF core boots
    uint32_t rfcpeifg = 0;
    while (!( (rfcpeifg = HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG)) & RFC_DBELL_RFCPEIFG_BOOT_DONE))
    {
        if (rfcpeifg & RAD_M_RFCPEIFG_ERROR)
        {
            Rad_Print_RFCPEIFG_Err();
            exit(0);
        }
    }

    // Ping the RF core
    if (Rad_Execute_Direct_Cmd(CMD_PING) != RAD_OK)
        exit(0);
}

void Rad_Init()
{
    Rad_Boot_RFC();

    int result;
    rfc_radioOp_t* radio_op_p = NULL;

    // Radio setup
    result = Rad_Execute_Radio_Op(radio_op_p = (rfc_radioOp_t*)cmd_ble5_radio_setup);
    if (result != RAD_OK || radio_op_p->status != BLE_DONE_OK)
        goto handle_err;

    // Start frequency synthesizer
    Rad_Execute_Radio_Op(radio_op_p = (rfc_radioOp_t*)cmd_fs);
    if (result != RAD_OK || radio_op_p->status != DONE_OK)
        goto handle_err;

    // Start RAT (RAdio Timer)
    Rad_Execute_Direct_Cmd(CMD_START_RAT);
    if (result != RAD_OK)
        exit(0);

    return;

handle_err:
    Rad_Print_Radio_Op_Err(radio_op_p);
    exit(0);
}

int Rad_Execute_Radio_Op(volatile rfc_radioOp_t* radio_op_p)
{
    assertion(radio_op_p != NULL);
    assertion(is_aligned(radio_op_p, 4));

    radio_op_p->status = IDLE; // reset status field

    // Send command to radio door bell and check value of CMDSTA register
    uint32_t cmdsta = RFCDoorbellSendTo((uint32_t)radio_op_p);
    if (cmdsta != CMDSTA_Done)
        return RAD_E_CPE_CMDSTA;

    // Wait for operation execution
    while (radio_op_p->status <= ACTIVE)
    {
        if (HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) & RAD_M_RFCPEIFG_ERROR) // critical error ?
        {
            Rad_Print_Radio_Op_Err(radio_op_p);
//            return RAD_E_RFCPEIFG;
            exit(0);
        }
    }
    if (radio_op_p->status & RAD_F_STATUS_ERR) // error in the status field of the command structure ?
        return RAD_E_STATUS_FIELD;

    return RAD_OK;
}

int Rad_Execute_Direct_Cmd(uint16_t cmd_id)
{
    // Send command to radio door bell and check value of CMDSTA register
    uint32_t cmdsta = RFCDoorbellSendTo(CMDR_DIR_CMD(cmd_id));
    if (cmdsta != CMDSTA_Done)
    {
        Rad_Print_Direct_Cmd_Err(cmd_id);
        return RAD_E_CPE_CMDSTA;
    }

    return RAD_OK;
}


