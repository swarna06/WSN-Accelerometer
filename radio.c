/*
 * radio.c
 *
 *  Created on: Mar 7, 2019
 *      Author: alvaro
 */

#include <stdint.h>
#include <stdbool.h>

#include <driverlib/rfc.h>

#include "radio.h"
#include "cp_engine.h"
#include "misc.h"
#include "smartrf_settings.h"

// ********************************
// Static (private) function declarations
// ********************************
// State procedures
static void Rad_S_Idle();

static void Rad_S_Wait_RFC_Boot();
static void Rad_S_Wait_RFC_Config_Sequence();
static void Rad_S_Wait_RFC_Sync_Stop_RAT();

static void Rad_S_Wait_Err_Cleared();

// Other static functions
static void Rad_Init_CPE_Structs();
static void Rad_Handle_Error(uint8_t err_code);

// ********************************
// Static (private) variables
// ********************************

// Control structure
static rad_control_t rac;

// State procedures array
static void (*rad_state_proc_ptr[RAD_STATE_NUM])() =
{
     [RAD_S_IDLE] = Rad_S_Idle,

     [RAD_S_WAIT_RFC_BOOT] = Rad_S_Wait_RFC_Boot,
     [RAD_S_WAIT_RFC_CONFIG_SEQUENCE] = Rad_S_Wait_RFC_Config_Sequence,
     [RAD_S_WAIT_RFC_SYNC_STOP_RAT] = Rad_S_Wait_RFC_Sync_Stop_RAT,

     [RAD_S_WAIT_ERR_CLEARED] = Rad_S_Wait_Err_Cleared
};

// RF core radio operation structures
static volatile rfc_CMD_BLE5_RADIO_SETUP_t* cmd_ble5_radio_setup_p = &RF_cmdBle5RadioSetup;
static volatile rfc_CMD_FS_t* cmd_fs_p = &RF_cmdFs;
static volatile rfc_CMD_SYNC_START_RAT_t* cmd_sync_start_rat_p = &RF_cmdSyncStartRat;
static volatile rfc_CMD_SYNC_STOP_RAT_t* cmd_sync_stop_rat_p = &RF_cmdSyncStopRat;

static volatile rfc_CMD_BLE5_ADV_AUX_t* cmd_ble5_adv_aux_p = &RF_cmdBle5AdvAux;
static volatile rfc_CMD_BLE5_SCANNER_t* cmd_ble5_scanner_p = &RF_cmdBle5Scanner;

// Other static variables
static volatile rfc_radioOp_t* rad_config_cmd_chain_p = (rfc_radioOp_t*)&RF_cmdBle5RadioSetup;

// ********************************
// Non-static (public) functions
// ********************************

void Rad_Init()
{
    rac.state = RAD_S_IDLE;
    rac.flags = 0;
    rac.err_code = 0;

    Rad_Init_CPE_Structs();
}

void Rad_Process()
{
    assertion(rac.state < RAD_STATE_NUM);
    assertion(rad_state_proc_ptr[rac.state] != NULL);
    rad_state_proc_ptr[rac.state](); // execute state procedure
}

inline uint8_t Rad_Get_FSM_State()
{
    return rac.state;
}

bool Rad_Turn_On_Radio()
{
    if (rac.state != RAD_S_IDLE || rac.flags & RAD_F_RFC_CONFIGURED) // busy or already on ?
        return false;

    if (Cpe_Turn_On_RFC() == true) // success ?
    {
        rac.state = RAD_S_WAIT_RFC_BOOT;
        return true;
    }
    else
        return false;
}

bool Rad_Turn_Off_Radio()
{
    if (rac.state != RAD_S_IDLE || !(rac.flags & RAD_F_RFC_CONFIGURED)) // busy or already off ?
        return false;

    if (Cpe_Start_Radio_Op((rfc_radioOp_t*)cmd_sync_stop_rat_p, 0) == true) // success ? xxx timeout
    {
        rac.state = RAD_S_WAIT_RFC_SYNC_STOP_RAT;
        return true;
    }
    else
        return false;
}

bool Rad_Radio_Is_On()
{
    if (rac.flags & RAD_F_RFC_CONFIGURED)
        return true;
    else
        return false;
}

bool Rad_Set_Data_Rate(rad_data_rate_t data_rate)
{
    if (rac.state != RAD_S_IDLE)
        return false;

    /*
     * Main mode and coding values according to data sheet
     *
     * Main modes => 0: 1 Mbps, 1: 2 Mbps, 2: Coded
     * Coding => 0: S = 8 (125 kbps), 1: S = 2 (500 kbps)
     */

    assertion(data_rate < RFC_DATA_RATES_NUM);

    uint8_t main_mode = RAD_BLE5_PHY_MAIN_MODE_2MBPS; // default mode 2 Mbps
    uint8_t coding = RAD_BLE5_PHY_CODING_NONE; // default: no coding

    if (data_rate == RAD_DATA_RATE_1MBPS)
        main_mode = RAD_BLE5_PHY_MAIN_MODE_1MBPS;
    else if  (data_rate == RAD_DATA_RATE_500KBPS)
    {
        main_mode = RAD_BLE5_PHY_MAIN_MODE_CODED;
        coding = RAD_BLE5_PHY_CODING_500KBPS;
    }
    else if  (data_rate == RAD_DATA_RATE_125KBPS)
    {
        main_mode = RAD_BLE5_PHY_MAIN_MODE_CODED;
        coding = RAD_BLE5_PHY_CODING_125KBPS;
    }

    // Set channel TX and RX commands
    cmd_ble5_radio_setup_p->defaultPhy.mainMode = main_mode; // TODO necessary ?
    cmd_ble5_radio_setup_p->defaultPhy.coding = coding; // TODO necessary ?

    cmd_ble5_adv_aux_p->phyMode.mainMode = main_mode;
    cmd_ble5_adv_aux_p->phyMode.coding = coding;

    cmd_ble5_scanner_p->phyMode.mainMode = main_mode;
    cmd_ble5_scanner_p->phyMode.coding = coding;

    return true;
}

bool Rad_Ready()
{
    if (rac.state == RAD_S_IDLE && rac.flags & RAD_F_RFC_CONFIGURED)
        return true;
    else
        return false;
}

bool Rad_Error_Occurred()
{
    return (rac.err_code != RAD_ERR_NONE);
}

uint8_t Rad_Get_Err_Code()
{
    uint8_t err_code = rac.err_code;
    rac.err_code = RAD_ERR_NONE;
    return err_code;
}


// ********************************
// FSM procedures (static)
// ********************************

static void Rad_S_Idle()
{
    // do nothing
}

static void Rad_S_Wait_RFC_Boot()
{
    if (Cpe_Ready() == true) // wait until the RFC becomes ready
    {
        // Start radio configuration
        Cpe_Start_Radio_Op_Chain(rad_config_cmd_chain_p, 0); // xxx timeout
        rac.state = RAD_S_WAIT_RFC_CONFIG_SEQUENCE;
    }
    else if (Cpe_Error_Occurred() == true)
    {
        Rad_Handle_Error(RAD_ERR_START_UP_FAILED);
        rac.state = RAD_S_WAIT_ERR_CLEARED;
    }
}

static void Rad_S_Wait_RFC_Config_Sequence()
{
    if (Cpe_Ready() == true) // wait end of execution of command chain
    {
        // Success; radio is now ready to transmit and receive
        rac.flags |= RAD_F_RFC_CONFIGURED;
        rac.state = RAD_S_IDLE;
    }
    else if (Cpe_Get_Err_Code() != CPE_ERR_NONE)
    {
        Rad_Handle_Error(RAD_ERR_START_UP_FAILED);
        rac.state = RAD_S_WAIT_ERR_CLEARED;
    }
}

static void Rad_S_Wait_RFC_Sync_Stop_RAT()
{
    if (Cpe_Ready() == true) // radio operation finished ?
    {
        Cpe_Turn_Off_RFC();
        rac.flags &= ~RAD_F_RFC_CONFIGURED;
        // Store rat0 value in SYNC_START_RAT for synchronization
        // with RTC next time the RFC is initialized.
        cmd_sync_start_rat_p->rat0 = cmd_sync_stop_rat_p->rat0;
        rac.state = RAD_S_IDLE;
    }
    else if (Cpe_Get_Err_Code() != CPE_ERR_NONE)
    {
        Rad_Handle_Error(RAD_ERR_SHUTDOWN_FAILED);
        rac.state = RAD_S_WAIT_ERR_CLEARED;
    }

}

static void Rad_S_Wait_Err_Cleared()
{
    // Wait until error code is cleared
    // The error code is cleared by calling Rad_Get_Err_Code()
    if (rac.err_code == RAD_ERR_NONE)
        rac.state = RAD_S_IDLE;
}

// ********************************
// Other static (local/private) functions
// ********************************

static void Rad_Init_CPE_Structs()
{
    // Radio initialization sequence (ble5_radio_setup->fs->sync_start_rat)
    cmd_ble5_radio_setup_p->pNextOp = (rfc_radioOp_t*)cmd_fs_p;
    cmd_ble5_radio_setup_p->condition.rule = COND_STOP_ON_FALSE;

    cmd_fs_p->pNextOp = (rfc_radioOp_t*)cmd_sync_start_rat_p;
    cmd_fs_p->condition.rule = COND_STOP_ON_FALSE;

    cmd_sync_start_rat_p->pNextOp = NULL;
    cmd_sync_start_rat_p->condition.rule = COND_NEVER;

    // Initialization of structures related to RAT-RTC synchronization
    cmd_sync_stop_rat_p->condition.rule = COND_NEVER;
}

static void Rad_Handle_Error(uint8_t err_code)
{
    rac.err_code = err_code;
}

