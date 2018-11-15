/*
 * rf_core.c
 *
 *  Created on: Oct 29, 2018
 *      Author: alvaro
 */

#include <stdlib.h>
#include <string.h>

#include <driverlib/prcm.h>
#include <driverlib/sys_ctrl.h>

#include <driverlib/rfc.h>
#include <driverlib/rf_mailbox.h>
#include <driverlib/rf_common_cmd.h>
#include <driverlib/rf_data_entry.h>
#include <inc/hw_rfc_rat.h>
#include <driverlib/rf_ble_mailbox.h>
#include <driverlib/rf_ble_cmd.h>

#include <driverlib/aon_rtc.h>

#include "rf_core.h"
#include "timing.h"
#include "smartrf_settings.h"
#include "log.h"

// RF core control structure
rfc_control_t rfc;

// RF core radio operation structures
static volatile rfc_CMD_BLE5_RADIO_SETUP_t* cmd_ble5_radio_setup_p = &RF_cmdBle5RadioSetup;
static volatile rfc_CMD_FS_t* cmd_fs_p = &RF_cmdFs;
static volatile rfc_CMD_BLE5_ADV_AUX_t* cmd_ble5_adv_aux_p = &RF_cmdBle5AdvAux;
static volatile rfc_CMD_BLE5_SCANNER_t* cmd_ble5_scanner_p = &RF_cmdBle5Scanner;
static volatile rfc_CMD_SYNC_STOP_RAT_t* cmd_sync_stop_rat_p = &RF_cmdSyncStopRat;
static volatile rfc_CMD_SYNC_START_RAT_t* cmd_sync_start_rat_p = &RF_cmdSyncStartRat;

// Buffers, data entries and queues used by the RF core
static volatile rfc_ble5ExtAdvEntry_t ble5_ext_adv_entry;
static volatile uint8_t data_entry_buf[RFC_RX_BUF_LEN];
static volatile rfc_dataEntryPointer_t data_entry_ptr;
static volatile dataQueue_t data_queue;
static volatile rfc_ble5ScanInitOutput_t ble5_scan_init_output;

static void Rfc_HW_Setup();
static void Rfc_Init_CPE_Structs();
static void Rfc_Handle_Error(uint8_t err_code);

void Rfc_Init()
{
    Rfc_HW_Setup();
    Rfc_Init_CPE_Structs();

    memset(&rfc, 0, sizeof(rfc_control_t));
    rfc.state = RFC_S_WAIT_RFC_BOOT;
    Tm_Start_Timeout(TM_RFC_TOUT_ID, RFC_TOUT_BOOT_MSEC);
}

void Rfc_Process()
{
    uint32_t cpe_int_flags = (uint32_t)-1;

    switch (rfc.state)
    {
    case RFC_S_IDLE: // wait for RF core operation request
        break;

    // ********************************
    // RF core initialization
    // ********************************
    case RFC_S_WAIT_RFC_BOOT: // wait until the RF Core boots
        cpe_int_flags = Rfc_Get_CPE_Int_Flags();

        if (cpe_int_flags & RFC_DBELL_RFCPEIFG_BOOT_DONE)
        {
            Rfc_Start_Direct_Cmd(CMD_PING);
            rfc.next_state = RFC_S_EXEC_RADIO_SETUP;
        }
        else if (cpe_int_flags & RFC_DBELL_RFCPEIFG_INTERNAL_ERROR ||
                Tm_Timeout_Completed(TM_RFC_TOUT_ID))
        {
            Rfc_Handle_Error(RFC_ERR_BOOT_FAILED);
            rfc.state = RFC_S_WAIT_ERR_ACTION;
        }
        break;

    case RFC_S_EXEC_RADIO_SETUP:
        Rfc_Start_Radio_Op(cmd_ble5_radio_setup_p, RFC_TOUT_DEFAULT);
        rfc.next_state = RFC_S_EXEC_FS;
        break;

    case RFC_S_EXEC_FS:
        Rfc_Start_Radio_Op(cmd_fs_p, RFC_TOUT_DEFAULT);
        rfc.next_state = RFC_S_EXEC_START_RAT;
        break;

    case RFC_S_EXEC_START_RAT:
        Rfc_Start_Direct_Cmd(CMD_START_RAT);
        rfc.next_state = RFC_S_IDLE;
        Rfc_Set_Flags_On_Success(RFC_F_INITIALIZED);
        break;

    case RFC_S_EXEC_SYNC_START_RAT:
        cmd_sync_start_rat_p->rat0 = cmd_sync_stop_rat_p->rat0;
        Rfc_Start_Radio_Op(cmd_sync_start_rat_p, RFC_TOUT_DEFAULT);
        rfc.next_state = RFC_S_IDLE;
        break;

    // ********************************
    // Handle command execution
    // ********************************
    case RFC_S_WAIT_CPE_READY: // wait until the door bell becomes available
        if (Rfc_CPE_Ready())
        {
            uint32_t operation = 0;

            if (rfc.radio_op_p != NULL) // radio operation or immediate command ?
            {
                operation = (uint32_t)rfc.radio_op_p;
                Rfc_Clear_CPE_Int_Flags(RFC_M_CPE_COMMAND_DONE);
            }
            else
                operation = (uint32_t)rfc.immediate_cmd_p;

            RFCAckIntClear();
            Rfc_Send_To_CPE(operation);
            rfc.state = RFC_S_WAIT_CPE_ACK;
            Tm_Start_Timeout(TM_RFC_TOUT_ID, RFC_TOUT_CPE_ACK_MSEC);
        }
        else if (Tm_Timeout_Completed(TM_RFC_TOUT_ID))
        {
            Rfc_Handle_Error(RFC_ERR_TIMEOUT);
            rfc.state = RFC_S_WAIT_ERR_ACTION;
        }
        break;

    case RFC_S_WAIT_CPE_ACK:
        if (Rfc_CPE_Ack())
        {
            uint32_t cpe_cmd_sta = Rfc_Get_CPE_CMDSTA();

            if (cpe_cmd_sta != CMDSTA_Done)
            {
                Rfc_Handle_Error(RFC_ERR_OPERATION_FAILED);
                rfc.state = RFC_S_WAIT_ERR_ACTION;
            }

            if (rfc.radio_op_p != NULL) // radio operation ?
            {
                rfc.state = RFC_S_WAIT_RADIO_OP_EXECUTION;
                if (rfc.op_timeout)
                    Tm_Start_Timeout(TM_RFC_TOUT_ID, rfc.op_timeout);
                else
                    Tm_Start_Timeout(TM_RFC_TOUT_ID, RFC_TOUT_MAX_OP_TIME_MSEC);
            }
            else // immediate command ?
            {
                Rfc_On_Success_Do(rfc.immediate_cmd_p);
                rfc.state = rfc.next_state;
            }
        }
        else if (Tm_Timeout_Completed(TM_RFC_TOUT_ID))
        {
            Rfc_Handle_Error(RFC_ERR_TIMEOUT);
            rfc.state = RFC_S_WAIT_ERR_ACTION;
        }
        break;

    case RFC_S_WAIT_RADIO_OP_EXECUTION:
        cpe_int_flags = Rfc_Get_CPE_Int_Flags();

        if (cpe_int_flags & RFC_M_CPE_COMMAND_DONE)
        {
            if (rfc.radio_op_p->status & RFC_F_RADIO_OP_STATUS_ERR)
            {
                Rfc_Handle_Error(RFC_ERR_OPERATION_FAILED);
                rfc.state = RFC_S_WAIT_ERR_ACTION;
            }

            Rfc_On_Success_Do(rfc.radio_op_p);
            rfc.state = rfc.next_state;
        }
        else if (cpe_int_flags & RFC_DBELL_RFCPEIFG_INTERNAL_ERROR)
        {
            Rfc_Handle_Error(RFC_ERR_INTERNAL);
            rfc.state = RFC_S_WAIT_ERR_ACTION;
        }
        else if (cpe_int_flags & RFC_DBELL_RFCPEIFG_SYNTH_NO_LOCK)
        {
            Rfc_Handle_Error(RFC_ERR_SYNTH_NO_LOCK);
            rfc.state = RFC_S_WAIT_ERR_ACTION;
        }
        else if (Tm_Timeout_Completed(TM_RFC_TOUT_ID))
        {
            Rfc_Handle_Error(RFC_ERR_TIMEOUT);
            rfc.state = RFC_S_WAIT_ERR_ACTION;
        }
        break;

    // ********************************
    // Error handling
    // ********************************
    case RFC_S_WAIT_ERR_ACTION: // wait until error is read by external module and some action is taken
        break;

    }
}

void Rfc_Set_Tx_Power(rfc_tx_power_t tx_power)
{
    cmd_ble5_adv_aux_p->txPower = tx_power;
}

void Rfc_BLE5_Set_PHY_Mode(uint8_t ble5_mode)
{
    /*
     * Main mode and coding values according to data sheet
     *
     * Main modes => 0: 1 Mbps, 1: 2 Mbps, 2: Coded
     * Coding => 0: S = 8 (125 kbps), 1: S = 2 (500 kbps)
     */

    assertion(ble5_mode < RFC_PHY_MODES_NUM);

    uint8_t main_mode = RFC_PHY_MAIN_MODE_2MBPS; // default mode 2 Mbps
    uint8_t coding = RFC_PHY_CODING_NONE; // default: no coding

    if (ble5_mode == RFC_PHY_MODE_1MBPS)
        main_mode = RFC_PHY_MAIN_MODE_1MBPS;
    else if  (ble5_mode == RFC_PHY_MODE_500KBPS)
    {
        main_mode = RFC_PHY_MAIN_MODE_CODED;
        coding = RFC_PHY_CODING_500KBPS;
    }
    else if  (ble5_mode == RFC_PHY_MODE_125KBPS)
    {
        main_mode = RFC_PHY_MAIN_MODE_CODED;
        coding = RFC_PHY_CODING_125KBPS;
    }

    // Set channel TX and RX commands
    cmd_ble5_adv_aux_p->phyMode.mainMode = main_mode;
    cmd_ble5_adv_aux_p->phyMode.coding = coding;

    cmd_ble5_scanner_p->phyMode.mainMode = main_mode;
    cmd_ble5_scanner_p->phyMode.coding = coding;
}

void Rfc_BLE5_Set_Channel(uint8_t channel)
{
    assertion(channel < 40);

    // Calculate channel offset (BLE channels are not consecutive)
    uint8_t ch_offset = 0;
    if (channel == 37)
        ch_offset = 0;
    else if (channel == 38)
        ch_offset = 12;
    else if (channel == 39)
        ch_offset = 39;
    else if ((channel >= 0) && (channel <= 10))
        ch_offset = channel + 1;
    else if ((channel >= 11) && (channel <= 36))
        ch_offset = channel + 2;

    // Calculate field values (defined in the data sheet)
    ch_offset = ch_offset * 2;
    uint16_t freq = RFC_BLE5_BASE_FREQ + ch_offset;
    uint8_t ch = RFC_BLE5_BASE_CH + ch_offset;
    uint8_t whitening = RFC_BLE5_BASE_WHITE_INIT + channel;

    // Set frequency of synthesizer
    cmd_fs_p->frequency = freq;

    // Set channel TX and RX commands
    cmd_ble5_adv_aux_p->channel = ch;
    cmd_ble5_adv_aux_p->whitening.init = whitening & 0x7F;
    cmd_ble5_adv_aux_p->whitening.bOverride = 1;

    cmd_ble5_scanner_p->channel = ch;
    cmd_ble5_scanner_p->whitening.init = whitening & 0x7F;
    cmd_ble5_scanner_p->whitening.bOverride = 1;
}

bool Rfc_BLE5_Adv_Aux(rfc_tx_param_t* tx_param_p)
{
    if (!Rfc_Ready())
        return false;

    uint16_t timeout_ms = RFC_TOUT_TX_MSEC;

    // set trigger type and start time
    if (tx_param_p->rat_start_time)
    {
        // Calculate the timeout of the operation considering delayed timeout
        uint32_t rat_curr_time = Rfc_Get_RAT_Time();
        uint32_t deta_time = Tm_Delta_Time32(rat_curr_time, tx_param_p->rat_start_time);
        timeout_ms += (deta_time / RAD_RAT_TICKS_PER_MSEC);

        cmd_ble5_adv_aux_p->startTrigger.triggerType = TRIG_ABSTIME;
        cmd_ble5_adv_aux_p->startTime = tx_param_p->rat_start_time;
    }
    else
        cmd_ble5_adv_aux_p->startTrigger.triggerType = TRIG_NOW;

    // Set payload length and pointer
    if (tx_param_p->buf == NULL)
        tx_param_p->len = 0;
    ((rfc_ble5ExtAdvEntry_t*)cmd_ble5_adv_aux_p->pParams->pAdvPkt)->advDataLen = tx_param_p->len;
    ((rfc_ble5ExtAdvEntry_t*)cmd_ble5_adv_aux_p->pParams->pAdvPkt)->pAdvData = tx_param_p->buf;

    // Start radio operation
    Rfc_Clear_CPE_Int_Flags(RFC_M_CPE_TX_INT_FLAGS);
    Rfc_Start_Radio_Op(cmd_ble5_adv_aux_p, timeout_ms);
    rfc.next_state = RFC_S_IDLE;

    return true;
}

bool Rfc_BLE5_Scanner(uint32_t timeout_usec)
{
    if (!Rfc_Ready())
        return false;

    // Reset data entry and queue
    data_entry_ptr.status = DATA_ENTRY_PENDING;
    data_entry_ptr.pData = (uint8_t*)data_entry_buf;
    data_queue.pCurrEntry = (uint8_t*)&data_entry_ptr;

    // Set command end time (Radio Timer)
    cmd_ble5_scanner_p->pParams->timeoutTime = timeout_usec * RAD_RAT_TICKS_PER_USEC; // end time relative to start of command

    // Calculate FSM operation timeout
    uint16_t tout_ms = timeout_usec/1000 + RFC_TOUT_RX_MSEC; // timeout in case the RF core is unresponsive

    // Start radio operation
    Rfc_Clear_CPE_Int_Flags(RFC_M_CPE_RX_INT_FLAGS);
    Rfc_Start_Radio_Op(cmd_ble5_scanner_p, tout_ms);
    rfc.next_state = RFC_S_IDLE;

    return true;
}

void Rfc_BLE5_Get_Scanner_Result(rfc_rx_result_t* dest)
{
    if (dest == NULL)
        return;

    // Check interrupt flags for errors in reception
    dest->err_flags = 0;
    uint32_t cpe_int_flags = Rfc_Get_CPE_Int_Flags();
    if (cpe_int_flags & RFC_DBELL_RFCPEIFG_RX_NOK)
        dest->err_flags |= RFC_F_RX_CRC_ERR;
    if (cmd_ble5_scanner_p->status == BLE_DONE_RXTIMEOUT)
        dest->err_flags |= RFC_F_RX_TOUT_ERR;

    // Copy results if there were no errors
    if (dest->buf != NULL && !((uint8_t)dest->err_flags))
    {
        // Copy payload if buffer was provided
        if (dest->buf != NULL)
        {
            dest->payload_len = data_entry_buf[RFC_RX_BUF_PAYLOAD_LEN_IDX] - 1; // exclude ext header field
            for (size_t n = 0; n < dest->payload_len && n < dest->buf_len; n++) // condition prevents destination buffer overflow
                ((uint8_t*)dest->buf)[n] = data_entry_buf[n + RFC_RX_BUF_PAYLOAD_OFFSET];
        }

        // Get timestamp and RSSI of last received packet
        dest->rat_timestamp = ble5_scan_init_output.timeStamp;
        dest->rssi_db = ble5_scan_init_output.lastRssi;
    }
    else
    {
        // Set special values to indicate that result is invalid
        dest->rat_timestamp = -1;
        dest->rssi_db = -128;
        dest->payload_len = 0;
    }
}

bool Rfc_Synchronize_RAT()
{
    if (!Rfc_Ready())
        return false;

    Rfc_Start_Radio_Op(cmd_sync_stop_rat_p, RFC_TOUT_DEFAULT);
    rfc.next_state = RFC_S_EXEC_SYNC_START_RAT;

    return true;
}

inline bool Rfc_Ready()
{
    return (rfc.state == RFC_S_IDLE);
}

uint8_t Rfc_Error()
{
    return rfc.error.code;  // TODO return the error structure
}

// ********************************
// Static functions
// ********************************
static void Rfc_HW_Setup()
{
    // Turn off RF core
    PRCMPowerDomainOff(PRCM_DOMAIN_RFCORE);
    while (PRCMPowerDomainStatus(PRCM_DOMAIN_RFCORE) == PRCM_DOMAIN_POWER_ON);

    // Clear interrupt flags
    HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) = 0;

    // Set RF mode
    HWREG(PRCM_BASE + PRCM_O_RFCMODESEL) = PRCM_RFCMODESEL_CURR_MODE1; // BLE

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
}

static void Rfc_Init_CPE_Structs()
{
    // CMD_BLE5_ADV_AUX
    memset((void*)&ble5_ext_adv_entry, 0, sizeof(rfc_ble5ExtAdvEntry_t));
    cmd_ble5_adv_aux_p->pParams->pAdvPkt = (uint8_t*)&ble5_ext_adv_entry;

    // CMD_BLE5_SCANNER
    memset((void*)&data_entry_ptr, 0, sizeof(rfc_dataEntryPointer_t));
    data_entry_ptr.config.type = DATA_ENTRY_TYPE_PTR;
    data_entry_ptr.length = sizeof(data_entry_buf);
    data_entry_ptr.pData = (uint8_t*)data_entry_buf;

    data_queue.pCurrEntry = (uint8_t*)&data_entry_ptr;
    data_queue.pLastEntry = NULL;

    memset((void*)&ble5_scan_init_output, 0, sizeof(ble5_scan_init_output));

    cmd_ble5_scanner_p->pParams->pRxQ = (dataQueue_t*)&data_queue;
    cmd_ble5_scanner_p->pParams->timeoutTrigger.triggerType = TRIG_REL_START;
    cmd_ble5_scanner_p->pOutput = (rfc_ble5ScanInitOutput_t*)&ble5_scan_init_output;

    // SYNC_STOP_RAT, SYNC_START_RAT
    cmd_sync_stop_rat_p->condition.rule = COND_NEVER;
    cmd_sync_start_rat_p->condition.rule = COND_NEVER;
}

static void Rfc_Handle_Error(uint8_t err_code)
{
    /*
     * Store context in which occurred the error
     */
    rfc.error.code = err_code;
    rfc.error.fsm_state = rfc.state;

    // Store the value of the RF core registers
    rfc.error.CMDSTA = HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDSTA);
    rfc.error.RFHWIFG = HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFHWIFG);
    rfc.error.RFCPEIFG = HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG);

    // If there was an ongoing operation store command id and status
    rfc.error.cmd_num = 0;
    rfc.error.cmd_status = 0;
    if (rfc.radio_op_p != NULL) // radio operation ?
    {
        rfc.error.cmd_num = rfc.radio_op_p->commandNo;
        rfc.error.cmd_status = rfc.radio_op_p->status;
    }
    else if (rfc.immediate_cmd_p) // immediate or direct command ?
    {
        if ((uint32_t)rfc.immediate_cmd_p & 0x1) // direct command ?
            rfc.error.cmd_num = (uint32_t)rfc.immediate_cmd_p >> 16;
        else
            rfc.error.cmd_num = rfc.immediate_cmd_p->commandNo;
    }

    // Log error
    Log_Line("RFC err:");
    Log_Value("\tcode: ", rfc.error.code);
    Log_Value("\tfsm_st: ", rfc.error.fsm_state);
    Log_Value("\tCMDSTA: ", rfc.error.CMDSTA);
    Log_Value("\tRFHWIFG: ", rfc.error.RFHWIFG);
    Log_Value("\tRFCPEIFG: ", rfc.error.RFCPEIFG);
    Log_Value("\tcmd_num: ", rfc.error.cmd_num);
    Log_Value("\tcmd_sta: ", rfc.error.cmd_status);
}
