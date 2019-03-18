/*
 * radio.c
 *
 *  Created on: Mar 7, 2019
 *      Author: alvaro
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <driverlib/rfc.h>
#include <driverlib/rf_data_entry.h>
#include <driverlib/rf_ble_mailbox.h>

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

static void Rad_S_Wait_Packet_Tx();
static void Rad_S_Wait_Packet_Rx();

static void Rad_S_Wait_Immed_Cmd_Execution();

static void Rad_S_Wait_Err_Cleared();

// Other static functions
static void Rad_Init_CPE_Structs();
static void Rad_Set_Radio_Op_Start_Time(rfc_radioOp_t* radio_op_p,
                                        bool delayed_start,
                                        uint32_t start_time);
static size_t Rad_Get_Rx_Payload_Len();
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

     [RAD_S_WAIT_PACKET_TX] = Rad_S_Wait_Packet_Tx,
     [RAD_S_WAIT_PACKET_RX] = Rad_S_Wait_Packet_Rx,

     [RAD_S_WAIT_IMMED_CMD_EXEC] = Rad_S_Wait_Immed_Cmd_Execution,

     [RAD_S_WAIT_ERR_CLEARED] = Rad_S_Wait_Err_Cleared
};

// RF core radio operation structures
static volatile rfc_CMD_BLE5_RADIO_SETUP_t* cmd_ble5_radio_setup_p = &RF_cmdBle5RadioSetup;
static volatile rfc_CMD_FS_t* cmd_fs_p = &RF_cmdFs;
static volatile rfc_CMD_SYNC_START_RAT_t* cmd_sync_start_rat_p = &RF_cmdSyncStartRat;
static volatile rfc_CMD_SYNC_STOP_RAT_t* cmd_sync_stop_rat_p = &RF_cmdSyncStopRat;
static volatile rfc_radioOp_t* rad_config_cmd_chain_p = (rfc_radioOp_t*)&RF_cmdBle5RadioSetup;

static volatile rfc_CMD_BLE5_ADV_AUX_t* cmd_ble5_adv_aux_p = &RF_cmdBle5AdvAux;
static volatile rfc_CMD_BLE5_SCANNER_t* cmd_ble5_scanner_p = &RF_cmdBle5Scanner;

// Buffers, data entries and queues used by the RF core
static volatile rfc_ble5ExtAdvEntry_t ble5_ext_adv_entry;
static volatile uint8_t data_entry_buf[RAD_RX_BUF_LEN];
static const volatile uint8_t* rx_payload_p = &data_entry_buf[RAD_RX_BUF_PAYLOAD_OFFSET];
static volatile rfc_dataEntryPointer_t data_entry_ptr;
static volatile dataQueue_t data_queue;
static volatile rfc_ble5ScanInitOutput_t ble5_scan_init_output;

// Immediate commands
static rfc_CMD_SET_RAT_CMP_t cmd_set_rat_cmp = {.commandNo = CMD_SET_RAT_CMP, .ratCh = RAD_RAT_CH};

// ********************************
// Non-static (public) functions
// ********************************

void Rad_Init()
{
    rac.state = RAD_S_IDLE;
    rac.flags = 0;
    rac.err_code = 0;

    Rad_Init_CPE_Structs();

    // Set default radio configuration
    Rad_Set_Data_Rate(RAD_DEFAULT_DATA_RATE);
    Rad_Set_Tx_Power(RAD_DEFAULT_TX_POW);
    Rad_Set_Freq_Channel(RAD_DEFAULT_FREQ_CH);
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

bool Rad_Set_Tx_Power(rad_tx_pow_t tx_power)
{
    if (rac.state != RAD_S_IDLE)
        return false;

    cmd_ble5_adv_aux_p->txPower = tx_power;
    return true;
}

bool Rad_Set_Freq_Channel(uint8_t channel_num)
{
    if (rac.state != RAD_S_IDLE)
        return false;

    assertion(channel_num < 40);

    // Calculate channel offset (BLE channels are not consecutive)
    uint8_t ch_offset = 0;
    if (channel_num == 37)
        ch_offset = 0;
    else if (channel_num == 38)
        ch_offset = 12;
    else if (channel_num == 39)
        ch_offset = 39;
    else if (channel_num <= 10) // 0 <= ch_num <= 10
        ch_offset = channel_num + 1;
    else if ((channel_num >= 11) && (channel_num <= 36))
        ch_offset = channel_num + 2;

    // Calculate field values (defined in the data sheet)
    ch_offset = ch_offset * 2;
    uint16_t freq = RAD_BLE5_BASE_FREQ + ch_offset;
    uint8_t ch = RAD_BLE5_BASE_CH + ch_offset;
    uint8_t whitening = RAD_BLE5_BASE_WHITE_INIT + channel_num;

    // Set frequency of synthesizer
    cmd_fs_p->frequency = freq;

    // Set channel TX and RX commands
    cmd_ble5_adv_aux_p->channel = ch;
    cmd_ble5_adv_aux_p->whitening.init = whitening & 0x7F;
    cmd_ble5_adv_aux_p->whitening.bOverride = 1;

    cmd_ble5_scanner_p->channel = ch;
    cmd_ble5_scanner_p->whitening.init = whitening & 0x7F;
    cmd_ble5_scanner_p->whitening.bOverride = 1;

    return true;
}

bool Rad_Transmit_Packet(rad_tx_param_t* tx_param_p)
{
    assertion(tx_param_p != NULL);
    assertion(tx_param_p->payload_len <= RAD_MAX_PAYLOAD_LEN);

    if (rac.state != RAD_S_IDLE || Cpe_Ready() == false) // busy ?
        return false;

    // Set trigger type and start time
    Rad_Set_Radio_Op_Start_Time((rfc_radioOp_t*)cmd_ble5_adv_aux_p,
                                tx_param_p->delayed_start,
                                tx_param_p->start_time);

    // Set payload length and pointer
    if (tx_param_p->payload_p == NULL)
        tx_param_p->payload_len = 0;
    rfc_ble5ExtAdvEntry_t* adv_entry_p = (rfc_ble5ExtAdvEntry_t*)cmd_ble5_adv_aux_p->pParams->pAdvPkt;
    adv_entry_p->advDataLen = tx_param_p->payload_len;
    adv_entry_p->pAdvData = tx_param_p->payload_p;

    // Start radio operation
    Cpe_Start_Radio_Op((rfc_radioOp_t*)cmd_ble5_adv_aux_p, 0); // xxx timeout
    rac.state = RAD_S_WAIT_PACKET_TX;
    return true;
}

bool Rad_Receive_Packet(rad_rx_param_t* rx_param_p)
{
    assertion(rx_param_p != NULL);

    if (rac.state != RAD_S_IDLE || Cpe_Ready() == false) // busy ?
        return false;

    rac.rx_param_p = rx_param_p;

    // Set trigger type and start time
    Rad_Set_Radio_Op_Start_Time((rfc_radioOp_t*)cmd_ble5_scanner_p,
                                rx_param_p->delayed_start,
                                rx_param_p->start_time);

    // Reset data entry and queue
    data_entry_ptr.status = DATA_ENTRY_PENDING;
    data_entry_ptr.pData = (uint8_t*)data_entry_buf;
    data_queue.pCurrEntry = (uint8_t*)&data_entry_ptr;

    // Set command end time (relative to the start time)
    cmd_ble5_scanner_p->pParams->timeoutTime = Rad_Microsec_To_RAT_Ticks(rx_param_p->timeout_usec);

    // Start radio operation
    Cpe_Start_Radio_Op((rfc_radioOp_t*)cmd_ble5_scanner_p, 0); // xxx timeout
    rac.state = RAD_S_WAIT_PACKET_RX;
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

bool Rad_Set_RAT_Cmp_Val(uint32_t compare_val, void (*isr)())
{
    if (rac.state != RAD_S_IDLE)
        return false;

    if (isr != NULL) (void)0;
    cmd_set_rat_cmp.compareTime = compare_val;

    Cpe_Start_Immediate_Cmd((rfc_command_t*)&cmd_set_rat_cmp);
    rac.state = RAD_S_WAIT_IMMED_CMD_EXEC;

    return true;
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

static void Rad_S_Wait_Packet_Tx()
{
    if (Cpe_Ready() == true) // radio operation finished ?
        rac.state = RAD_S_IDLE;
    else if (Cpe_Get_Err_Code() != CPE_ERR_NONE)
    {
        Rad_Handle_Error(RAD_ERR_RADIO_OP_FAILED);
        rac.state = RAD_S_WAIT_ERR_CLEARED;
    }
}

static void Rad_S_Wait_Packet_Rx()
{
    if (Cpe_Ready() == true) // radio operation finished ?
    {
        // Check if an error occurred during reception
        rac.rx_param_p->error = RAD_RX_ERR_NONE;
        if (cmd_ble5_scanner_p->status != BLE_DONE_OK)
        {
            if (cmd_ble5_scanner_p->status == DONE_RXERR)
                rac.rx_param_p->error = RAD_RX_ERR_CRC;
            else if (cmd_ble5_scanner_p->status == BLE_DONE_RXTIMEOUT)
                rac.rx_param_p->error = RAD_RX_ERR_TIMEOUT;
            else
                rac.rx_param_p->error = RAD_RX_ERR_OTHER;
        }

        // Copy results if there were no errors
        if (rac.rx_param_p->error == RAD_RX_ERR_NONE)
        {
            // Copy payload if buffer was provided
            if (rac.rx_param_p->dest_buf != NULL && rac.rx_param_p->dest_buf_len > 0)
            {
                rac.rx_param_p->payload_len = Rad_Get_Rx_Payload_Len();
                // Check if payload length is greater than buffer capacity
                size_t length = rac.rx_param_p->payload_len;
                if (rac.rx_param_p->payload_len > rac.rx_param_p->dest_buf_len)
                    length = rac.rx_param_p->dest_buf_len;

                // Copy payload into destination buffer
                memcpy(rac.rx_param_p->dest_buf, (void*)rx_payload_p, length);
            }

            // Set reception information
            rac.rx_param_p->timestamp = ble5_scan_init_output.timeStamp;
            rac.rx_param_p->rssi_dBm = ble5_scan_init_output.lastRssi;
        }
        else
        {
            // Clear fields to indicate that they are invalid
            rac.rx_param_p->payload_len = 0;
            rac.rx_param_p->timestamp = 0;
            rac.rx_param_p->rssi_dBm = 0;
        }

        rac.state = RAD_S_IDLE;
    }
    else if (Cpe_Get_Err_Code() != CPE_ERR_NONE)
    {
        Rad_Handle_Error(RAD_ERR_RADIO_OP_FAILED);
        rac.state = RAD_S_WAIT_ERR_CLEARED;
    }
}

static void Rad_S_Wait_Immed_Cmd_Execution()
{
    if (Cpe_Ready() == true) // radio operation finished ?
        rac.state = RAD_S_IDLE;
    else if (Cpe_Get_Err_Code() != CPE_ERR_NONE)
    {
        Rad_Handle_Error(RAD_ERR_IMMED_CMD_FAILED);
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

    // Initialization of structures related to transmission and reception
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
}

static inline void Rad_Set_Radio_Op_Start_Time(rfc_radioOp_t* radio_op_p,
                                               bool delayed_start,
                                               uint32_t start_time)
{
    if (delayed_start == true)
    {
        radio_op_p->startTrigger.triggerType = TRIG_ABSTIME;
        radio_op_p->startTime = start_time;
    }
    else
        radio_op_p->startTrigger.triggerType = TRIG_NOW;
}

static inline size_t Rad_Get_Rx_Payload_Len()
{
    return (data_entry_buf[RAD_RX_BUF_PAYLOAD_LEN_IDX] - 1);
}

static void Rad_Handle_Error(uint8_t err_code)
{
    rac.err_code = err_code;
}
