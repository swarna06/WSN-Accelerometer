/*
 * radio.c
 *
 *  Created on: 21 oct. 2018
 *      Author: Alvaro
 */

#include <stdint.h>
#include <string.h>

#include <driverlib/prcm.h>
#include <driverlib/sys_ctrl.h>

#include <driverlib/rfc.h>
#include <driverlib/rf_mailbox.h>
#include <driverlib/rf_common_cmd.h>
#include <driverlib/rf_ble_mailbox.h>
#include <driverlib/rf_ble_cmd.h>
#include <driverlib/rf_data_entry.h>
#include <inc/hw_rfc_rat.h>

#include <driverlib/aon_rtc.h>

#include "radio.h"
#include "smartrf_settings.h"
#include "timing.h"

#include "misc.h"
#include "printf.h"

// Pointers to SmartRF command structures
static volatile rfc_CMD_BLE5_RADIO_SETUP_t* cmd_ble5_radio_setup = &RF_cmdBle5RadioSetup;
static volatile rfc_CMD_FS_t* cmd_fs = &RF_cmdFs;
static volatile rfc_CMD_BLE5_ADV_AUX_t* cmd_ble5_adv_aux = &RF_cmdBle5AdvAux;
static volatile rfc_CMD_BLE5_SCANNER_t* cmd_ble5_scanner = &RF_cmdBle5Scanner;
static volatile rfc_CMD_SYNC_STOP_RAT_t* cmd_sync_stop_rat = &RF_cmdSyncStopRat;
static volatile rfc_CMD_SYNC_START_RAT_t* cmd_sync_start_rat = &RF_cmdSyncStartRat;

// Data entries TODO should them be memory aligned ?
static volatile rfc_ble5ExtAdvEntry_t ble5_ext_adv_entry;
static volatile uint8_t data_entry_buf[RAD_DATA_ENTRY_BUF_LEN];
static volatile rfc_dataEntryPointer_t data_entry_ptr;
static volatile dataQueue_t data_queue;

// Local functions
static void Rad_Boot_RF_Core();
static void Rad_Set_PHY(rfc_ble5RadioOp_t* ble5_radio_op_p,
                        uint8_t main_mode,
                        uint8_t coding);
static void Rad_Set_Ch(rfc_ble5RadioOp_t* ble5_radio_op_p,
                       uint8_t channel,
                       uint8_t whitening);
static int Rad_Execute_Radio_Op(volatile rfc_radioOp_t* radio_op_p);
static int Rad_Execute_Direct_Cmd(uint16_t cmd_id);
static void Rad_Reset_Radio_Op_Struct(rfc_radioOp_t* radio_op_p);
static void Rad_Reset_Adv_Entry(volatile rfc_ble5ExtAdvEntry_t* adv_entry_p);
static void Rad_Reset_Adv_Aux_Struct(volatile rfc_CMD_BLE5_ADV_AUX_t* cmd_ble5_adv_aux_p);

static void Rad_Reset_Data_Entry(volatile rfc_dataEntryPointer_t* data_entry_p);
static void Rad_Reset_Data_Queue(volatile dataQueue_t* data_queue_p);
static void Rad_Reset_Scanner_Struct(volatile rfc_CMD_BLE5_SCANNER_t* cmd_ble5_scanner_p);

static uint16_t Rad_Synch_RAT();

void Rad_Init()
{
    // Hardware initialization
    Rad_Boot_RF_Core();

    // Reset radio operation structures
    Rad_Reset_Radio_Op_Struct((rfc_radioOp_t*)cmd_ble5_radio_setup);
    Rad_Reset_Radio_Op_Struct((rfc_radioOp_t*)cmd_fs);
    Rad_Reset_Adv_Aux_Struct(cmd_ble5_adv_aux);
    Rad_Reset_Scanner_Struct(cmd_ble5_scanner);
    Rad_Reset_Radio_Op_Struct((rfc_radioOp_t*)cmd_sync_stop_rat);
    Rad_Reset_Radio_Op_Struct((rfc_radioOp_t*)cmd_sync_start_rat);

    // Set default channel, PHY mode and TX power
    cmd_ble5_radio_setup->defaultPhy.mainMode = RAD_DEFAULT_PHY_MODE;
    cmd_ble5_radio_setup->defaultPhy.coding = RAD_DEFAULT_CODING;
    cmd_ble5_radio_setup->txPower = RAD_DEFAULT_TX_POW;

    Rad_Set_PHY_Mode(RAD_DEFAULT_PHY_MODE, RAD_DEFAULT_CODING);
    Rad_Set_Tx_Power(RAD_DEFAULT_TX_POW);
    Rad_Set_Channel(RAD_DEFAULT_CHANNEL);

    // Startup sequence
    int result;
    rfc_radioOp_t* radio_op_p = NULL;

    result = Rad_Execute_Radio_Op(radio_op_p = (rfc_radioOp_t*)cmd_ble5_radio_setup); // radio setup
    if (result != RAD_OK || radio_op_p->status != BLE_DONE_OK)
        goto handle_err;

    Rad_Execute_Radio_Op(radio_op_p = (rfc_radioOp_t*)cmd_fs); // start synthesizer
    if (result != RAD_OK || radio_op_p->status != DONE_OK)
        goto handle_err;

    Rad_Execute_Direct_Cmd(CMD_START_RAT); // start radio timer
    if (result != RAD_OK)
        exit(0);

    return;

handle_err:
    Rad_Print_Radio_Op_Err(radio_op_p);
    exit(0);
}

void Rad_Set_PHY_Mode(uint8_t mode, uint8_t coding)
{
    // Set PHY mode in RX/TX commands
    Rad_Set_PHY((rfc_ble5RadioOp_t*)cmd_ble5_adv_aux, mode, coding);
    Rad_Set_PHY((rfc_ble5RadioOp_t*)cmd_ble5_scanner, mode, coding);
}

void Rad_Set_Tx_Power(rad_tx_power_t tx_pow)
{
    cmd_ble5_adv_aux->txPower = tx_pow;
}

void Rad_Set_Channel(uint8_t channel)
{
    assertion(channel <= 39);

    // Base values for calculating fields of radio operation structures
    const uint16_t BASE_FREQ = 2402; // frequency channel 37
    const uint8_t BASE_CH = 0x66; // id channel 37
    const uint8_t BASE_WHITENING_INIT = 0x40; // whitening initial value for channel 0

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
    uint16_t freq = BASE_FREQ + ch_offset;
    uint8_t whitening = BASE_WHITENING_INIT + channel;
    uint8_t ch = BASE_CH + ch_offset;

    // Set frequency of synthesizer
    cmd_fs->frequency = freq;

    // Set channel in RX/TX commands
    Rad_Set_Ch((rfc_ble5RadioOp_t*)cmd_ble5_adv_aux, ch, whitening);
    Rad_Set_Ch((rfc_ble5RadioOp_t*)cmd_ble5_scanner, ch, whitening);
}

int Rad_Ble5_Adv_Aux(uint8_t* payload, size_t payload_len)
{
    assertion(cmd_ble5_adv_aux->pParams->pAdvPkt == (uint8_t *)&ble5_ext_adv_entry);

    // Add payload to advertising packet
    if (payload != NULL)
    {
        ble5_ext_adv_entry.pAdvData = payload;
        ble5_ext_adv_entry.advDataLen = payload_len;
    }
    else
    {
        ble5_ext_adv_entry.pAdvData = NULL;
        ble5_ext_adv_entry.advDataLen = 0;
    }

    // Execute command
    int result = Rad_Execute_Radio_Op((rfc_radioOp_t*)cmd_ble5_adv_aux);
    if (result < 0)
    {
        Rad_Print_Radio_Op_Err(cmd_ble5_adv_aux);
        exit(0);
    }

    return 0;
}

int Rad_Ble5_Scanner(uint8_t* buffer, size_t buf_len)
{
    Rad_Reset_Data_Queue(&data_queue);

    // Execute command
    int result = Rad_Execute_Radio_Op((rfc_radioOp_t*)cmd_ble5_scanner);
    if (result < 0)
    {
        Rad_Print_Radio_Op_Err(cmd_ble5_scanner);
        exit(0);
    }

    size_t payload_len = data_entry_buf[RAD_DENTRY_LEN_IDX];
    uint8_t *payload = (uint8_t*)data_entry_buf + RAD_DENTRY_PAYLOAD_IDX;

    if (payload_len > buf_len)
        payload_len = buf_len;

    memcpy(buffer, payload, payload_len);

    return payload_len;
}

void Rad_Synch_Master()
{
    uint32_t rat_synch_time, rtc_synch_time;
    uint32_t rat_timestamp_tx, rat_timestamp_rx;

    // Prepare empty packet
    ble5_ext_adv_entry.pAdvData = NULL;
    ble5_ext_adv_entry.advDataLen = 0;

    // Prepare tx and rx commands
    cmd_ble5_adv_aux->status = IDLE;
    cmd_ble5_adv_aux->startTrigger.triggerType = TRIG_ABSTIME;
    cmd_ble5_adv_aux->pNextOp = (rfc_radioOp_t *)cmd_ble5_scanner;
    cmd_ble5_adv_aux->condition.rule = COND_STOP_ON_FALSE;

    Rad_Reset_Data_Queue(&data_queue);
    cmd_ble5_scanner->status = IDLE;
    cmd_ble5_scanner->startTrigger.triggerType = TRIG_NOW;
//    cmd_ble5_scanner->pParams->endTrigger.triggerType = TRIG_REL_START; // TODO add timeout
//    cmd_ble5_scanner->pParams->endTime = xxx;

    // Synchronize clocks
    Rad_Synch_RAT();
    Tm_Synch_With_RTC(); // CPU waits until the start of next RTC period
    rat_synch_time = HWREG(RFC_RAT_BASE + RFC_RAT_O_RATCNT); // get RAT time right after RTC edge
    rtc_synch_time = AONRTCCurrentCompareValueGet(); // TODO is it necessary to get this time stamp ?
    rat_timestamp_tx = rat_synch_time + 1024; // transmission will start after N RAT cycles xxx (enough time to prepare command structures)
    cmd_ble5_adv_aux->startTime = rat_timestamp_tx; // set the TX start time

    // Send command to the RF Core and wait until ACK is received
    while(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDR) != 0); // wait until RFC door bell is ready
    RFCAckIntClear();
    HWREG(RFC_DBELL_BASE+RFC_DBELL_O_CMDR) = (uint32_t)cmd_ble5_adv_aux;
    while(!HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG)); // wait for ACK TODO not necessary ?
    RFCAckIntClear();

    // Wait for operation execution TODO
    while (cmd_ble5_adv_aux->status <= ACTIVE)
    {
        if (HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) & RAD_M_RFCPEIFG_ERROR) // critical error ?
        {
            Rad_Print_Radio_Op_Err(cmd_ble5_adv_aux);
            exit(0);
        }
    }
    if (cmd_ble5_adv_aux->status & RAD_F_STATUS_ERR) // error in the status field of the command structure ?
    {
        Rad_Print_Radio_Op_Err(cmd_ble5_adv_aux);
        exit(0);
    }

    while (cmd_ble5_scanner->status <= ACTIVE)
    {
        if (HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) & RAD_M_RFCPEIFG_ERROR) // critical error ?
        {
            Rad_Print_Radio_Op_Err(cmd_ble5_scanner);
            exit(0);
        }
    }
    if (cmd_ble5_scanner->status & RAD_F_STATUS_ERR) // error in the status field of the command structure ?
    {
        Rad_Print_Radio_Op_Err(cmd_ble5_scanner);
        exit(0);
    }

    // Get reception time stamp
    rat_timestamp_rx = Rad_Get_Pkt_Timestamp(data_entry_buf);

    // Print results
    uint32_t round_trip_time = Tm_Delta_Time32(rat_timestamp_tx, rat_timestamp_rx);
    PRINTF("%rtc_timestamp: %lu, round_trip_time: %lu ns\r\n",
           rtc_synch_time, round_trip_time * RAD_RAT_NSEC_PER_TICK);

    // Restore tx and rx commands
    Rad_Reset_Adv_Aux_Struct(cmd_ble5_adv_aux);
    Rad_Reset_Scanner_Struct(cmd_ble5_scanner);
}

void Rad_Synch_Slave()
{
    uint32_t rat_timestamp_rx;
    uint32_t rat_synch_time, rtc_synch_time;

    // Prepare empty packet
    ble5_ext_adv_entry.pAdvData = NULL;
    ble5_ext_adv_entry.advDataLen = 0;

    // Prepare tx and rx commands
    Rad_Reset_Data_Queue(&data_queue);
    cmd_ble5_scanner->status = IDLE;
    cmd_ble5_scanner->startTrigger.triggerType = TRIG_NOW;
    cmd_ble5_scanner->pNextOp = (rfc_radioOp_t *)cmd_ble5_adv_aux;
    cmd_ble5_scanner->condition.rule = COND_ALWAYS; // TODO change rule
//    cmd_ble5_scanner->pParams->endTrigger.triggerType = TRIG_REL_START; // TODO add timeout
//    cmd_ble5_scanner->pParams->endTime = xxx;

    cmd_ble5_adv_aux->status = IDLE;
    cmd_ble5_adv_aux->startTrigger.triggerType = TRIG_NOW;

    // Synchronize clocks
    Rad_Synch_RAT();
    Tm_Synch_With_RTC(); // CPU waits until the start of next RTC period
    rat_synch_time = HWREG(RFC_RAT_BASE + RFC_RAT_O_RATCNT); // get RAT time right after RTC edge
    rtc_synch_time = AONRTCCurrentCompareValueGet(); // TODO is it necessary to get this time stamp ?

    // Send command to the RF Core and wait until ACK is received
    while(HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDR) != 0); // wait until RFC door bell is ready
    RFCAckIntClear();
    HWREG(RFC_DBELL_BASE+RFC_DBELL_O_CMDR) = (uint32_t)cmd_ble5_scanner;
    while(!HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFACKIFG)); // wait for ACK TODO not necessary ?
    RFCAckIntClear();

    // Wait for operation execution TODO
    while (cmd_ble5_scanner->status <= ACTIVE)
    {
        if (HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) & RAD_M_RFCPEIFG_ERROR) // critical error ?
        {
            Rad_Print_Radio_Op_Err(cmd_ble5_scanner);
            exit(0);
        }
    }
    if (cmd_ble5_scanner->status & RAD_F_STATUS_ERR) // error in the status field of the command structure ?
    {
        Rad_Print_Radio_Op_Err(cmd_ble5_scanner);
        exit(0);
    }

    while (cmd_ble5_adv_aux->status <= ACTIVE)
    {
        if (HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG) & RAD_M_RFCPEIFG_ERROR) // critical error ?
        {
            Rad_Print_Radio_Op_Err(cmd_ble5_adv_aux);
            exit(0);
        }
    }
    if (cmd_ble5_adv_aux->status & RAD_F_STATUS_ERR) // error in the status field of the command structure ?
    {
        Rad_Print_Radio_Op_Err(cmd_ble5_adv_aux);
        exit(0);
    }

    // Get reception time stamp
    rat_timestamp_rx = Rad_Get_Pkt_Timestamp(data_entry_buf);

    // Print results
    PRINTF("%rtc_synch_time: %lu, rat_synch_time: %lu, rat_timestamp_rx: %lu, \r\n",
           rtc_synch_time, rat_synch_time, rat_timestamp_rx);

    // Restore tx and rx commands
    Rad_Reset_Adv_Aux_Struct(cmd_ble5_adv_aux);
    Rad_Reset_Scanner_Struct(cmd_ble5_scanner);
}

//********************************
// Static functions
//********************************

static void Rad_Boot_RF_Core()
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

static void Rad_Set_PHY(rfc_ble5RadioOp_t* ble5_radio_op_p,
                        uint8_t main_mode,
                        uint8_t coding)
{
    assertion(main_mode <= RAF_PHY_MODE_CODED);
    assertion(coding <= RAF_PHY_CODING_500KBPS);

    // Set PHY mode fields as specified in the data sheet
    ble5_radio_op_p->phyMode.mainMode = main_mode;
    ble5_radio_op_p->phyMode.coding = coding;
}

static void Rad_Set_Ch(rfc_ble5RadioOp_t* ble5_radio_op_p,
                       uint8_t channel,
                       uint8_t whitening)
{
    ble5_radio_op_p->channel = channel; // channel value as specified in the data sheet
    if (whitening) // use custom whitening initialization value (from SmartRF)
    {
        ble5_radio_op_p->whitening.init = whitening & 0x7F;
        ble5_radio_op_p->whitening.bOverride = 1;
    }
    else
        ble5_radio_op_p->whitening.bOverride = 0; // use default whitening for BLE
}

static int Rad_Execute_Radio_Op(volatile rfc_radioOp_t* radio_op_p)
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

static int Rad_Execute_Direct_Cmd(uint16_t cmd_id)
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

static void Rad_Reset_Radio_Op_Struct(rfc_radioOp_t* radio_op_p)
{
    // Reset common fields in radio operation structure (default values)
    assertion(radio_op_p != NULL);
    radio_op_p->pNextOp = NULL; // next operation pointer

    radio_op_p->startTrigger.triggerType = TRIG_NOW; // command triggers immediately
    radio_op_p->startTrigger.bEnaCmd = 0;
    radio_op_p->startTrigger.triggerNo = 0;
    radio_op_p->startTrigger.pastTrig = 0;

    radio_op_p->condition.rule = COND_NEVER; // next operation is never executed
    radio_op_p->condition.nSkip = 0;
}

static void Rad_Reset_Adv_Entry(volatile rfc_ble5ExtAdvEntry_t* adv_entry_p)
{
    // Empty packet
    adv_entry_p->extHdrInfo.advMode = 0;
    adv_entry_p->extHdrInfo.length = 0;
    adv_entry_p->extHdrFlags = 0;
    adv_entry_p->extHdrConfig.bSkipAdvA = 0;
    adv_entry_p->extHdrConfig.bSkipTargetA = 0;
    adv_entry_p->extHdrConfig.deviceAddrType = 0;
    adv_entry_p->extHdrConfig.targetAddrType = 0;
    adv_entry_p->advDataLen = 0;
    adv_entry_p->pAdvData = NULL;
    adv_entry_p->pExtHeader = NULL;
}

static void Rad_Reset_Adv_Aux_Struct(volatile rfc_CMD_BLE5_ADV_AUX_t *cmd_ble5_adv_aux_p)
{
    Rad_Reset_Radio_Op_Struct((rfc_radioOp_t*)cmd_ble5_adv_aux_p);

    // Reset advertising data entry and store pointer in command parameters
    Rad_Reset_Adv_Entry(&ble5_ext_adv_entry);
    cmd_ble5_adv_aux_p->pParams->pAdvPkt = (uint8_t*)&ble5_ext_adv_entry;
}

static void Rad_Reset_Data_Entry(volatile rfc_dataEntryPointer_t* data_entry_p)
{
    memset((rfc_dataEntryPointer_t*)data_entry_p, 0, sizeof(rfc_dataEntryPointer_t));
    data_entry_p->config.type = DATA_ENTRY_TYPE_PTR;
    data_entry_p->length = sizeof(data_entry_buf);
    data_entry_p->pData = (uint8_t*)data_entry_buf;
}

static void Rad_Reset_Data_Queue(volatile dataQueue_t* data_queue_p)
{
    Rad_Reset_Data_Entry(&data_entry_ptr);
    data_queue_p->pCurrEntry = (uint8_t*)&data_entry_ptr;
    data_queue_p->pLastEntry = NULL;
}

static void Rad_Reset_Scanner_Struct(volatile rfc_CMD_BLE5_SCANNER_t* cmd_ble5_scanner_p)
{
    Rad_Reset_Radio_Op_Struct((rfc_radioOp_t*)cmd_ble5_scanner_p);
    Rad_Reset_Data_Queue(&data_queue);

    assertion(cmd_ble5_scanner_p->pParams != NULL);
    cmd_ble5_scanner_p->pParams->pRxQ = (dataQueue_t*)&data_queue;

    cmd_ble5_scanner_p->pParams->rxConfig.bAppendRssi = 1;
    cmd_ble5_scanner_p->pParams->rxConfig.bAppendStatus = 0;
    cmd_ble5_scanner_p->pParams->rxConfig.bAppendTimestamp = 1;
    cmd_ble5_scanner_p->pParams->rxConfig.bIncludeCrc = 0;
    cmd_ble5_scanner_p->pParams->rxConfig.bIncludeLenByte = 1;
}

static uint16_t Rad_Synch_RAT()
{
    int result;

    // Stop RAT
    result = Rad_Execute_Radio_Op((rfc_radioOp_t*)cmd_sync_stop_rat);
    if (result < 0)
    {
        Rad_Print_Radio_Op_Err(cmd_sync_stop_rat);
        exit(0);
    }
    // Use result as a parameter in START_RAT
    cmd_sync_start_rat->rat0 = cmd_sync_stop_rat->rat0;

    // Synchronize with RTC and start RAT
    result = Rad_Execute_Radio_Op((rfc_radioOp_t*)cmd_sync_start_rat);
    if (result < 0)
    {
        Rad_Print_Radio_Op_Err(cmd_sync_start_rat);
        exit(0);
    }

    return cmd_sync_start_rat->status;
}
