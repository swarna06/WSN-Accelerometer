/*
 * radio.h
 *
 *  Created on: 21 oct. 2018
 *      Author: Alvaro
 */

#ifndef RADIO_H_
#define RADIO_H_

#include <driverlib/rf_common_cmd.h>

// PHY modes
#define RAF_PHY_MODE_1MBPS          0
#define RAF_PHY_MODE_2MBPS          1
#define RAF_PHY_MODE_CODED          2
#define RAF_PHY_CODING_NONE         0
#define RAF_PHY_CODING_125KBPS      0
#define RAF_PHY_CODING_500KBPS      1

// RX buffer optional fields
#define RAD_F_LEN_FIELD_EN          0x01
#define RAD_F_CRC_FIELD_EN          0x02
#define RAD_F_RSSI_FIELD_EN         0x04
#define RAD_F_STAT_FIELD_EN         0x08
#define RAD_F_TSTAMP_FIELD_EN       0x10

// Transmission power codes (taken from SmartRF)
typedef enum
{
    PLUS_5dBm = 0x9330,
    PLUS_4dBm = 0x9324,
    PLUS_3dBm = 0x5A1C,
    PLUS_2dBm = 0x4E18,
    PLUS_1dBm = 0x4214,
    ZERO_0dBm = 0x3161,
    MINUS_3dBm = 0x2558,
    MINUS_6dBm = 0x1D52,
    MINUS_9dBm = 0x194E,
    MINUS_12dBm = 0x144B,
    MINUS_15dBm = 0x0CCB,
    MINUS_18dBm = 0x0CC9,
    MINUS_21dBm = 0x0CC7,
} rad_tx_power_t;

// Default radio configuration
#define RAD_DEFAULT_PHY_MODE    RAF_PHY_MODE_2MBPS
#define RAD_DEFAULT_CODING      RAF_PHY_CODING_NONE
#define RAD_DEFAULT_TX_POW      ZERO_0dBm
#define RAD_DEFAULT_CHANNEL     17

// Command parameter structures
typedef struct
{
    bool synch;
    size_t payload_len;
    uint8_t* payload;
} rad_tx_param_t;

// Command result structures
typedef struct
{
    uint32_t timestamp;
} rad_tx_result_t;

// Macros for printing error information
#define Rad_Print_RFCPEIFG_Err() \
        PRINTF("ERR: func: %s, RFCPEIFG: %p\r\n", \
               (void*)HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG))

#define Rad_Print_Radio_Op_Err(op_p) \
        PRINTF("ERR: func: %s, commandNo: %04X, CMDSTA: %p, status: %04X, RFCPEIFG: %p\r\n", \
               __func__, (op_p)->commandNo, (void*)HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDSTA), \
               (op_p)->status, (void*)HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG))

#define Rad_Print_Direct_Cmd_Err(id) \
        PRINTF("ERR: func: %s, commandNo: %04X, CMDSTA: %p, RFCPEIFG: %p\r\n", \
               __func__, id, (void*)HWREG(RFC_DBELL_BASE + RFC_DBELL_O_CMDSTA), \
               (void*)HWREG(RFC_DBELL_BASE + RFC_DBELL_O_RFCPEIFG))

// CPE error interrupt flags mask
#define RAD_M_RFCPEIFG_ERROR    (RFC_DBELL_RFCPEIFG_INTERNAL_ERROR | \
                                 RFC_DBELL_RFCPEIFG_SYNTH_NO_LOCK)

// Error flag for CMDSTA register and status field in
#define RAD_F_STATUS_ERR        0x0800

// Return values
#define RAD_OK                  0
#define RAD_E_CPE_CMDSTA        -1
#define RAD_E_RFCPEIFG          -2
#define RAD_E_STATUS_FIELD      -3

void Rad_Init();

void Rad_Set_PHY_Mode(uint8_t mode, uint8_t coding);

void Rad_Set_Tx_Power(rad_tx_power_t tx_pow);

void Rad_Set_Channel(uint8_t channel);

int Rad_Ble5_Adv_Aux(rad_tx_param_t* tx_param,
                     rad_tx_result_t* tx_result);

#endif /* RADIO_H_ */
