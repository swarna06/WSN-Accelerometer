/*
 * radio.h
 *
 *  Created on: Mar 5, 2019
 *      Author: alvaro
 */

#ifndef RADIO_H_
#define RADIO_H_

#include <stdint.h>
#include <stdbool.h>

#include <driverlib/rfc.h>
#include <inc/hw_rfc_rat.h>

#include "cp_engine.h"

// ********************************
// RF core API
// ********************************


// ********************************
// Timing
// ********************************

// Macro functions to obtain and convert RAT time
#define Rad_Get_Radio_Time()            Cpe_Get_RAT_Time()
#define Rad_RAT_Ticks_To_Nanosec(t)     ((t)*RAD_RAT_NSEC_PER_TICK)
#define Rad_RAT_Ticks_To_Microsec(t)    ((t)/RAD_RAT_TICKS_PER_USEC)

#define Rad_Microsec_To_RAT_Ticks(t)    ((t)*RAD_RAT_TICKS_PER_USEC)

// RAT time conversion
enum
{
    RAD_RAT_NSEC_PER_TICK = 250,
    RAD_RAT_USEC_PER_TICK = RAD_RAT_NSEC_PER_TICK * 1000,
    RAD_RAT_MSEC_PER_TICK = RAD_RAT_USEC_PER_TICK * 1000,

    RAD_RAT_TICKS_PER_USEC = 1000 / RAD_RAT_NSEC_PER_TICK,
    RAD_RAT_TICKS_PER_MSEC = RAD_RAT_TICKS_PER_USEC * 1000,

    RAD_RAT_TICKS_PER_RTC_TICK = 122, // 30.5175 usec / 0.25 usec = 122.07 (~122)
};

// ********************************
// BLE 5
// ********************************

// BLE5 PHY modes
enum
{
    RAD_BLE5_PHY_MAIN_MODE_1MBPS = 0,
    RAD_BLE5_PHY_MAIN_MODE_2MBPS = 1,
    RAD_BLE5_PHY_MAIN_MODE_CODED = 2,
};

// BLE5 channel coding
enum
{
    RAD_BLE5_PHY_CODING_NONE = 0,
    RAD_BLE5_PHY_CODING_125KBPS = 0,
    RAD_BLE5_PHY_CODING_500KBPS = 1,
};

// BLE5 frequency channel base values (taken from SmartRF)
enum
{
    RAD_BLE5_BASE_FREQ = 2402, // frequency channel 37 (channel offset = 0)
    RAD_BLE5_BASE_CH = 0x66, // id channel 37 (channel offset = 0)
    RAD_BLE5_BASE_WHITE_INIT = 0x40, // whitening initial value for channel 0
};

// ********************************
// Transmission and reception
// ********************************

// Data rates
typedef enum
{
    RAD_DATA_RATE_2MBPS = 0,
    RAD_DATA_RATE_1MBPS,
    RAD_DATA_RATE_500KBPS,
    RAD_DATA_RATE_125KBPS,
    RFC_DATA_RATES_NUM
} rad_data_rate_t;

// Transmission power codes (taken from SmartRF)
typedef enum
{
    RAD_TX_POW_PLUS_5dBm = 0x9330,
    RAD_TX_POW_PLUS_4dBm = 0x9324,
    RAD_TX_POW_PLUS_3dBm = 0x5A1C,
    RAD_TX_POW_PLUS_2dBm = 0x4E18,
    RAD_TX_POW_PLUS_1dBm = 0x4214,
    RAD_TX_POW_0dBm = 0x3161,
    RAD_TX_POW_MINUS_3dBm = 0x2558,
    RAD_TX_POW_MINUS_6dBm = 0x1D52,
    RAD_TX_POW_MINUS_9dBm = 0x194E,
    RAD_TX_POW_MINUS_12dBm = 0x144B,
    RAD_TX_POW_MINUS_15dBm = 0x0CCB,
    RAD_TX_POW_MINUS_18dBm = 0x0CC9,
    RAD_TX_POW_MINUS_21dBm = 0x0CC7,
    RAD_TX_POW_NUM = 13 // !!! TODO: keep this value updated
} rad_tx_pow_t;

// Max channel ID
#define RAD_FREQ_CH_NUM                 40

// Max packet payload length
#define RAD_MAX_PAYLOAD_LEN             254

// Reception buffer
#define RAD_RX_BUF_LEN                  512 // xxx
#define RAD_RX_BUF_PAYLOAD_LEN_IDX      1
#define RAD_RX_BUF_PAYLOAD_OFFSET       3

// Default radio configuration
#define RAD_DEFAULT_DATA_RATE           RAD_DATA_RATE_1MBPS
#define RAD_DEFAULT_TX_POW              RAD_TX_POW_0dBm
#define RAD_DEFAULT_FREQ_CH             17

// Reception error codes
typedef enum
{
    RAD_RX_ERR_NONE = 0,
    RAD_RX_ERR_CRC,
    RAD_RX_ERR_TIMEOUT,
    RAD_RX_ERR_OTHER,
} rad_rx_err_t;

// Packet transmission parameters
typedef struct
{
    bool delayed_start;
    uint32_t start_time;

    size_t payload_len;
    uint8_t* payload_p;
} rad_tx_param_t;

// Packet reception parameters
typedef struct
{
    bool delayed_start;
    uint32_t start_time;
    uint32_t timeout_usec;

    size_t dest_buf_len;
    uint8_t *dest_buf;

    size_t payload_len;
    uint32_t timestamp;
    int8_t rssi_dBm;

    rad_rx_err_t error;
} rad_rx_param_t;

// ********************************
// Error handling
// ********************************
// Error codes
enum
{
    RAD_ERR_NONE = 0,
    RAD_ERR_START_UP_FAILED,
    RAD_ERR_SHUTDOWN_FAILED,
    RAD_ERR_RADIO_OP_FAILED,
};


// ********************************
// State machine and control structure
// ********************************

// FSM states
typedef enum
{
    RAD_S_IDLE = 0,

    RAD_S_WAIT_RFC_BOOT,
    RAD_S_WAIT_RFC_CONFIG_SEQUENCE,
    RAD_S_WAIT_RFC_SYNC_STOP_RAT,

    RAD_S_WAIT_PACKET_TX,
    RAD_S_WAIT_PACKET_RX,

    RAD_S_WAIT_ERR_CLEARED,

    RAD_STATE_NUM
} rad_state_t;

typedef enum
{
    RAD_F_RFC_CONFIGURED = 0x01,
} rad_flags_t;

// Control structure (module's state)
typedef struct
{
    rad_state_t state;
    rad_flags_t flags;

    rad_rx_param_t* rx_param_p;

    uint8_t err_code;
} rad_control_t;

// ********************************
// Public functions
// ********************************

void Rad_Init();

void Rad_Process();

uint8_t Rad_Get_FSM_State();

bool Rad_Turn_On_Radio();

bool Rad_Turn_Off_Radio();

bool Rad_Radio_Is_On();

bool Rad_Set_Data_Rate(rad_data_rate_t data_rate);

bool Rad_Set_Tx_Power(rad_tx_pow_t tx_power);

bool Rad_Set_Freq_Channel(uint8_t channel_num);

bool Rad_Transmit_Packet(rad_tx_param_t* tx_param);

bool Rad_Receive_Packet(rad_rx_param_t* rx_param);

bool Rad_Ready();

bool Rad_Error_Occurred();

uint8_t Rad_Get_Err_Code();

#endif /* RADIO_H_ */
