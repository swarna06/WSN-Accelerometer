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

// ********************************
// RF core API
// ********************************

// ********************************
// Transmission and reception
// ********************************

typedef struct
{

} rad_tx_param_t;

typedef struct
{

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

    uint8_t err_code;
} rad_control_t;

// ********************************
// Public functions
// ********************************

void Rad_Init();

void Rad_Process();

uint8_t Rad_Get_FSM_State();

bool Rad_Turn_On();

bool Rad_Turn_Off();

bool Rad_Is_On();

uint32_t Rad_Get_Time();

bool Rad_Set_Data_Rate();

bool Rad_Set_Tx_Power();

bool Rad_Set_Freq_Channel();

bool Rad_Transmit_Packet(rad_tx_param_t *tx_param);

bool Rad_Receive_Packet(rad_rx_param_t *rx_result);

bool Rad_Ready();

bool Rad_Error_Occurred();

uint8_t Rad_Get_Err_Code();

#endif /* RADIO_H_ */
