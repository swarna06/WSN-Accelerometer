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

bool Rad_Turn_On_Radio();

bool Rad_Turn_Off_Radio();

bool Rad_Radio_Is_On();

bool Rad_Set_Data_Rate();

bool Rad_Set_Tx_Power();

bool Rad_Set_Freq_Channel();

bool Rad_Transmit_Packet(rad_tx_param_t *tx_param);

bool Rad_Receive_Packet(rad_rx_param_t *rx_result);

bool Rad_Ready();

bool Rad_Error_Occurred();

uint8_t Rad_Get_Err_Code();

#endif /* RADIO_H_ */
