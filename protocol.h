/*
 * protocol.h
 *
 *  Created on: Nov 1, 2018
 *      Author: alvaro
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include "timing.h"
#include "rf_core.h"
#include "power_management.h"

// Number of sensor nodes
#define PTC_SENSOR_NODES_NUM        10

// Frame duration
#define PTC_FRAME_TIME_SEC          (1)
#define PTC_BEAC_SLOT_TIME_MSEC     (20)
#define PTC_DATA_SLOT_TIME_MSEC     (80)

#define PTC_RTC_FRAME_TIME          (TM_RTC_TICKS_PER_SEC * PTC_FRAME_TIME_SEC)
#define PTC_RTC_BEAC_SLOT_TIME      (TM_RTC_TICKS_PER_MSEC * PTC_BEAC_SLOT_TIME_MSEC)
#define PTC_RTC_DATA_SLOT_TIME      (TM_RTC_TICKS_PER_MSEC * PTC_DATA_SLOT_TIME_MSEC)
#define PTC_RTC_GUARD_TIME          (TM_RTC_TICKS_PER_MSEC * PTC_GUARD_TIME_USEC)

#define PTC_RTC_MCU_WAKEUP_TIME     (PMA_WAKEUP_TIME_USEC / TM_RTC_USEC_PER_TICK)
#define PTC_RTC_RADIO_WAKEUP_TIME   (TM_RTC_TICKS_PER_MSEC * RFC_WAKEUP_TIME_MSEC)
#define PTC_RTC_TOTAL_WAKEUP_TIME   (PTC_RTC_MCU_WAKEUP_TIME + PTC_RTC_RADIO_WAKEUP_TIME)

// Size of reception-transmission buffer
#define PTC_RXTX_BUF_LEN            256

// Number of random seeds
#define PTC_RAND_SEEDS_NUM          4

// Sink node device id and macro to evaluate if the device is the sink node or a sensor node
#define PTC_SINK_NODE_DEV_ID        0
#define Ptc_Dev_Is_Sink_Node()      (ptc.dev_id == PTC_SINK_NODE_DEV_ID)
#define Ptc_Dev_Is_Sensor_Node()    (ptc.dev_id != PTC_SINK_NODE_DEV_ID)

// FSM states
typedef enum
{
    // Common states for both device roles
    PTC_S_IDLE = 0,
    PTC_S_WAIT_RF_CORE_INIT,
    PTC_S_WAIT_RF_CORE_WAKEUP,

    PTC_S_WAIT_START_OF_FRAME = 0x10,
    PTC_S_SCHEDULE_BEACON_TX,
    PTC_S_WAIT_START_OF_SLOT,

    // Initialization states for 'sink' role
    PTC_S_SCHEDULE_COMM = 0X20,

    // Initialization states 'sensor' role
    PTC_S_WAIT_FIRST_BEACON = 0x30,

    PTC_S_WAIT_TIMEOUT = 0xF0,
} ptc_state_t;

// Structure to hold the state of the protocol module
typedef struct
{
    uint8_t flags;
    uint8_t state, next_state;
    uint32_t absent_nodes;

    uint8_t dev_id;
    uint32_t ble_access_l, ble_access_h;

    uint16_t random_seeds[PTC_RAND_SEEDS_NUM];
    uint32_t start_of_next_frame;

    uint16_t tx_power;
    uint8_t phy_mode;
    uint8_t channel;

    rfc_tx_param_t tx_param;
    rfc_rx_result_t rx_result;

    uint8_t tx_buf[PTC_RXTX_BUF_LEN];
    uint8_t rx_buf[PTC_RXTX_BUF_LEN];
} ptc_control_t;

void Ptc_Init();

void Ptc_Process_Sink_Init();

void Ptc_Process_Sensor_Init();

void Ptc_Process_Sink_Normal_Op();

void Ptc_Process_Sensor_Normal_Op();

void (*Ptc_Process)();

uint8_t Ptc_Get_FSM_State();

void Ptc_Handle_Error();

#endif /* PROTOCOL_H_ */
