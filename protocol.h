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

// Number of sensor nodes
#define PTC_SENSOR_NODES_NUM        10

// Frame duration
#define PTC_FRAME_TIME_USEC         (1000000)
#define PTC_BEAC_SLOT_TIME_USEC     (1000*20)
#define PTC_DATA_SLOT_TIME_USEC     (1000*80)
#define PTC_GUARD_TIME_USEC         (100)

#define PTC_RTC_FRAME_TIME          (TM_RTC_TICKS_PER_MSEC * PTC_FRAME_TIME_USEC)
#define PTC_RTC_BEAC_SLOT_TIME      (TM_RTC_TICKS_PER_MSEC * PTC_BEAC_SLOT_TIME_USEC)
#define PTC_RTC_DATA_SLOT_TIME      (TM_RTC_TICKS_PER_MSEC * PTC_DATA_SLOT_TIME_USEC)
#define PTC_RTC_GUARD_TIME          (TM_RTC_TICKS_PER_MSEC * PTC_GUARD_TIME_USEC)

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
    PTC_S_IDLE = 0,
    PTC_S_WAIT_RF_CORE_INIT,
    PTC_S_WAIT_RFC_RAT_SYNC,

    // Common states for both device roles
    PTC_S_WAIT_START_OF_FRAME = 0x10,
    PTC_S_WAIT_START_OF_SLOT,

    PTC_S_WAIT_FIRST_BEACON = 0x20,

    PTC_S_WAIT_INIT_PERIOD = 0x30,
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

    uint16_t tx_power;
    uint8_t phy_mode;
    uint8_t channel;

    rfc_tx_param_t tx_param;
    rfc_rx_result_t rx_result;

    uint8_t tx_buf[PTC_RXTX_BUF_LEN];
    uint8_t rx_buf[PTC_RXTX_BUF_LEN];
} ptc_control_t;

void Ptc_Init();

void Ptc_Process();

#endif /* PROTOCOL_H_ */
