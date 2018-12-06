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
#include "configuration.h"


// Macro used to add field field payload (increase readability)
// To be used with the 'Ptc_Add_Field_To_Payload' and 'Ptc_Get_Field_From_Payload' functions
#define Ptc_Payload_Field(f)        &f, sizeof(f)

// Debug signal; transition indicates the start of a frame
#if (CFG_DEBUG_START_OF_FRAME_OUT == CFG_SETTING_ENABLED)
#define PTC_START_OF_FRAME_OUT
#endif

// Number of sensor nodes in the network
#define PTC_SENSOR_NODE_NUM         CFG_SENSOR_NODE_NUM

// Reception timeout
#define PTC_RX_TIMEOUT_USEC         1000
#define PTC_OFFSET_RX_TOUT_USEC     200

// Start of transmission offset
#define PTC_RAT_TX_START_OFFSET     (-647) // value measured ~160 microseconds
// Start of reception offset
// value measured ~200 microseconds; start calculated to center the reception time around the start of frame/slot
#define PTC_RAT_RX_START_OFFSET     (-800 - ((PTC_RX_TIMEOUT_USEC/2)*RFC_RAT_TICKS_PER_USEC))

// Number of sensor nodes
#define PTC_SENSOR_NODES_NUM        10

// Frame duration
#define PTC_FRAME_TIME_SEC          (1)
#define PTC_BEAC_SLOT_TIME_MSEC     (20)
#define PTC_DATA_SLOT_TIME_MSEC     (80)

#define PTC_RTC_FRAME_TIME          (TM_RTC_TICKS_PER_SEC * PTC_FRAME_TIME_SEC)
//#define PTC_RTC_FRAME_TIME          (TM_RTC_TICKS_PER_SEC * PTC_FRAME_TIME_SEC)/8
#define PTC_RTC_SLOT_TIME           (PTC_RTC_FRAME_TIME/4)
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

    // Initialization states
    PTC_S_SCHEDULE_FIRST_BEACON_RX = 0x10,
    PTC_S_WAIT_FIRST_BEACON,

    // 'Steady-state' states
    PTC_S_WAIT_START_OF_FRAME = 0x20,
    PTC_S_SCHEDULE_BEACON_RADIO_OP,
    PTC_S_WAIT_START_OF_SLOT,
    PTC_S_SCHEDULE_SLOT_RADIO_OP,
    PTC_S_WAIT_PKT_RECEPTION,

    // Debug states
    PTC_S_WAIT_TIMEOUT = 0xF0,
} ptc_state_t;

// Flags
#define PTC_F_IN_SYNC               0x01

// Structure to hold the state of the protocol module
typedef struct
{
    uint8_t flags;
    uint8_t state, next_state;
    uint32_t absent_nodes;

    uint8_t dev_id, dev_index;
    uint32_t ble_access_l, ble_access_h;

    uint16_t random_seeds[PTC_RAND_SEEDS_NUM];
    uint32_t start_of_next_frame;
    uint32_t start_of_next_slot;

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

uint8_t Ptc_Get_FSM_State();

void Ptc_Handle_Error();

#endif /* PROTOCOL_H_ */
