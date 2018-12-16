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

// ********************************
// Configuration (see configuration.h)
// ********************************
// Debug signal; transition indicates the start of a frame
#if (CFG_DEBUG_START_OF_FRAME_OUT == CFG_SETTING_ENABLED)
#define PTC_START_OF_FRAME_OUT
#endif

// Print additional information
#if (CFG_DEBUG_VERBOSE == CFG_SETTING_ENABLED)
#define PTC_VERBOSE
#endif

// ********************************
// Radio and protocol
// ********************************
// Default radio parameters
#define PTC_DEFAULT_TX_POW          RFC_TX_POW_0dBm
#define PTC_DEFAULT_PHY_MODE        RFC_PHY_MODE_125KBPS
#define PTC_DEFAULT_CHANNEL         37

// Maximum number of consecutive errors
#define PTC_MAX_ERR_NUM             3

// Macro used to add field pay load (increase readability); see 'Ptc_Add_Field_To_Payload()' and 'Ptc_Get_Field_From_Payload()'
#define Ptc_Payload_Field(f)        &f, sizeof(f)

// Number of sensor nodes in the network
#define PTC_SENSOR_NODE_NUM         CFG_SENSOR_NODE_NUM

// Size of reception-transmission buffer
#define PTC_RXTX_BUF_LEN            256

// Number of random seeds
#define PTC_RAND_SEEDS_NUM          4

// Sink node device id and macro to evaluate if the device is the sink node or a sensor node
#define PTC_SINK_NODE_DEV_ID        0
#define Ptc_Dev_Is_Sink_Node()      (ptc.dev_id == PTC_SINK_NODE_DEV_ID)
#define Ptc_Dev_Is_Sensor_Node()    (ptc.dev_id != PTC_SINK_NODE_DEV_ID)

// ********************************
// Timing
// ********************************
// Reception timeout
#define PTC_RX_TIMEOUT_USEC         1300
#define PTC_OFFSET_RX_TOUT_USEC     200 // this value compensate for measured offset

// Start of transmission offset
#define PTC_RAT_TX_START_OFFSET     (-647) // value measured ~160 microseconds
// Start of reception offset; value measured ~200 microseconds; start calculated to center the reception time around the start of frame/slot
#define PTC_RAT_RX_START_OFFSET     (-800 - ((PTC_RX_TIMEOUT_USEC/2)*RFC_RAT_TICKS_PER_USEC))

// Frame duration
#define PTC_FRAME_TIME_SEC          1
#define PTC_RTC_FRAME_TIME          (TM_RTC_TICKS_PER_SEC * PTC_FRAME_TIME_SEC)
//#define PTC_RTC_FRAME_TIME          (TM_RTC_TICKS_PER_SEC * PTC_FRAME_TIME_SEC)/8

// Slot duration
#define PTC_RTC_SLOT_NUM            (PTC_SENSOR_NODE_NUM + 1)
#define PTC_RTC_SLOT_TIME           (PTC_RTC_FRAME_TIME/PTC_RTC_SLOT_NUM)

// Number of RTC ticks required to wake up
#define PTC_RTC_MCU_WAKEUP_TIME     (PMA_WAKEUP_TIME_USEC / TM_RTC_USEC_PER_TICK)
#define PTC_RTC_RADIO_WAKEUP_TIME   (TM_RTC_TICKS_PER_MSEC * RFC_WAKEUP_TIME_MSEC)
#define PTC_RTC_TOTAL_WAKEUP_TIME   (PTC_RTC_MCU_WAKEUP_TIME + PTC_RTC_RADIO_WAKEUP_TIME)

// ********************************
// Reliability test
// ********************************
//#define PTC_SUBSLOT_NUM             8
#define PTC_SUBSLOT_NUM             2 // xxx
#define PTC_RTC_SUBSLOT_TIME        (PTC_RTC_SLOT_TIME/PTC_SUBSLOT_NUM) // ~30 ms
#define PTC_TEST_TOUT_MSEC          20

typedef struct
{
    uint8_t i, j, k; // indexes of the PHY mode, TX power, and channel arrays
    uint8_t* phy_mode;
    rfc_tx_power_t* tx_power;
    uint8_t* channel;
} ptc_test_t;

// ********************************
// Control structure and FSM
// ********************************
// Flags
#define PTC_F_IN_SYNC               0x01
#define PTC_F_BEACON_RXED           0x02

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

    // Reliability test states
    PTC_S_WAIT_START_OF_SUBSLOT = 0x30,
    PTC_S_SCHEDULE_SUBSLOT_RADIO_OP,
    PTC_S_WAIT_TEST_PKT_RECEPTION,

    // Debug states
    PTC_S_WAIT_TIMEOUT = 0xF0,
} ptc_state_t;

// Structure to hold the state of the protocol module
typedef struct
{
    uint8_t flags;
    uint8_t state, next_state;
    uint32_t absent_nodes;
    uint8_t err_count;

    uint8_t dev_id;
    uint32_t ble_access_l, ble_access_h;

    uint16_t random_seeds[PTC_RAND_SEEDS_NUM];
    uint32_t start_of_next_frame;
    uint32_t start_of_next_slot;
    uint32_t start_of_next_subslot;
    uint8_t slot_count;
    uint8_t subslot_count;

    uint16_t tx_power;
    uint8_t phy_mode;
    uint8_t channel;

    rfc_tx_param_t tx_param;
    rfc_rx_result_t rx_result;

    uint8_t tx_buf[PTC_RXTX_BUF_LEN];
    uint8_t rx_buf[PTC_RXTX_BUF_LEN];

    ptc_test_t* test;
} ptc_control_t;

void Ptc_Init();

void (*Ptc_Process)();

uint8_t Ptc_Get_FSM_State();

void Ptc_Handle_Error();

#endif /* PROTOCOL_H_ */
