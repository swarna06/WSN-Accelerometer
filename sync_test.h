/*
 * sync_test.h
 *
 *  Created on: Mar 23, 2019
 *      Author: alvaro
 */

#ifndef SYNC_TEST_H_
#define SYNC_TEST_H_

#include <stdint.h>

#include "radio.h"

#define STS_DATA_RATE                   RAD_DATA_RATE_125KBPS

#define STS_SYNC_PER_SEC                1
#define STS_SYNC_PER_DIV                1

#define STS_SYNC_PERIOD_RTC             (((1<<16)/STS_SYNC_PER_DIV)*STS_SYNC_PER_SEC) // 1 second
#define STS_WAKEUP_DELAY                (TM_RTC_TICKS_PER_MSEC * 2) // 2 milliseconds

#define STS_SYNC_PERIOD_RAT             (((4000000)/STS_SYNC_PER_DIV)*STS_SYNC_PER_SEC) // 1 second

#define STS_RADIO_OP_DELAY              (762 + 5) // 191.4 usec / 0.25 usec (+ 5 trimming)

#define STS_RX_START_DELAY              (RAD_RAT_TICKS_PER_USEC * 40) // 40 usec
#define STS_GUARD_TIME_USEC             50 // max synchronization error
#define STS_PREAMBLE_TIME_USEC          (80 + 256 + 16 + 24 + 3*8*8 + 24*8 + 3*8) // coded PHY
#define STS_GUART_TIME_RAT_TICKS        (RAD_RAT_TICKS_PER_USEC * STS_GUARD_TIME_USEC)
#define STS_PREAMBLE_TIME_RAT_TICKS     (RAD_RAT_TICKS_PER_USEC * STS_PREAMBLE_TIME_USEC)


// No moving average or low sample number provide better results
#define STS_SAMP_NUM            1

enum
{
    STS_S_WAIT_WAKEUP_TIME = 0,

    STS_S_WAIT_RADIO_STARTUP,

    STS_S_WAIT_SET_RAT_OUTPUT,

    STS_S_WAIT_SET_RAT_CMP_VAL,

    STS_S_WAIT_PKT_TX,

    STS_S_WAIT_1ST_PKT,

    STS_S_WAIT_PKT_RX,

    STS_S_WAIT_RADIO_OFF,

    STS_S_DUMMY,

    STS_COMMON_STATES_NUM
};

enum
{
    STS_F_1ST_PKT_RXED = 0x01,
    STS_F_MASTER_INITIALIZED = 0x02,
};


typedef struct
{
    uint8_t dev_id;

    uint32_t rtc_sync_time;
    uint32_t rtc_wakeup_time;
    uint32_t rat_sync_time;

    rad_tx_param_t tx_param;
    rad_rx_param_t rx_param;
    uint8_t txrx_buf[RAD_MAX_PAYLOAD_LEN];

    int32_t missed_pkt_cnt;
    int32_t sync_err_samples[STS_SAMP_NUM];
    size_t samp_idx;
    size_t samp_cnt;

    uint8_t flags;
    uint8_t state;
} sts_control;

void Sts_Init();

void (*Sts_Process)();

uint8_t Sts_Get_FSM_State();

#endif /* SYNC_TEST_H_ */
