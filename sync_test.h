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

#define STS_SYNC_PERIOD_RTC     ((1<<16)/16) // 1 second
#define STS_WAKEUP_DELAY        (TM_RTC_TICKS_PER_MSEC * 2) // 2 milliseconds

#define STS_RADIO_OP_DELAY      (762 + 5) // 191.4 usec / 0.25 usec (+ 5 trimming)

enum
{
    STS_S_WAIT_WAKEUP_TIME = 0,

    STS_S_WAIT_RADIO_STARTUP,

    STS_S_WAIT_SET_RAT_OUTPUT,

    STS_S_WAIT_SET_RAT_CMP_VAL,

    STS_S_WAIT_PKT_TX,

    STS_S_WAIT_FIRST_PKT,


    STS_S_WAIT_RADIO_OFF,

    STS_S_DUMMY,

    STS_COMMON_STATES_NUM
};


typedef struct
{
    uint8_t dev_id;

    uint32_t sync_time;
    uint32_t wakeup_time;

    rad_tx_param_t tx_param;
    uint8_t tx_buf[RAD_MAX_PAYLOAD_LEN];

    uint8_t flags;
    uint8_t state;
} sts_control;

void Sts_Init();

void (*Sts_Process)();

#endif /* SYNC_TEST_H_ */
