/*
 * coordinator.h
 *
 *  Created on: Nov 2, 2018
 *      Author: alvaro
 */

#ifndef COORDINATOR_H_
#define COORDINATOR_H_

#include <stdbool.h>
#include <stdlib.h>

// Coordinator FSM states
typedef enum
{
    CRD_S_WAIT_COMMAND = 0,
    CRD_S_WAIT_SYNC,
} crd_state_t;

// Coordinator module control structure
typedef struct
{
    int state;
    bool mode;
    bool sync;
    size_t per_sync_idx;
} crd_control_t;

void Crd_Init();

void Crd_Process();

#endif /* COORDINATOR_H_ */
