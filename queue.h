/*
 * queue.h
 *
 *  Created on: Jul 9, 2018
 *      Author: alvaro
 */

#ifndef QUEUE_H_
#define QUEUE_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct
{
    uint8_t* buf;
    size_t size;
    size_t head, tail;
} queue_t;

void Queue_Init(queue_t *qcp,
                uint8_t* buf,
                size_t size);

bool Queue_Empty(queue_t *qcp);

bool Queue_Full(queue_t *qcp);

size_t Queue_Add(queue_t *qcp,
                 uint8_t* src,
                 size_t count);

size_t Queue_Remove(queue_t *qcp,
                    uint8_t* dest,
                    size_t count);

#endif /* QUEUE_H_ */
