/*
 * queue.c
 *
 *  Created on: Jul 9, 2018
 *      Author: alvaro
 */

#include "queue.h"

void Queue_Init(queue_t *qcp,
                uint8_t* buf,
                size_t size)
{
    qcp->buf = buf;
    qcp->size = size; // TODO size can be fixed for faster implementation
    qcp->head = qcp->tail = size - 1;
}

inline bool Queue_Empty(queue_t *qcp)
{
    return (qcp->tail == qcp->head);
}

inline bool Queue_Full(queue_t *qcp)
{
    return ((qcp->head - 1 ==  qcp->tail) ||
            ( (qcp->head == 0) && (qcp->tail == qcp->size - 1) ));
}

size_t Queue_Add(queue_t *qcp,
                 uint8_t* src,
                 size_t count)
{
    size_t n;

    for (n = count; n; n--, src++)
    {
        if (Queue_Full(qcp))
            break;
        else
        {
            qcp->buf[qcp->head] = *src;
            if (qcp->head == 0)
                qcp->head = qcp->size - 1;
            else
                qcp->head--;
        }
    }

    return (count - n);
}

size_t Queue_Remove(queue_t *qcp,
                    uint8_t* dest,
                    size_t count)
{
    size_t n;

    for (n = count; n; n--, dest++)
    {
        if (Queue_Empty(qcp))
            break;
        else
        {
            *dest = qcp->buf[qcp->tail];
            if (qcp->tail == 0)
                qcp->tail = qcp->size - 1;
            else
                qcp->tail--;
        }
    }

    return (count - n);
}

