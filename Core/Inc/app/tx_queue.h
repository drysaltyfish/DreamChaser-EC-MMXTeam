#ifndef TX_QUEUE_H
#define TX_QUEUE_H

#include <stdbool.h>
#include <stdint.h>

#include "protocol/protocol_defs.h"

void TxQueue_Init(void);

/** Queues one frame. Returns false when the queue is full. */
bool TxQueue_Send(const ProtoFrame* frame);

void TxQueue_Task(void* argument);

uint32_t TxQueue_GetDroppedCount(void);

#endif /* TX_QUEUE_H */
