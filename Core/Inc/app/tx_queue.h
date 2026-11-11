#ifndef TX_QUEUE_H
#define TX_QUEUE_H

#include <stdbool.h>
#include <stdint.h>

#include "protocol/protocol_defs.h"

void TxQueue_Init(void);

/** Queues one frame for one link. Returns false when that queue is full. */
bool TxQueue_Send(proto_link_t link, const ProtoFrame* frame);

/** Task argument must point at the AppLink owned by this sender. */
void TxQueue_Task(void* argument);

uint32_t TxQueue_GetDroppedCount(proto_link_t link);

#endif /* TX_QUEUE_H */
