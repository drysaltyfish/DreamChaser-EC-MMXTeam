#include "app/tx_queue.h"

#include <stddef.h>

#include "FreeRTOS.h"
#include "queue.h"

#include "app/app_config.h"
#include "uart_connection.h"
#include "protocol/protocol_codec.h"

static QueueHandle_t s_queue;
static StaticQueue_t s_queue_control;
static ProtoFrame s_queue_storage[APP_TX_QUEUE_DEPTH];
static volatile uint32_t s_dropped;

void TxQueue_Init(void) {
    s_queue = xQueueCreateStatic(APP_TX_QUEUE_DEPTH, sizeof(ProtoFrame), (uint8_t*)s_queue_storage, &s_queue_control);
    s_dropped = 0u;
}

bool TxQueue_Send(const ProtoFrame* frame) {
    if (s_queue == NULL || frame == NULL) {
        return false;
    }

    if (xQueueSend(s_queue, frame, 0u) != pdTRUE) {
        s_dropped++;
        return false;
    }

    return true;
}

uint32_t TxQueue_GetDroppedCount(void) {
    return s_dropped;
}

void TxQueue_Task(void* argument) {
    (void)argument;

    ProtoFrame frame;
    uint8_t wire[PROTO_WIRE_MAX];

    for (;;) {
        if (xQueueReceive(s_queue, &frame, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        size_t length = Protocol_EncodeProtoFrame(&frame, wire, sizeof(wire));
        if (length == 0u) {
            continue;
        }

        if (!UartConnection_Transmit(wire, (uint16_t)length, APP_LINK_TX_TIMEOUT_MS)) {
            s_dropped++;
        }
    }
}
