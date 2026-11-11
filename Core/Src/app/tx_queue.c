#include "app/tx_queue.h"

#include <stddef.h>

#include "FreeRTOS.h"
#include "queue.h"

#include "app/app_config.h"
#include "app/app_links.h"
#include "protocol/protocol_codec.h"

typedef struct {
    QueueHandle_t queue;
    StaticQueue_t control;
    ProtoFrame storage[APP_TX_QUEUE_DEPTH];
    volatile uint32_t dropped;
} TxChannel;

static TxChannel s_channels[PROTO_LINK_COUNT];

void TxQueue_Init(void) {
    for (uint32_t i = 0u; i < (uint32_t)PROTO_LINK_COUNT; i++) {
        TxChannel* channel = &s_channels[i];
        channel->queue = xQueueCreateStatic(APP_TX_QUEUE_DEPTH, sizeof(ProtoFrame), (uint8_t*)channel->storage,
                                            &channel->control);
        channel->dropped = 0u;
    }
}

bool TxQueue_Send(proto_link_t link, const ProtoFrame* frame) {
    if ((uint32_t)link >= (uint32_t)PROTO_LINK_COUNT || frame == NULL) {
        return false;
    }

    TxChannel* channel = &s_channels[link];
    if (channel->queue == NULL) {
        return false;
    }

    if (xQueueSend(channel->queue, frame, 0u) != pdTRUE) {
        channel->dropped++;
        return false;
    }

    return true;
}

uint32_t TxQueue_GetDroppedCount(proto_link_t link) {
    if ((uint32_t)link >= (uint32_t)PROTO_LINK_COUNT) {
        return 0u;
    }
    return s_channels[link].dropped;
}

void TxQueue_Task(void* argument) {
    const AppLink* link = argument;
    if (link == NULL || link->uart == NULL) {
        vTaskDelete(NULL);
        return;
    }

    TxChannel* channel = &s_channels[link->id];
    ProtoFrame frame;
    uint8_t wire[PROTO_WIRE_MAX];

    for (;;) {
        if (xQueueReceive(channel->queue, &frame, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        size_t length = Protocol_EncodeProtoFrame(&frame, wire, sizeof(wire));
        if (length == 0u) {
            continue;
        }

        if (!UartConnection_Transmit(link->uart, wire, (uint16_t)length, APP_LINK_TX_TIMEOUT_MS)) {
            channel->dropped++;
        }
    }
}
