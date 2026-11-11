#include "uart_connection.h"

#include <stddef.h>

#include "stm32f4xx_hal.h"

/* One slot per physical protocol link; keep in sync with PROTO_LINK_COUNT. */
#define UART_CONNECTION_MAX 2u

static UartConnection* s_instances[UART_CONNECTION_MAX];

static UartConnection* UartConnection_Find(const UART_HandleTypeDef* huart) {
    for (uint32_t i = 0u; i < UART_CONNECTION_MAX; i++) {
        UartConnection* connection = s_instances[i];
        if (connection != NULL && connection->huart == huart) {
            return connection;
        }
    }
    return NULL;
}

static void UartConnection_Register(UartConnection* connection) {
    for (uint32_t i = 0u; i < UART_CONNECTION_MAX; i++) {
        if (s_instances[i] == connection) {
            return;
        }
        if (s_instances[i] == NULL) {
            s_instances[i] = connection;
            return;
        }
    }
}

static void UartConnection_NotifyTaskFromIsr(UartConnection* connection) {
    TaskHandle_t task = connection->notify_task;
    if (task == NULL) {
        return;
    }

    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(task, &woken);
    portYIELD_FROM_ISR(woken);
}

static void UartConnection_PushRange(UartConnection* connection, uint16_t begin, uint16_t end) {
    uint16_t length = (uint16_t)(end - begin);
    uint16_t accepted = ByteRing_PushBlock(&connection->ring, &connection->dma_buffer[begin], length);

    if (accepted < length) {
        connection->overrun_count += (uint32_t)(length - accepted);
    }
}

void UartConnection_Init(UartConnection* connection, UART_HandleTypeDef* huart) {
    if (connection == NULL || huart == NULL) {
        return;
    }

    *connection = (UartConnection){0};
    connection->huart = huart;
    ByteRing_Init(&connection->ring, connection->ring_storage, APP_LINK_RX_RING_SIZE);
    UartConnection_Register(connection);
}

static bool UartConnection_StartReceive(UartConnection* connection) {
    /* Set the flag before the HAL can raise the first DMA event; otherwise the
     * callback would drop the first bytes of the new transfer. */
    connection->started = true;
    HAL_StatusTypeDef status =
        HAL_UARTEx_ReceiveToIdle_DMA(connection->huart, connection->dma_buffer, APP_LINK_DMA_RX_SIZE);

    if (status == HAL_OK) {
        return true;
    }

    connection->started = false;
    (void)HAL_UART_AbortReceive(connection->huart);

    connection->started = true;
    status = HAL_UARTEx_ReceiveToIdle_DMA(connection->huart, connection->dma_buffer, APP_LINK_DMA_RX_SIZE);
    connection->started = (status == HAL_OK);
    return status == HAL_OK;
}

bool UartConnection_Start(UartConnection* connection) {
    if (connection == NULL || connection->huart == NULL) {
        return false;
    }

    connection->dma_position = 0u;
    bool started = UartConnection_StartReceive(connection);
    connection->restart_pending = !started;
    return started;
}

void UartConnection_SetNotifyTask(UartConnection* connection, TaskHandle_t task) {
    if (connection != NULL) {
        connection->notify_task = task;
    }
}

uint32_t UartConnection_Read(UartConnection* connection, uint8_t* destination, uint32_t capacity) {
    if (connection == NULL || destination == NULL) {
        return 0u;
    }
    if (capacity > UINT16_MAX) {
        capacity = UINT16_MAX;
    }

    return ByteRing_PopBlock(&connection->ring, destination, (uint16_t)capacity);
}

bool UartConnection_Transmit(UartConnection* connection, const uint8_t* data, uint16_t length,
                             uint32_t timeout_ms) {
    if (connection == NULL || connection->huart == NULL || data == NULL || length == 0u) {
        return false;
    }

    return HAL_UART_Transmit(connection->huart, (uint8_t*)data, length, timeout_ms) == HAL_OK;
}

void UartConnection_RestartIfNeeded(UartConnection* connection) {
    if (connection == NULL || !connection->restart_pending) {
        return;
    }

    connection->restart_pending = false;
    connection->started = false;
    (void)HAL_UART_AbortReceive(connection->huart);
    connection->dma_position = 0u;
    if (!UartConnection_StartReceive(connection)) {
        /* Keep retrying from the owning task instead of leaving the port dead. */
        connection->restart_pending = true;
    }
}

/* HAL callbacks: the HAL calls these from the UART ISR. */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size) {
    (void)size;

    UartConnection* connection = UartConnection_Find(huart);
    if (connection == NULL || !connection->started) {
        return;
    }

    uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(huart->hdmarx);
    uint16_t position = (uint16_t)(APP_LINK_DMA_RX_SIZE - remaining);
    uint16_t previous = connection->dma_position;

    if (position > APP_LINK_DMA_RX_SIZE) {
        return;
    }
    if (position == APP_LINK_DMA_RX_SIZE) {
        /* The counter has not reloaded yet. Treat it as the wrapped position;
         * otherwise the TC branch would push the whole buffer twice. */
        position = 0u;
    }

    if (huart->RxEventType == HAL_UART_RXEVENT_TC) {
        /* The DMA reached the end of the buffer and reloaded its counter.
         * Everything from the previous position to the end is new data. */
        UartConnection_PushRange(connection, previous, APP_LINK_DMA_RX_SIZE);
        if (position > 0u) {
            UartConnection_PushRange(connection, 0u, position);
        }
    } else if (position == previous) {
        return;
    } else if (position > previous) {
        UartConnection_PushRange(connection, previous, position);
    } else {
        /* A half-transfer or idle event after the DMA wrapped. */
        UartConnection_PushRange(connection, previous, APP_LINK_DMA_RX_SIZE);
        UartConnection_PushRange(connection, 0u, position);
    }

    connection->dma_position = position;
    UartConnection_NotifyTaskFromIsr(connection);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    UartConnection* connection = UartConnection_Find(huart);
    if (connection == NULL) {
        return;
    }

    connection->error_count++;
    connection->restart_pending = true;
    UartConnection_NotifyTaskFromIsr(connection);
}
