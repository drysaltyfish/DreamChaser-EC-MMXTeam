#include "uart_connection.h"

#include <stddef.h>

#include "stm32f4xx_hal.h"

UartConnection g_uart_link;

static void UartConnection_NotifyTaskFromIsr(void) {
    TaskHandle_t task = g_uart_link.notify_task;
    if (task == NULL) {
        return;
    }

    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(task, &woken);
    portYIELD_FROM_ISR(woken);
}

static void UartConnection_PushRange(uint16_t begin, uint16_t end) {
    uint16_t length = (uint16_t)(end - begin);
    uint16_t accepted = ByteRing_PushBlock(&g_uart_link.ring, &g_uart_link.dma_buffer[begin], length);

    if (accepted < length) {
        g_uart_link.overrun_count += (uint32_t)(length - accepted);
    }
}

void UartConnection_Init(void) {
    g_uart_link.huart = APP_LINK_UART;
    ByteRing_Init(&g_uart_link.ring, g_uart_link.ring_storage, APP_LINK_RX_RING_SIZE);
    g_uart_link.dma_position = 0u;
    g_uart_link.notify_task = NULL;
    g_uart_link.restart_pending = false;
    g_uart_link.overrun_count = 0u;
    g_uart_link.error_count = 0u;
    g_uart_link.started = false;
}

static bool UartConnection_StartReceive(void) {
    /* Set the flag before the HAL can raise the first DMA event; otherwise the
     * callback would drop the first bytes of the new transfer. */
    g_uart_link.started = true;
    HAL_StatusTypeDef status =
        HAL_UARTEx_ReceiveToIdle_DMA(g_uart_link.huart, g_uart_link.dma_buffer, APP_LINK_DMA_RX_SIZE);

    if (status == HAL_OK) {
        return true;
    }

    g_uart_link.started = false;
    (void)HAL_UART_AbortReceive(g_uart_link.huart);

    g_uart_link.started = true;
    status = HAL_UARTEx_ReceiveToIdle_DMA(g_uart_link.huart, g_uart_link.dma_buffer, APP_LINK_DMA_RX_SIZE);
    g_uart_link.started = (status == HAL_OK);
    return status == HAL_OK;
}

bool UartConnection_Start(void) {
    g_uart_link.dma_position = 0u;
    bool started = UartConnection_StartReceive();
    g_uart_link.restart_pending = !started;
    return started;
}

void UartConnection_SetNotifyTask(TaskHandle_t task) {
    g_uart_link.notify_task = task;
}

uint32_t UartConnection_Read(uint8_t* destination, uint32_t capacity) {
    if (capacity > UINT16_MAX) {
        capacity = UINT16_MAX;
    }

    return ByteRing_PopBlock(&g_uart_link.ring, destination, (uint16_t)capacity);
}

bool UartConnection_Transmit(const uint8_t* data, uint16_t length, uint32_t timeout_ms) {
    if (data == NULL || length == 0u) {
        return false;
    }

    return HAL_UART_Transmit(g_uart_link.huart, (uint8_t*)data, length, timeout_ms) == HAL_OK;
}

void UartConnection_RestartIfNeeded(void) {
    if (!g_uart_link.restart_pending) {
        return;
    }

    g_uart_link.restart_pending = false;
    g_uart_link.started = false;
    (void)HAL_UART_AbortReceive(g_uart_link.huart);
    g_uart_link.dma_position = 0u;
    if (!UartConnection_StartReceive()) {
        /* Keep retrying from the link task instead of leaving the port dead. */
        g_uart_link.restart_pending = true;
    }
}

/* HAL callbacks: the HAL calls these from the UART ISR. */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size) {
    (void)size;

    if (huart != g_uart_link.huart || !g_uart_link.started) {
        return;
    }

    uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(huart->hdmarx);
    uint16_t position = (uint16_t)(APP_LINK_DMA_RX_SIZE - remaining);
    uint16_t previous = g_uart_link.dma_position;

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
        UartConnection_PushRange(previous, APP_LINK_DMA_RX_SIZE);
        if (position > 0u) {
            UartConnection_PushRange(0u, position);
        }
    } else if (position == previous) {
        return;
    } else if (position > previous) {
        UartConnection_PushRange(previous, position);
    } else {
        /* A half-transfer or idle event after the DMA wrapped. */
        UartConnection_PushRange(previous, APP_LINK_DMA_RX_SIZE);
        UartConnection_PushRange(0u, position);
    }

    g_uart_link.dma_position = position;
    UartConnection_NotifyTaskFromIsr();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
    if (huart != g_uart_link.huart) {
        return;
    }

    g_uart_link.error_count++;
    g_uart_link.restart_pending = true;
    UartConnection_NotifyTaskFromIsr();
}
