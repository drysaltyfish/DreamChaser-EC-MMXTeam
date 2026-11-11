#ifndef UART_CONNECTION_H
#define UART_CONNECTION_H

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app/app_config.h"
#include "utils/byte_ring.h"

/**
 * Byte stream endpoint for one protocol link.
 *
 * RX uses HAL_UARTEx_ReceiveToIdle_DMA in circular mode. Because the DMA never
 * stops in circular mode, the HAL event length cannot be trusted after a
 * wrap; this module tracks the DMA write position itself from the remaining
 * counter and moves new bytes into a software ring. The owning task is
 * notified by a task notification from the UART ISR.
 *
 * Several instances may coexist. The HAL callbacks are global, so each
 * instance is registered by its UART handle and looked up on every event.
 */
typedef struct {
    UART_HandleTypeDef* huart;
    uint8_t dma_buffer[APP_LINK_DMA_RX_SIZE];
    uint8_t ring_storage[APP_LINK_RX_RING_SIZE];
    ByteRing ring;
    uint16_t dma_position;
    TaskHandle_t volatile notify_task;
    volatile bool restart_pending;
    volatile uint32_t overrun_count;
    volatile uint32_t error_count;
    volatile bool started;
} UartConnection;

/** Binds one UART handle to one connection object. Call before any task starts. */
void UartConnection_Init(UartConnection* connection, UART_HandleTypeDef* huart);

/** Starts circular DMA reception. Call once from the owning task. */
bool UartConnection_Start(UartConnection* connection);

void UartConnection_SetNotifyTask(UartConnection* connection, TaskHandle_t task);

/** Pops received bytes. Returns the number of bytes copied. */
uint32_t UartConnection_Read(UartConnection* connection, uint8_t* destination, uint32_t capacity);

/** Blocking transmit; one task per connection may call it. */
bool UartConnection_Transmit(UartConnection* connection, const uint8_t* data, uint16_t length, uint32_t timeout_ms);

/** Restarts reception after a UART error. Call from the owning task. */
void UartConnection_RestartIfNeeded(UartConnection* connection);

#endif /* UART_CONNECTION_H */