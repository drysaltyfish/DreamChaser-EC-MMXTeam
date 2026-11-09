#ifndef UART_CONNECTION_H
#define UART_CONNECTION_H

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app/app_config.h"
#include "utils/byte_ring.h"

/**
 * Byte stream endpoint for the protocol link.
 *
 * RX uses HAL_UARTEx_ReceiveToIdle_DMA in circular mode. Because the DMA never
 * stops in circular mode, the HAL event length cannot be trusted after a
 * wrap; this module tracks the DMA write position itself from the remaining
 * counter and moves new bytes into a software ring. The application task is
 * notified by a task notification from the UART ISR.
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

extern UartConnection g_uart_link;

void UartConnection_Init(void);

/** Starts circular DMA reception. Call once from a task, after the UART init. */
bool UartConnection_Start(void);

void UartConnection_SetNotifyTask(TaskHandle_t task);

/** Pops received bytes. Returns the number of bytes copied. */
uint32_t UartConnection_Read(uint8_t* destination, uint32_t capacity);

/** Blocking transmit; must be called from one task only. */
bool UartConnection_Transmit(const uint8_t* data, uint16_t length, uint32_t timeout_ms);

/** Restarts reception after a UART error. Call from the link task. */
void UartConnection_RestartIfNeeded(void);

#endif /* UART_CONNECTION_H */
