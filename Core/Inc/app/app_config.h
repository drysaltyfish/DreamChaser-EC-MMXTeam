#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "main.h"

/*
 * Single place to change the physical link.
 *
 * To move the protocol to another UART, change the extern declaration and
 * APP_LINK_UART below; the DMA stream, GPIO and NVIC setup belong to CubeMX
 * (Fusai.ioc). Nothing else in the application names a UART instance.
 */
extern UART_HandleTypeDef huart5;
#define APP_LINK_UART (&huart5)

/* RX DMA buffer and software ring. The ring size must be a power of two. */
#define APP_LINK_DMA_RX_SIZE 256u
#define APP_LINK_RX_RING_SIZE 1024u

#if (APP_LINK_RX_RING_SIZE & (APP_LINK_RX_RING_SIZE - 1u)) != 0u
#error "APP_LINK_RX_RING_SIZE must be a power of two"
#endif

#define APP_LINK_TX_TIMEOUT_MS 50u

/* Task priorities (FreeRTOS numbering: larger runs first). */
#define APP_TASK_PRIORITY_LINK 5u
#define APP_TASK_PRIORITY_COMMAND 4u
#define APP_TASK_PRIORITY_GIMBAL 3u
#define APP_TASK_PRIORITY_CHASSIS 3u
#define APP_TASK_PRIORITY_TX 2u
#define APP_TASK_PRIORITY_TELEMETRY 1u

/* Task stacks, in words (4 bytes each). */
#define APP_TASK_STACK_LINK 320u
#define APP_TASK_STACK_COMMAND 384u
#define APP_TASK_STACK_GIMBAL 256u
#define APP_TASK_STACK_CHASSIS 256u
#define APP_TASK_STACK_TX 256u
#define APP_TASK_STACK_TELEMETRY 256u

/* Queue depths. */
#define APP_COMMAND_QUEUE_DEPTH 8u
#define APP_TX_QUEUE_DEPTH 8u

/* Control loop period and protocol timeouts. */
#define APP_CONTROL_PERIOD_MS 20u
#define APP_VELOCITY_TIMEOUT_MS 300u
#define APP_HEARTBEAT_TIMEOUT_MS 500u
#define APP_TELEMETRY_PERIOD_MS 1000u
#define APP_LINK_POLL_MS 10u
#define APP_COMMAND_WAIT_MS 100u

/* Gimbal model: 0.01 degree units. */
#define APP_GIMBAL_PAN_MIN (-18000)
#define APP_GIMBAL_PAN_MAX (18000)
#define APP_GIMBAL_TILT_MIN (-9000)
#define APP_GIMBAL_TILT_MAX (9000)
#define APP_GIMBAL_MAX_SPEED_LSB_PER_S 9000u
#define APP_GIMBAL_ARRIVAL_TOLERANCE_LSB 10

/* Chassis ramp, in normalized units per second. */
#define APP_CHASSIS_ACCEL_PER_S 2.5f
#define APP_CHASSIS_DECEL_PER_S 3.5f
#define APP_CHASSIS_ESTOP_DECEL_PER_S 8.0f
#define APP_CHASSIS_DEADZONE 0.02f

/* Set to 1 once MotorB/C/D and Vehicle_SetVelocity exist. */
#define APP_CHASSIS_DRIVE_ENABLED 0

/* Calibration flash: two 128 KiB sectors of the STM32F407VE (512 KiB total).
 * Sectors 6 and 7 occupy 0x08040000..0x0807FFFF, above the application image. */
#define APP_CALIBRATION_SECTOR_A FLASH_SECTOR_6
#define APP_CALIBRATION_SECTOR_B FLASH_SECTOR_7
#define APP_CALIBRATION_ADDRESS_A 0x08040000u
#define APP_CALIBRATION_ADDRESS_B 0x08060000u

#endif /* APP_CONFIG_H */
