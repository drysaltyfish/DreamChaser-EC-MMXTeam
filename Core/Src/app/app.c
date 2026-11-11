#include "app/app.h"

#include <stdbool.h>
#include <stddef.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "app/app_config.h"
#include "app/app_links.h"
#include "app/chassis_port.h"
#include "app/command_router.h"
#include "app/gimbal_port.h"
#include "app/telemetry.h"
#include "app/tx_queue.h"
#include "motion/motion_control.h"
#include "protocol/protocol_decoder.h"
#include "uart_connection.h"

typedef struct {
    proto_link_t link;
    ProtoFrame frame;
} LinkFrame;

static UartConnection s_pi_uart;
static UartConnection s_bt_uart;

AppLink g_app_links[PROTO_LINK_COUNT] = {
    [PROTO_LINK_PI] = {.id = PROTO_LINK_PI, .uart = &s_pi_uart},
    [PROTO_LINK_BT] = {.id = PROTO_LINK_BT, .uart = &s_bt_uart},
};

static StaticQueue_t s_command_queue_control;
static LinkFrame s_command_storage[APP_COMMAND_QUEUE_DEPTH];
static QueueHandle_t s_command_queue;

static StaticTask_t s_link_tcb[PROTO_LINK_COUNT];
static StaticTask_t s_command_tcb;
static StaticTask_t s_gimbal_tcb;
static StaticTask_t s_chassis_tcb;
static StaticTask_t s_tx_tcb[PROTO_LINK_COUNT];
static StaticTask_t s_telemetry_tcb;

static StackType_t s_link_stack[PROTO_LINK_COUNT][APP_TASK_STACK_LINK];
static StackType_t s_command_stack[APP_TASK_STACK_COMMAND];
static StackType_t s_gimbal_stack[APP_TASK_STACK_GIMBAL];
static StackType_t s_chassis_stack[APP_TASK_STACK_CHASSIS];
static StackType_t s_tx_stack[PROTO_LINK_COUNT][APP_TASK_STACK_TX];
static StackType_t s_telemetry_stack[APP_TASK_STACK_TELEMETRY];

static uint32_t s_dropped_commands;

/* A client that connects after boot has missed the startup READY. Re-announce
 * once on its first heartbeat so the handshake works without a reset. */
static bool s_ready_on_heartbeat[PROTO_LINK_COUNT];

static void DrainCommandQueue(void) {
    LinkFrame discarded;
    while (xQueueReceive(s_command_queue, &discarded, 0u) == pdTRUE) {
        /* Drop commands that were queued before the emergency stop. */
    }
}

static void HandleHeartbeat(proto_link_t link, const ProtoFrame* frame, uint32_t now) {
    if (frame->data_len != 0u) {
        CommandRouter_SendError(link, PROTO_ERROR_FRAME, frame->cmd);
        return;
    }
    MotionControl_NoteHeartbeat(link, now);
    if (!s_ready_on_heartbeat[link]) {
        s_ready_on_heartbeat[link] = true;
        Telemetry_SendReady(link);
    }
}

static void HandleDecodedFrame(proto_link_t link, const ProtoFrame* frame) {
    /* Heartbeats bypass the command queue: dropping one because a burst filled
     * the queue would look like a link failure. */
    if (frame->cmd == PROTO_CMD_HEARTBEAT) {
        HandleHeartbeat(link, frame, HAL_GetTick());
        return;
    }

    /* Treat any SET_MODE frame whose first data byte is 3 as an emergency
     * stop, even if the length is malformed. A corrupt length must not be able
     * to suppress a stop request from either link. */
    bool emergency = (frame->cmd == PROTO_CMD_SET_MODE) && (frame->data_len >= 1u) &&
                     (frame->data[0] == PROTO_MODE_EMERGENCY_STOP);
    if (emergency) {
        MotionControl_EmergencyStop();
        DrainCommandQueue();
        Telemetry_SendStatuses(HAL_GetTick());
        return;
    }

    LinkFrame envelope = {.link = link, .frame = *frame};
    if (xQueueSend(s_command_queue, &envelope, 0u) != pdTRUE) {
        s_dropped_commands++;
    }
}

static void LinkTask(void* argument) {
    const AppLink* link = argument;
    if (link == NULL || link->uart == NULL) {
        vTaskDelete(NULL);
        return;
    }

    UartConnection_SetNotifyTask(link->uart, xTaskGetCurrentTaskHandle());
    (void)UartConnection_Start(link->uart);

    ProtoDecoder decoder;
    ProtocolDecoder_Init(&decoder);
    uint32_t format_errors = decoder.format_errors;

    for (;;) {
        uint32_t notification = 0u;
        (void)xTaskNotifyWait(0u, UINT32_MAX, &notification, pdMS_TO_TICKS(APP_LINK_POLL_MS));

        UartConnection_RestartIfNeeded(link->uart);

        uint8_t buffer[64];
        uint32_t available;
        while ((available = UartConnection_Read(link->uart, buffer, sizeof(buffer))) > 0u) {
            for (uint32_t i = 0u; i < available; i++) {
                if (!ProtocolDecoder_Feed(&decoder, buffer[i])) {
                    continue;
                }

                ProtoFrame frame;
                if (ProtocolDecoder_TakeFrame(&decoder, &frame)) {
                    HandleDecodedFrame(link->id, &frame);
                }
            }
        }

        if (ProtocolDecoder_TakeCrcAlarm(&decoder)) {
            CommandRouter_SendError(link->id, PROTO_ERROR_CRC, 0u);
        }
        if (decoder.format_errors != format_errors) {
            format_errors = decoder.format_errors;
            CommandRouter_SendError(link->id, PROTO_ERROR_FRAME, 0u);
        }
        if (s_dropped_commands > 0u) {
            /* A full queue means a burst outran the command task. The count is
             * visible through the status error counter; do not emit one ERROR
             * per dropped frame and flood the link. */
            MotionControl_NoteError();
            s_dropped_commands = 0u;
        }
    }
}

static void CommandTask(void* argument) {
    (void)argument;

    for (;;) {
        LinkFrame envelope;
        if (xQueueReceive(s_command_queue, &envelope, pdMS_TO_TICKS(APP_CONTROL_PERIOD_MS)) == pdTRUE) {
            CommandRouter_HandleFrame(envelope.link, &envelope.frame, HAL_GetTick());

            /* Drain a burst before the next control period so a stream of
             * heartbeats and setpoints cannot back up the link. */
            while (xQueueReceive(s_command_queue, &envelope, 0u) == pdTRUE) {
                CommandRouter_HandleFrame(envelope.link, &envelope.frame, HAL_GetTick());
            }
        }

        uint32_t now = HAL_GetTick();
        uint32_t faulted_links = MotionControl_PollHeartbeats(now);
        if (faulted_links == 0u) {
            continue;
        }

        for (uint32_t i = 0u; i < (uint32_t)PROTO_LINK_COUNT; i++) {
            if ((faulted_links & (1u << i)) != 0u) {
                CommandRouter_SendError((proto_link_t)i, PROTO_ERROR_HEARTBEAT, 0u);
            }
        }
        Telemetry_SendStatuses(now);
    }
}

static void GimbalTask(void* argument) {
    (void)argument;

    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(APP_CONTROL_PERIOD_MS));
        MotionControl_TickGimbal(HAL_GetTick());
    }
}

static void ChassisTask(void* argument) {
    (void)argument;

    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(APP_CONTROL_PERIOD_MS));
        MotionControl_TickChassis(HAL_GetTick());
    }
}

static void CreateTasks(void) {
    for (uint32_t i = 0u; i < (uint32_t)PROTO_LINK_COUNT; i++) {
        TaskHandle_t link_task = xTaskCreateStatic(LinkTask, (i == (uint32_t)PROTO_LINK_PI) ? "link_pi" : "link_bt",
                                                   APP_TASK_STACK_LINK, &g_app_links[i], APP_TASK_PRIORITY_LINK,
                                                   s_link_stack[i], &s_link_tcb[i]);
        configASSERT(link_task != NULL);

        TaskHandle_t tx_task = xTaskCreateStatic(TxQueue_Task, (i == (uint32_t)PROTO_LINK_PI) ? "tx_pi" : "tx_bt",
                                                 APP_TASK_STACK_TX, &g_app_links[i], APP_TASK_PRIORITY_TX,
                                                 s_tx_stack[i], &s_tx_tcb[i]);
        configASSERT(tx_task != NULL);
    }

    (void)xTaskCreateStatic(CommandTask, "command", APP_TASK_STACK_COMMAND, NULL, APP_TASK_PRIORITY_COMMAND,
                            s_command_stack, &s_command_tcb);
    (void)xTaskCreateStatic(GimbalTask, "gimbal", APP_TASK_STACK_GIMBAL, NULL, APP_TASK_PRIORITY_GIMBAL,
                            s_gimbal_stack, &s_gimbal_tcb);
    (void)xTaskCreateStatic(ChassisTask, "chassis", APP_TASK_STACK_CHASSIS, NULL, APP_TASK_PRIORITY_CHASSIS,
                            s_chassis_stack, &s_chassis_tcb);
    (void)xTaskCreateStatic(Telemetry_Task, "telemetry", APP_TASK_STACK_TELEMETRY, NULL,
                            APP_TASK_PRIORITY_TELEMETRY, s_telemetry_stack, &s_telemetry_tcb);
}

void App_Init(void) {
    MotionControl_Init(HAL_GetTick());

    GimbalPort_Init();
    ChassisPort_Init();
    TxQueue_Init();
    UartConnection_Init(&s_pi_uart, APP_PI_UART);
    UartConnection_Init(&s_bt_uart, APP_BT_UART);

    s_command_queue = xQueueCreateStatic(APP_COMMAND_QUEUE_DEPTH, sizeof(LinkFrame), (uint8_t*)s_command_storage,
                                         &s_command_queue_control);
    configASSERT(s_command_queue != NULL);

    CreateTasks();
}
