#include "app/telemetry.h"

#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app/app_config.h"
#include "app/tx_queue.h"
#include "motion/motion_control.h"
#include "protocol/protocol_messages.h"

static bool s_status_seen;
static ProtoStatus s_last_status;
static uint32_t s_last_status_ms;

static void SendFrame(const ProtoFrame* frame) {
    (void)TxQueue_Send(frame);
}

void Telemetry_SendReady(void) {
    ProtoReady ready = {
        .version_major = PROTO_VERSION_MAJOR,
        .version_minor = PROTO_VERSION_MINOR,
        .flags = PROTO_READY_HAS_GIMBAL,
    };
#if APP_CHASSIS_DRIVE_ENABLED
    ready.flags |= PROTO_READY_HAS_CHASSIS;
#endif

    ProtoFrame frame;
    Protocol_MakeReady(&frame, &ready);
    SendFrame(&frame);
}

void Telemetry_SendStatus(uint32_t now) {
    ProtoStatus status;
    MotionControl_GetStatus(&status, now);

    ProtoFrame frame;
    Protocol_MakeStatus(&frame, &status);
    SendFrame(&frame);
}

static bool StatusChanged(const ProtoStatus* status) {
    if (!s_status_seen) {
        return true;
    }
    return status->gimbal_state != s_last_status.gimbal_state || status->owner != s_last_status.owner ||
           status->motion_state != s_last_status.motion_state || status->flags != s_last_status.flags;
}

void Telemetry_Task(void* argument) {
    (void)argument;

    Telemetry_SendReady();
    s_last_status_ms = HAL_GetTick();

    for (;;) {
        uint32_t now = HAL_GetTick();

        ProtoStatus status;
        MotionControl_GetStatus(&status, now);
        bool due = (uint32_t)(now - s_last_status_ms) >= APP_TELEMETRY_PERIOD_MS;

        if (StatusChanged(&status) || due) {
            ProtoFrame frame;
            Protocol_MakeStatus(&frame, &status);
            SendFrame(&frame);
            s_last_status = status;
            s_last_status_ms = now;
            s_status_seen = true;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
