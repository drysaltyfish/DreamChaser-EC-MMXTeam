#include "app/telemetry.h"

#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app/app_config.h"
#include "app/tx_queue.h"
#include "motion/motion_control.h"
#include "motor.h"
#include "protocol/protocol_messages.h"

static bool s_gimbal_seen;
static ProtoGimbalStatus s_last_gimbal;
static uint32_t s_last_gimbal_ms;

static bool s_chassis_seen;
static ProtoChassisStatus s_last_chassis;
static uint32_t s_last_chassis_ms;

static uint32_t s_last_motor_ms;

void Telemetry_SendReady(proto_link_t link) {
    ProtoReady ready = {
        .version_major = PROTO_VERSION_MAJOR,
        .version_minor = PROTO_VERSION_MINOR,
        .flags = (link == PROTO_LINK_PI) ? (uint8_t)PROTO_READY_HAS_GIMBAL
                                         : (uint8_t)(PROTO_READY_HAS_CHASSIS | PROTO_READY_HAS_MOTOR_TELEMETRY),
    };

    ProtoFrame frame;
    Protocol_MakeReady(&frame, &ready);
    (void)TxQueue_Send(link, &frame);
}

void Telemetry_SendGimbalStatus(uint32_t now) {
    MotionStatus status;
    MotionControl_GetStatus(&status, now);

    ProtoGimbalStatus message = {
        .gimbal_state = status.gimbal_state,
        .owner = status.owner,
        .pan = status.pan,
        .tilt = status.tilt,
        .error_count = status.error_count,
        .flags = status.flags,
    };

    ProtoFrame frame;
    Protocol_MakeGimbalStatus(&frame, &message);
    (void)TxQueue_Send(PROTO_LINK_PI, &frame);
}

void Telemetry_SendChassisStatus(uint32_t now) {
    MotionStatus status;
    MotionControl_GetStatus(&status, now);

    ProtoChassisStatus message = {
        .motion_state = status.motion_state,
        .owner = status.owner,
        .velocity_x = status.velocity_x,
        .velocity_y = status.velocity_y,
        .omega = status.omega,
        .error_count = status.error_count,
        .flags = status.flags,
    };

    ProtoFrame frame;
    Protocol_MakeChassisStatus(&frame, &message);
    (void)TxQueue_Send(PROTO_LINK_BT, &frame);
}

void Telemetry_SendStatuses(uint32_t now) {
    Telemetry_SendGimbalStatus(now);
    Telemetry_SendChassisStatus(now);
}

static int16_t RpmToInt16(float rpm) {
    if (rpm >= 32767.0f) {
        return INT16_MAX;
    }
    if (rpm <= -32768.0f) {
        return INT16_MIN;
    }
    return (int16_t)(rpm >= 0.0f ? rpm + 0.5f : rpm - 0.5f);
}

static void SendMotorStatus(void) {
    MotorTelemetry telemetry;
    Motor_GetTelemetry(&telemetry);

    ProtoMotorStatus status = {.present_mask = telemetry.present_mask};
    for (uint32_t i = 0u; i < PROTO_MOTOR_COUNT; i++) {
        status.rpm[i] = RpmToInt16(telemetry.rpm[i]);
    }

    ProtoFrame frame;
    Protocol_MakeMotorStatus(&frame, &status);
    (void)TxQueue_Send(PROTO_LINK_BT, &frame);
}

static bool GimbalStatusChanged(const ProtoGimbalStatus* status) {
    if (!s_gimbal_seen) {
        return true;
    }
    return status->gimbal_state != s_last_gimbal.gimbal_state || status->owner != s_last_gimbal.owner ||
           status->flags != s_last_gimbal.flags;
}

static bool ChassisStatusChanged(const ProtoChassisStatus* status) {
    if (!s_chassis_seen) {
        return true;
    }
    return status->motion_state != s_last_chassis.motion_state || status->owner != s_last_chassis.owner ||
           status->flags != s_last_chassis.flags;
}

void Telemetry_Task(void* argument) {
    (void)argument;

    uint32_t now = HAL_GetTick();
    Telemetry_SendReady(PROTO_LINK_PI);
    Telemetry_SendReady(PROTO_LINK_BT);
    Telemetry_SendGimbalStatus(now);
    Telemetry_SendChassisStatus(now);
    SendMotorStatus();
    s_last_gimbal_ms = now;
    s_last_chassis_ms = now;
    s_last_motor_ms = now;

    for (;;) {
        now = HAL_GetTick();

        ProtoGimbalStatus gimbal;
        ProtoChassisStatus chassis;
        MotionStatus status;
        MotionControl_GetStatus(&status, now);

        gimbal = (ProtoGimbalStatus){
            .gimbal_state = status.gimbal_state,
            .owner = status.owner,
            .pan = status.pan,
            .tilt = status.tilt,
            .error_count = status.error_count,
            .flags = status.flags,
        };
        chassis = (ProtoChassisStatus){
            .motion_state = status.motion_state,
            .owner = status.owner,
            .velocity_x = status.velocity_x,
            .velocity_y = status.velocity_y,
            .omega = status.omega,
            .error_count = status.error_count,
            .flags = status.flags,
        };

        bool gimbal_due = (uint32_t)(now - s_last_gimbal_ms) >= APP_TELEMETRY_PERIOD_MS;
        if (GimbalStatusChanged(&gimbal) || gimbal_due) {
            Telemetry_SendGimbalStatus(now);
            s_last_gimbal = gimbal;
            s_last_gimbal_ms = now;
            s_gimbal_seen = true;
        }

        bool chassis_due = (uint32_t)(now - s_last_chassis_ms) >= APP_CHASSIS_TELEMETRY_PERIOD_MS;
        if (ChassisStatusChanged(&chassis) || chassis_due) {
            Telemetry_SendChassisStatus(now);
            s_last_chassis = chassis;
            s_last_chassis_ms = now;
            s_chassis_seen = true;
        }

        if ((uint32_t)(now - s_last_motor_ms) >= APP_MOTOR_TELEMETRY_PERIOD_MS) {
            SendMotorStatus();
            s_last_motor_ms = now;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
