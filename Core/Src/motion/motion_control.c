#include "motion/motion_control.h"

#include <stddef.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "app/app_config.h"
#include "app/chassis_port.h"
#include "app/gimbal_port.h"

typedef struct {
    MotionArbiter arbiter;
    GimbalState gimbal;
    ChassisState chassis;
    bool emergency_decelerating;
} MotionControl;

static MotionControl s_control;
static SemaphoreHandle_t s_mutex;
static StaticSemaphore_t s_mutex_control;

static void MotionControl_Lock(void) {
    (void)xSemaphoreTake(s_mutex, portMAX_DELAY);
}

static void MotionControl_Unlock(void) {
    (void)xSemaphoreGive(s_mutex);
}

static MotionOutcome MakeOutcome(motion_result_t result, uint8_t flags, uint8_t expected_seq,
                                 const MotionArbiter* arbiter) {
    return (MotionOutcome){
        .result = result,
        .ack_flags = flags,
        .expected_seq = expected_seq,
        .owner = (uint8_t)arbiter->owner,
        .motion_state = (uint8_t)arbiter->motion_state,
    };
}

static motion_result_t ConvertAdmit(arbiter_admit_t result) {
    switch (result) {
    case ARBITER_ADMIT_OK:
        return MOTION_RESULT_ACCEPTED;
    case ARBITER_ADMIT_DUPLICATE:
        return MOTION_RESULT_DUPLICATE;
    case ARBITER_ADMIT_BUSY:
        return MOTION_RESULT_BUSY;
    case ARBITER_ADMIT_SEQ_LOST:
        return MOTION_RESULT_SEQ_LOST;
    default:
        return MOTION_RESULT_DENIED;
    }
}

void MotionControl_Init(uint32_t now) {
    s_mutex = xSemaphoreCreateMutexStatic(&s_mutex_control);

    const GimbalConfig gimbal_config = {
        .pan_min = APP_GIMBAL_PAN_MIN,
        .pan_max = APP_GIMBAL_PAN_MAX,
        .tilt_min = APP_GIMBAL_TILT_MIN,
        .tilt_max = APP_GIMBAL_TILT_MAX,
        .max_speed_lsb_per_s = APP_GIMBAL_MAX_SPEED_LSB_PER_S,
        .arrival_tolerance_lsb = APP_GIMBAL_ARRIVAL_TOLERANCE_LSB,
        .max_delta_lsb = PROTO_DELTA_LIMIT,
    };
    const ChassisConfig chassis_config = {
        .accel_per_s = APP_CHASSIS_ACCEL_PER_S,
        .decel_per_s = APP_CHASSIS_DECEL_PER_S,
        .emergency_decel_per_s = APP_CHASSIS_ESTOP_DECEL_PER_S,
        .deadzone = APP_CHASSIS_DEADZONE,
    };
    const ArbiterConfig arbiter_config = {
        .velocity_timeout_ms = APP_VELOCITY_TIMEOUT_MS,
        .heartbeat_timeout_ms = APP_HEARTBEAT_TIMEOUT_MS,
    };

    Gimbal_Init(&s_control.gimbal, &gimbal_config);
    Chassis_Init(&s_control.chassis, &chassis_config);
    Arbiter_Init(&s_control.arbiter, &arbiter_config, now);
    s_control.emergency_decelerating = false;
}

void MotionControl_NoteHeartbeat(uint32_t now) {
    MotionControl_Lock();
    Arbiter_NoteHeartbeat(&s_control.arbiter, now);
    MotionControl_Unlock();
}

bool MotionControl_PollHeartbeat(uint32_t now) {
    MotionControl_Lock();
    bool faulted = Arbiter_PollHeartbeat(&s_control.arbiter, now);
    if (faulted) {
        Gimbal_Disable(&s_control.gimbal);
        Arbiter_SetFeedback(&s_control.arbiter, s_control.arbiter.chassis_stopped, true);
        Chassis_EmergencyStop(&s_control.chassis);
        s_control.emergency_decelerating = true;
    }
    MotionControl_Unlock();

    if (faulted) {
        GimbalPort_Disable();
    }
    return faulted;
}

MotionOutcome MotionControl_SetTarget(uint8_t seq, int16_t pan, int16_t tilt, uint32_t now) {
    MotionControl_Lock();
    arbiter_admit_t admit = Arbiter_AdmitGimbalMotion(&s_control.arbiter, seq, now);
    MotionOutcome outcome = MakeOutcome(ConvertAdmit(admit), s_control.arbiter.gimbal_ack_flags,
                                        (uint8_t)(s_control.arbiter.gimbal_seq + 1u), &s_control.arbiter);

    if (admit == ARBITER_ADMIT_OK) {
        Gimbal_Enable(&s_control.gimbal);
        bool clamped = Gimbal_SetTarget(&s_control.gimbal, pan, tilt);
        uint8_t flags = PROTO_ACK_ACCEPTED;
        if (clamped) {
            flags |= PROTO_ACK_CLAMPED;
        }
        if (Gimbal_AtTarget(&s_control.gimbal)) {
            flags |= PROTO_ACK_AT_TARGET;
        }
        Arbiter_SetGimbalAckFlags(&s_control.arbiter, flags);
        outcome.ack_flags = flags;
    } else if (admit == ARBITER_ADMIT_DUPLICATE) {
        outcome.ack_flags = s_control.arbiter.gimbal_ack_flags;
    }

    MotionControl_Unlock();
    return outcome;
}

MotionOutcome MotionControl_SetDelta(uint8_t seq, int16_t delta_pan, int16_t delta_tilt, uint32_t now) {
    MotionControl_Lock();
    arbiter_admit_t admit = Arbiter_AdmitGimbalMotion(&s_control.arbiter, seq, now);
    MotionOutcome outcome = MakeOutcome(ConvertAdmit(admit), s_control.arbiter.gimbal_ack_flags,
                                        (uint8_t)(s_control.arbiter.gimbal_seq + 1u), &s_control.arbiter);

    if (admit == ARBITER_ADMIT_OK) {
        Gimbal_Enable(&s_control.gimbal);
        bool clamped = Gimbal_AddDelta(&s_control.gimbal, delta_pan, delta_tilt);
        uint8_t flags = PROTO_ACK_ACCEPTED;
        if (clamped) {
            flags |= PROTO_ACK_CLAMPED;
        }
        if (Gimbal_AtTarget(&s_control.gimbal)) {
            flags |= PROTO_ACK_AT_TARGET;
        }
        Arbiter_SetGimbalAckFlags(&s_control.arbiter, flags);
        outcome.ack_flags = flags;
    } else if (admit == ARBITER_ADMIT_DUPLICATE) {
        outcome.ack_flags = s_control.arbiter.gimbal_ack_flags;
    }

    MotionControl_Unlock();
    return outcome;
}

MotionOutcome MotionControl_SetPath(uint8_t seq, const GimbalPoint* points, uint8_t count, uint32_t now) {
    MotionControl_Lock();
    arbiter_admit_t admit = Arbiter_AdmitGimbalMotion(&s_control.arbiter, seq, now);
    MotionOutcome outcome = MakeOutcome(ConvertAdmit(admit), s_control.arbiter.gimbal_ack_flags,
                                        (uint8_t)(s_control.arbiter.gimbal_seq + 1u), &s_control.arbiter);

    if (admit == ARBITER_ADMIT_OK) {
        Gimbal_Enable(&s_control.gimbal);
        bool clamped = Gimbal_SetPath(&s_control.gimbal, points, count);
        uint8_t flags = PROTO_ACK_ACCEPTED;
        if (clamped) {
            flags |= PROTO_ACK_CLAMPED;
        }
        Arbiter_SetGimbalAckFlags(&s_control.arbiter, flags);
        outcome.ack_flags = flags;
    } else if (admit == ARBITER_ADMIT_DUPLICATE) {
        outcome.ack_flags = s_control.arbiter.gimbal_ack_flags;
    }

    MotionControl_Unlock();
    return outcome;
}

MotionOutcome MotionControl_SetVelocity(uint8_t seq, int16_t vx, int16_t vy, int16_t omega, uint32_t now) {
    bool zero = (vx == 0) && (vy == 0) && (omega == 0);

    MotionControl_Lock();
    arbiter_admit_t admit = Arbiter_AdmitVelocity(&s_control.arbiter, seq, zero, now);
    MotionOutcome outcome = MakeOutcome(ConvertAdmit(admit), s_control.arbiter.motion_ack_flags,
                                        (uint8_t)(s_control.arbiter.motion_seq + 1u), &s_control.arbiter);

    if (admit == ARBITER_ADMIT_OK) {
        if (zero) {
            Chassis_Stop(&s_control.chassis);
            outcome.ack_flags = PROTO_ACK_MOTION_ACCEPTED;
        } else {
            /* A new moving command supersedes any emergency ramp that is still
             * running after SET_MODE(0). Without this, the next chassis tick
             * would call Chassis_EmergencyStop and silently discard the
             * accepted command. A zero command keeps the stronger ramp. */
            s_control.emergency_decelerating = false;
            Chassis_SetCommand(&s_control.chassis, vx, vy, omega);
            bool clamped = (vx < -1000) || (vx > 1000) || (vy < -1000) || (vy > 1000) ||
                           (omega < -1000) || (omega > 1000);
            outcome.ack_flags = PROTO_ACK_MOTION_ACCEPTED;
            if (clamped) {
                outcome.ack_flags |= PROTO_ACK_MOTION_CLAMPED;
            }
        }
        Arbiter_SetMotionAckFlags(&s_control.arbiter, outcome.ack_flags);
    } else if (admit == ARBITER_ADMIT_DUPLICATE) {
        outcome.ack_flags = s_control.arbiter.motion_ack_flags;
    }

    outcome.motion_state = (uint8_t)s_control.arbiter.motion_state;
    MotionControl_Unlock();
    return outcome;
}

arbiter_mode_t MotionControl_SetMode(uint8_t mode, uint32_t now) {
    MotionControl_Lock();
    arbiter_mode_t result = Arbiter_SetMode(&s_control.arbiter, mode, now);

    if (result == ARBITER_MODE_OK) {
        if (mode == PROTO_MODE_GIMBAL_TRACKING) {
            Gimbal_Enable(&s_control.gimbal);
        } else {
            /* Release mode leaves the servos disabled. */
            Gimbal_Disable(&s_control.gimbal);
            Arbiter_SetFeedback(&s_control.arbiter, s_control.arbiter.chassis_stopped, true);
        }
    }
    MotionControl_Unlock();

    if (result == ARBITER_MODE_OK && mode == PROTO_MODE_RELEASE) {
        GimbalPort_Disable();
    }
    return result;
}

void MotionControl_EmergencyStop(void) {
    MotionControl_Lock();
    Arbiter_EmergencyStop(&s_control.arbiter);
    Gimbal_Disable(&s_control.gimbal);
    Arbiter_SetFeedback(&s_control.arbiter, s_control.arbiter.chassis_stopped, true);
    Chassis_EmergencyStop(&s_control.chassis);
    s_control.emergency_decelerating = true;
    MotionControl_Unlock();

    GimbalPort_Disable();
}

bool MotionControl_IsEmergency(void) {
    MotionControl_Lock();
    bool emergency = s_control.arbiter.estop_latched || s_control.arbiter.fault_latched;
    MotionControl_Unlock();
    return emergency;
}

void MotionControl_NoteError(void) {
    MotionControl_Lock();
    if (s_control.arbiter.error_count < UINT16_MAX) {
        s_control.arbiter.error_count++;
    }
    MotionControl_Unlock();
}

void MotionControl_TickGimbal(uint32_t now) {
    (void)now;

    MotionControl_Lock();
    Gimbal_Step(&s_control.gimbal, APP_CONTROL_PERIOD_MS);
    bool at_target = Gimbal_AtTarget(&s_control.gimbal);
    Arbiter_SetFeedback(&s_control.arbiter, s_control.arbiter.chassis_stopped, at_target);

    int16_t pan = 0;
    int16_t tilt = 0;
    Gimbal_GetActual(&s_control.gimbal, &pan, &tilt);

    /* Keep the state update and the port write atomic with respect to an
     * emergency stop from a higher priority task. */
    if (s_control.gimbal.enabled) {
        GimbalPort_Write(pan, tilt);
    } else {
        GimbalPort_Disable();
    }
    MotionControl_Unlock();
}

void MotionControl_TickChassis(uint32_t now) {
    MotionControl_Lock();

    Arbiter_PollTimeouts(&s_control.arbiter, now);

    bool emergency = s_control.arbiter.estop_latched || s_control.arbiter.fault_latched ||
                     s_control.emergency_decelerating;
    if (emergency) {
        Chassis_EmergencyStop(&s_control.chassis);
    } else if (s_control.arbiter.motion_state == PROTO_MOTION_TIMEOUT_DECEL) {
        /* A lost velocity stream decelerates with the normal profile. */
        Chassis_Stop(&s_control.chassis);
    }

    Chassis_Step(&s_control.chassis, APP_CONTROL_PERIOD_MS);

    bool stopped = Chassis_IsStopped(&s_control.chassis);
    if (stopped) {
        s_control.emergency_decelerating = false;
    }

    Arbiter_SetFeedback(&s_control.arbiter, stopped, s_control.arbiter.gimbal_at_target);
    Arbiter_PollTimeouts(&s_control.arbiter, now);

    float vx = 0.0f;
    float vy = 0.0f;
    float omega = 0.0f;
    Chassis_GetOutput(&s_control.chassis, &vx, &vy, &omega);
    ChassisPort_Apply(vx, vy, omega);
    MotionControl_Unlock();
}

void MotionControl_GetStatus(ProtoStatus* status, uint32_t now) {
    MotionControl_Lock();

    int16_t pan = 0;
    int16_t tilt = 0;
    Gimbal_GetActual(&s_control.gimbal, &pan, &tilt);

    uint8_t flags = 0u;
    if (s_control.arbiter.estop_latched || s_control.arbiter.fault_latched) {
        flags |= PROTO_STATUS_EMERGENCY;
    }
    if (s_control.arbiter.heartbeat_seen &&
        (uint32_t)(now - s_control.arbiter.heartbeat_ms) <= s_control.arbiter.config.heartbeat_timeout_ms) {
        flags |= PROTO_STATUS_HEARTBEAT_OK;
    }
    if (s_control.arbiter.chassis_stopped) {
        flags |= PROTO_STATUS_CHASSIS_STOPPED;
    }

    status->gimbal_state = (uint8_t)s_control.arbiter.gimbal_state;
    status->owner = (uint8_t)s_control.arbiter.owner;
    status->motion_state = (uint8_t)s_control.arbiter.motion_state;
    status->pan = pan;
    status->tilt = tilt;
    status->error_count = s_control.arbiter.error_count;
    status->flags = flags;

    MotionControl_Unlock();
}
