#include "motion/motion_arbiter.h"

#include <stddef.h>

static uint8_t NextSequence(uint8_t seq) {
    return (uint8_t)(seq + 1u);
}

static arbiter_admit_t ClassifySequence(bool valid, uint8_t last, uint8_t seq) {
    if (!valid) {
        return ARBITER_ADMIT_OK;
    }
    if (seq == last) {
        return ARBITER_ADMIT_DUPLICATE;
    }
    if (seq == NextSequence(last)) {
        return ARBITER_ADMIT_OK;
    }
    return ARBITER_ADMIT_SEQ_LOST;
}

void Arbiter_Init(MotionArbiter* arbiter, const ArbiterConfig* config, uint32_t now) {
    *arbiter = (MotionArbiter){0};
    if (config != NULL) {
        arbiter->config = *config;
    }
    arbiter->boot_ms = now;
    arbiter->owner = PROTO_OWNER_NONE;
    arbiter->gimbal_state = PROTO_GIMBAL_IDLE;
    arbiter->motion_state = PROTO_MOTION_STOPPED;
    arbiter->chassis_stopped = true;
    arbiter->gimbal_at_target = true;
}

arbiter_admit_t Arbiter_AdmitGimbalMotion(MotionArbiter* arbiter, uint8_t seq, uint32_t now) {
    (void)now;

    if (arbiter->estop_latched || arbiter->fault_latched) {
        return ARBITER_ADMIT_DENIED;
    }
    if (arbiter->owner == PROTO_OWNER_CHASSIS || !arbiter->chassis_stopped) {
        return ARBITER_ADMIT_BUSY;
    }

    arbiter_admit_t result = ClassifySequence(arbiter->gimbal_seq_valid, arbiter->gimbal_seq, seq);
    if (result != ARBITER_ADMIT_OK) {
        return result;
    }

    arbiter->gimbal_seq_valid = true;
    arbiter->gimbal_seq = seq;
    arbiter->gimbal_ack_flags = 0u;
    arbiter->owner = PROTO_OWNER_GIMBAL;
    arbiter->gimbal_state = PROTO_GIMBAL_TRACKING;
    return ARBITER_ADMIT_OK;
}

arbiter_admit_t Arbiter_AdmitVelocity(MotionArbiter* arbiter, uint8_t seq, bool zero, uint32_t now) {
    if (arbiter->estop_latched || arbiter->fault_latched) {
        return ARBITER_ADMIT_DENIED;
    }

    if (zero) {
        /* The all-zero command is always accepted outside emergency stop. A
         * repeated sequence still only repeats its acknowledgement. */
        if (arbiter->motion_seq_valid && seq == arbiter->motion_seq) {
            return ARBITER_ADMIT_DUPLICATE;
        }

        arbiter->motion_seq_valid = true;
        arbiter->motion_seq = seq;
        arbiter->motion_ack_flags = PROTO_ACK_MOTION_ACCEPTED;
        arbiter->motion_state = PROTO_MOTION_STOPPED;
        arbiter->velocity_seen = false;
        return ARBITER_ADMIT_OK;
    }

    if (arbiter->owner == PROTO_OWNER_GIMBAL || arbiter->gimbal_state != PROTO_GIMBAL_IDLE ||
        !arbiter->gimbal_at_target) {
        return ARBITER_ADMIT_BUSY;
    }

    arbiter_admit_t result = ClassifySequence(arbiter->motion_seq_valid, arbiter->motion_seq, seq);
    if (result != ARBITER_ADMIT_OK) {
        return result;
    }

    arbiter->motion_seq_valid = true;
    arbiter->motion_seq = seq;
    arbiter->motion_ack_flags = 0u;
    arbiter->owner = PROTO_OWNER_CHASSIS;
    arbiter->motion_state = PROTO_MOTION_MOVING;
    arbiter->velocity_seen = true;
    arbiter->velocity_ms = now;
    return ARBITER_ADMIT_OK;
}

void Arbiter_SetGimbalAckFlags(MotionArbiter* arbiter, uint8_t flags) {
    arbiter->gimbal_ack_flags = flags;
}

void Arbiter_SetMotionAckFlags(MotionArbiter* arbiter, uint8_t flags) {
    arbiter->motion_ack_flags = flags;
}

static arbiter_mode_t Arbiter_Release(MotionArbiter* arbiter);

arbiter_mode_t Arbiter_SetMode(MotionArbiter* arbiter, uint8_t mode, uint32_t now) {
    (void)now;

    switch (mode) {
    case PROTO_MODE_RELEASE:
        return Arbiter_Release(arbiter);

    case PROTO_MODE_GIMBAL_TRACKING: {
        if (arbiter->estop_latched || arbiter->fault_latched) {
            return ARBITER_MODE_DENIED;
        }
        if (arbiter->owner == PROTO_OWNER_CHASSIS || !arbiter->chassis_stopped) {
            return ARBITER_MODE_BUSY;
        }

        arbiter->owner = PROTO_OWNER_GIMBAL;
        arbiter->gimbal_state = PROTO_GIMBAL_TRACKING;
        arbiter->gimbal_seq_valid = false;
        return ARBITER_MODE_OK;
    }

    case PROTO_MODE_EMERGENCY_STOP:
        Arbiter_EmergencyStop(arbiter);
        return ARBITER_MODE_OK;

    default:
        return ARBITER_MODE_INVALID;
    }
}

static arbiter_mode_t Arbiter_Release(MotionArbiter* arbiter) {
    bool was_emergency = (arbiter->owner == PROTO_OWNER_ESTOP_FAULT);
    bool was_chassis = (arbiter->owner == PROTO_OWNER_CHASSIS);
    bool was_decelerating = (arbiter->motion_state == PROTO_MOTION_TIMEOUT_DECEL);

    arbiter->estop_latched = false;
    arbiter->fault_latched = false;
    arbiter->gimbal_seq_valid = false;
    arbiter->motion_seq_valid = false;
    arbiter->gimbal_state = PROTO_GIMBAL_IDLE;

    if (was_chassis) {
        /* SET_MODE(0) releases the gimbal but not the chassis. Keep the
         * velocity watchdog armed so a lost SET_VELOCITY stream still stops
         * the chassis instead of leaving it on its last command. */
        return ARBITER_MODE_OK;
    }

    arbiter->velocity_seen = false;
    arbiter->owner = PROTO_OWNER_NONE;
    if (was_emergency || (was_decelerating && !arbiter->chassis_stopped)) {
        /* The chassis may still be decelerating; report that until it stops. */
        arbiter->motion_state = PROTO_MOTION_TIMEOUT_DECEL;
    } else {
        arbiter->motion_state = PROTO_MOTION_STOPPED;
    }
    return ARBITER_MODE_OK;
}

void Arbiter_EmergencyStop(MotionArbiter* arbiter) {
    arbiter->estop_latched = true;
    arbiter->owner = PROTO_OWNER_ESTOP_FAULT;
    arbiter->gimbal_state = PROTO_GIMBAL_ESTOP;
    arbiter->motion_state = PROTO_MOTION_ESTOP;
    arbiter->velocity_seen = false;
}

void Arbiter_NoteHeartbeat(MotionArbiter* arbiter, uint32_t now) {
    arbiter->heartbeat_seen = true;
    arbiter->heartbeat_ms = now;
}

bool Arbiter_PollHeartbeat(MotionArbiter* arbiter, uint32_t now) {
    if (arbiter->estop_latched || arbiter->fault_latched) {
        return false;
    }

    uint32_t reference = arbiter->heartbeat_seen ? arbiter->heartbeat_ms : arbiter->boot_ms;
    if ((uint32_t)(now - reference) <= arbiter->config.heartbeat_timeout_ms) {
        return false;
    }

    arbiter->fault_latched = true;
    arbiter->owner = PROTO_OWNER_ESTOP_FAULT;
    arbiter->gimbal_state = PROTO_GIMBAL_ERROR;
    arbiter->motion_state = PROTO_MOTION_FAULT;
    arbiter->velocity_seen = false;
    return true;
}

void Arbiter_SetFeedback(MotionArbiter* arbiter, bool chassis_stopped, bool gimbal_at_target) {
    arbiter->chassis_stopped = chassis_stopped;
    arbiter->gimbal_at_target = gimbal_at_target;
}

void Arbiter_PollTimeouts(MotionArbiter* arbiter, uint32_t now) {
    if (arbiter->estop_latched || arbiter->fault_latched) {
        return;
    }

    if (arbiter->motion_state == PROTO_MOTION_MOVING && arbiter->velocity_seen &&
        (uint32_t)(now - arbiter->velocity_ms) > arbiter->config.velocity_timeout_ms) {
        arbiter->motion_state = PROTO_MOTION_TIMEOUT_DECEL;
        arbiter->velocity_seen = false;
    }

    bool chassis_releasable = (arbiter->motion_state != PROTO_MOTION_MOVING) && arbiter->chassis_stopped;
    if (arbiter->owner == PROTO_OWNER_CHASSIS && chassis_releasable) {
        arbiter->owner = PROTO_OWNER_NONE;
        arbiter->motion_state = PROTO_MOTION_STOPPED;
    } else if (arbiter->owner == PROTO_OWNER_NONE && arbiter->motion_state == PROTO_MOTION_TIMEOUT_DECEL &&
               arbiter->chassis_stopped) {
        arbiter->motion_state = PROTO_MOTION_STOPPED;
    }
}
