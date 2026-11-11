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

static bool LinkIndexValid(proto_link_t link) {
    return (uint32_t)link < (uint32_t)PROTO_LINK_COUNT;
}

void Arbiter_Init(MotionArbiter* arbiter, const ArbiterConfig* config, uint32_t now) {
    (void)now;

    *arbiter = (MotionArbiter){0};
    if (config != NULL) {
        arbiter->config = *config;
    }
    arbiter->owner = PROTO_OWNER_NONE;
    arbiter->gimbal_state = PROTO_GIMBAL_IDLE;
    arbiter->motion_state = PROTO_MOTION_STOPPED;
    arbiter->chassis_stopped = true;
    arbiter->gimbal_at_target = true;
}

arbiter_admit_t Arbiter_AdmitGimbalMotion(MotionArbiter* arbiter, uint8_t seq, uint32_t now) {
    (void)now;

    if (arbiter->estop_latched) {
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
    if (arbiter->estop_latched) {
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

static arbiter_mode_t Arbiter_Release(MotionArbiter* arbiter, proto_link_t link) {
    bool was_emergency = arbiter->estop_latched;
    arbiter->estop_latched = false;

    if (was_emergency) {
        /* The emergency stop owns every actuator. Releasing it resets both
         * sequence bases; the chassis may still be decelerating. */
        arbiter->gimbal_seq_valid = false;
        arbiter->motion_seq_valid = false;
        arbiter->gimbal_state = PROTO_GIMBAL_IDLE;
        arbiter->gimbal_at_target = true;
        arbiter->velocity_seen = false;
        arbiter->owner = PROTO_OWNER_NONE;
        arbiter->motion_state = arbiter->chassis_stopped ? PROTO_MOTION_STOPPED : PROTO_MOTION_TIMEOUT_DECEL;
        return ARBITER_MODE_OK;
    }

    if (link == PROTO_LINK_PI) {
        /* The Pi releases only the gimbal. A chassis owner and its velocity
         * watchdog are untouched. */
        arbiter->gimbal_seq_valid = false;
        arbiter->gimbal_state = PROTO_GIMBAL_IDLE;
        arbiter->gimbal_at_target = true;
        if (arbiter->owner == PROTO_OWNER_GIMBAL) {
            arbiter->owner = PROTO_OWNER_NONE;
        }
        return ARBITER_MODE_OK;
    }

    /* The Bluetooth client releases only the chassis. It must not disturb a
     * gimbal that the Pi is currently tracking. */
    arbiter->motion_seq_valid = false;
    arbiter->velocity_seen = false;
    if (arbiter->owner == PROTO_OWNER_CHASSIS) {
        if (arbiter->chassis_stopped) {
            arbiter->owner = PROTO_OWNER_NONE;
            arbiter->motion_state = PROTO_MOTION_STOPPED;
        } else {
            arbiter->motion_state = PROTO_MOTION_TIMEOUT_DECEL;
        }
    }
    return ARBITER_MODE_OK;
}

arbiter_mode_t Arbiter_SetMode(MotionArbiter* arbiter, proto_link_t link, uint8_t mode, uint32_t now) {
    (void)now;

    switch (mode) {
    case PROTO_MODE_RELEASE:
        return Arbiter_Release(arbiter, link);

    case PROTO_MODE_GIMBAL_TRACKING: {
        if (link != PROTO_LINK_PI) {
            return ARBITER_MODE_DENIED;
        }
        if (arbiter->estop_latched) {
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

void Arbiter_EmergencyStop(MotionArbiter* arbiter) {
    arbiter->estop_latched = true;
    arbiter->owner = PROTO_OWNER_ESTOP_FAULT;
    arbiter->gimbal_state = PROTO_GIMBAL_ESTOP;
    arbiter->motion_state = PROTO_MOTION_ESTOP;
    arbiter->velocity_seen = false;
}

void Arbiter_NoteHeartbeat(MotionArbiter* arbiter, proto_link_t link, uint32_t now) {
    if (!LinkIndexValid(link)) {
        return;
    }

    arbiter->heartbeat[link].seen = true;
    arbiter->heartbeat[link].last_ms = now;
    arbiter->heartbeat[link].faulted = false;
}

uint32_t Arbiter_PollHeartbeats(MotionArbiter* arbiter, uint32_t now) {
    uint32_t faulted_links = 0u;

    if (arbiter->estop_latched) {
        return 0u;
    }

    for (uint32_t i = 0u; i < (uint32_t)PROTO_LINK_COUNT; i++) {
        ArbiterHeartbeat* heartbeat = &arbiter->heartbeat[i];
        if (!heartbeat->seen || heartbeat->faulted) {
            continue;
        }
        if ((uint32_t)(now - heartbeat->last_ms) <= arbiter->config.heartbeat_timeout_ms) {
            continue;
        }

        heartbeat->faulted = true;
        faulted_links |= (uint32_t)1u << i;

        if ((proto_link_t)i == PROTO_LINK_PI) {
            /* Losing the Pi disables the gimbal but must not stop a chassis
             * that another client is driving. Report the loss in the status
             * flags; IDLE keeps the chassis admissible. */
            if (arbiter->owner == PROTO_OWNER_GIMBAL) {
                arbiter->owner = PROTO_OWNER_NONE;
            }
            if (arbiter->gimbal_state != PROTO_GIMBAL_ESTOP) {
                arbiter->gimbal_state = PROTO_GIMBAL_IDLE;
            }
            arbiter->gimbal_at_target = true;
        } else {
            /* Losing the Bluetooth client stops the chassis with the
             * strongest ramp; the owner is released once it has stopped. */
            if (arbiter->motion_state == PROTO_MOTION_MOVING) {
                arbiter->motion_state = PROTO_MOTION_TIMEOUT_DECEL;
            }
            arbiter->velocity_seen = false;
        }
    }

    return faulted_links;
}

bool Arbiter_LinkHeartbeatOk(const MotionArbiter* arbiter, proto_link_t link, uint32_t now) {
    if (!LinkIndexValid(link)) {
        return false;
    }

    const ArbiterHeartbeat* heartbeat = &arbiter->heartbeat[link];
    return heartbeat->seen && !heartbeat->faulted &&
           (uint32_t)(now - heartbeat->last_ms) <= arbiter->config.heartbeat_timeout_ms;
}

bool Arbiter_LinkFaulted(const MotionArbiter* arbiter, proto_link_t link) {
    if (!LinkIndexValid(link)) {
        return false;
    }
    return arbiter->heartbeat[link].faulted;
}

void Arbiter_SetFeedback(MotionArbiter* arbiter, bool chassis_stopped, bool gimbal_at_target) {
    arbiter->chassis_stopped = chassis_stopped;
    arbiter->gimbal_at_target = gimbal_at_target;
}

void Arbiter_PollTimeouts(MotionArbiter* arbiter, uint32_t now) {
    if (arbiter->estop_latched) {
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
