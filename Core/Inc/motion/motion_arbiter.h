#ifndef MOTION_ARBITER_H
#define MOTION_ARBITER_H

#include <stdbool.h>
#include <stdint.h>

#include "protocol/protocol_defs.h"

/**
 * Arbitration result for a motion command.
 *
 * DUPLICATE means the sequence number equals the last accepted one: the
 * command must not run again, and the cached acknowledgement is repeated.
 */
typedef enum {
    ARBITER_ADMIT_OK = 0,
    ARBITER_ADMIT_DUPLICATE,
    ARBITER_ADMIT_BUSY,
    ARBITER_ADMIT_SEQ_LOST,
    ARBITER_ADMIT_DENIED
} arbiter_admit_t;

typedef enum {
    ARBITER_MODE_OK = 0,
    ARBITER_MODE_BUSY,
    ARBITER_MODE_DENIED,
    ARBITER_MODE_INVALID
} arbiter_mode_t;

typedef struct {
    uint32_t velocity_timeout_ms;
    uint32_t heartbeat_timeout_ms;
} ArbiterConfig;

typedef struct {
    bool seen;
    uint32_t last_ms;
    bool faulted;
} ArbiterHeartbeat;

typedef struct {
    ArbiterConfig config;

    proto_owner_t owner;
    proto_gimbal_state_t gimbal_state;
    proto_motion_state_t motion_state;

    bool gimbal_seq_valid;
    uint8_t gimbal_seq;
    uint8_t gimbal_ack_flags;

    bool motion_seq_valid;
    uint8_t motion_seq;
    uint8_t motion_ack_flags;

    ArbiterHeartbeat heartbeat[PROTO_LINK_COUNT];

    bool velocity_seen;
    uint32_t velocity_ms;

    bool chassis_stopped;
    bool gimbal_at_target;

    bool estop_latched;

    uint16_t error_count;
} MotionArbiter;

void Arbiter_Init(MotionArbiter* arbiter, const ArbiterConfig* config, uint32_t now);

arbiter_admit_t Arbiter_AdmitGimbalMotion(MotionArbiter* arbiter, uint8_t seq, uint32_t now);

/** Accepts or rejects a velocity command. zero means all three axes are zero. */
arbiter_admit_t Arbiter_AdmitVelocity(MotionArbiter* arbiter, uint8_t seq, bool zero, uint32_t now);

/** Stores the flags of the acknowledgement for the accepted sequence. */
void Arbiter_SetGimbalAckFlags(MotionArbiter* arbiter, uint8_t flags);
void Arbiter_SetMotionAckFlags(MotionArbiter* arbiter, uint8_t flags);

arbiter_mode_t Arbiter_SetMode(MotionArbiter* arbiter, proto_link_t link, uint8_t mode, uint32_t now);
void Arbiter_EmergencyStop(MotionArbiter* arbiter);

void Arbiter_NoteHeartbeat(MotionArbiter* arbiter, proto_link_t link, uint32_t now);

/**
 * Checks every link that has already sent a heartbeat.
 *
 * Returns a bit mask of links that just timed out. A link that has never
 * connected is not faulted: a system using only Bluetooth must not latch a
 * fault because the Pi is absent.
 */
uint32_t Arbiter_PollHeartbeats(MotionArbiter* arbiter, uint32_t now);

bool Arbiter_LinkHeartbeatOk(const MotionArbiter* arbiter, proto_link_t link, uint32_t now);
bool Arbiter_LinkFaulted(const MotionArbiter* arbiter, proto_link_t link);

void Arbiter_SetFeedback(MotionArbiter* arbiter, bool chassis_stopped, bool gimbal_at_target);

/** Applies the velocity timeout and releases a stopped chassis. */
void Arbiter_PollTimeouts(MotionArbiter* arbiter, uint32_t now);

#endif /* MOTION_ARBITER_H */