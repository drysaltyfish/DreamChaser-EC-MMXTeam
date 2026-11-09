#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "motion/chassis.h"
#include "motion/gimbal.h"
#include "motion/motion_arbiter.h"
#include "protocol/protocol_defs.h"
#include "protocol/protocol_messages.h"

typedef enum {
    MOTION_RESULT_ACCEPTED = 0,
    MOTION_RESULT_DUPLICATE,
    MOTION_RESULT_BUSY,
    MOTION_RESULT_SEQ_LOST,
    MOTION_RESULT_DENIED
} motion_result_t;

typedef struct {
    motion_result_t result;
    uint8_t ack_flags;
    uint8_t expected_seq;
    uint8_t owner;
    uint8_t motion_state;
} MotionOutcome;

void MotionControl_Init(uint32_t now);

void MotionControl_NoteHeartbeat(uint32_t now);

/** Returns true exactly once when the heartbeat timeout latches a fault. */
bool MotionControl_PollHeartbeat(uint32_t now);

MotionOutcome MotionControl_SetTarget(uint8_t seq, int16_t pan, int16_t tilt, uint32_t now);
MotionOutcome MotionControl_SetDelta(uint8_t seq, int16_t delta_pan, int16_t delta_tilt, uint32_t now);
MotionOutcome MotionControl_SetPath(uint8_t seq, const GimbalPoint* points, uint8_t count, uint32_t now);
MotionOutcome MotionControl_SetVelocity(uint8_t seq, int16_t vx, int16_t vy, int16_t omega, uint32_t now);

arbiter_mode_t MotionControl_SetMode(uint8_t mode, uint32_t now);
void MotionControl_EmergencyStop(void);

bool MotionControl_IsCalibrating(void);
bool MotionControl_IsEmergency(void);

void MotionControl_SetCalibrationValid(bool valid);

/** Counts one protocol or communication fault for STATUS.err_count. */
void MotionControl_NoteError(void);

void MotionControl_TickGimbal(uint32_t now);
void MotionControl_TickChassis(uint32_t now);

void MotionControl_GetStatus(ProtoStatus* status, uint32_t now);

#endif /* MOTION_CONTROL_H */
