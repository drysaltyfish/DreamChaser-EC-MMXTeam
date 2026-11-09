#include "app/command_router.h"

#include <stddef.h>

#include "app/calibration.h"
#include "app/telemetry.h"
#include "app/tx_queue.h"
#include "motion/motion_control.h"
#include "protocol/protocol_messages.h"

static void SendFrame(const ProtoFrame* frame) {
    (void)TxQueue_Send(frame);
}

void CommandRouter_SendError(uint8_t code, uint8_t detail) {
    MotionControl_NoteError();

    ProtoFrame frame;
    Protocol_MakeError(&frame, code, detail);
    SendFrame(&frame);
}

static void SendAck(uint8_t command, uint8_t seq, uint8_t flags) {
    ProtoFrame frame;
    Protocol_MakeAck(&frame, command, seq, flags);
    SendFrame(&frame);
}

static void SendNackBusy(uint8_t command, uint8_t seq, uint8_t owner) {
    ProtoFrame frame;
    Protocol_MakeNackBusy(&frame, command, seq, owner);
    SendFrame(&frame);
}

static void SendNackSeqLost(uint8_t command, uint8_t expected_seq) {
    ProtoFrame frame;
    Protocol_MakeNackSeqLost(&frame, command, expected_seq);
    SendFrame(&frame);
}

static uint8_t CurrentOwner(uint32_t now) {
    ProtoStatus status;
    MotionControl_GetStatus(&status, now);
    return status.owner;
}

static void ReplyGimbalMotion(uint8_t command, uint8_t seq, const MotionOutcome* outcome) {
    switch (outcome->result) {
    case MOTION_RESULT_ACCEPTED:
    case MOTION_RESULT_DUPLICATE:
        SendAck(command, seq, outcome->ack_flags);
        break;

    case MOTION_RESULT_BUSY:
        SendNackBusy(command, seq, outcome->owner);
        break;

    case MOTION_RESULT_SEQ_LOST:
        SendNackSeqLost(command, outcome->expected_seq);
        break;

    default:
        CommandRouter_SendError(PROTO_ERROR_STATE, command);
        break;
    }
}

static void ReplyVelocity(uint8_t seq, const MotionOutcome* outcome) {
    ProtoFrame frame;

    switch (outcome->result) {
    case MOTION_RESULT_ACCEPTED:
    case MOTION_RESULT_DUPLICATE:
        Protocol_MakeAckMotion(&frame, seq, outcome->motion_state, outcome->ack_flags);
        SendFrame(&frame);
        break;

    case MOTION_RESULT_BUSY:
        /* A rejected velocity keeps the current owner and sequence; the reply
         * is an ACK_MOTION with motion_state BUSY_REJECT, not NACK_BUSY. */
        Protocol_MakeAckMotion(&frame, seq, PROTO_MOTION_BUSY_REJECT, 0u);
        SendFrame(&frame);
        break;

    case MOTION_RESULT_SEQ_LOST:
        SendNackSeqLost(PROTO_CMD_SET_VELOCITY, outcome->expected_seq);
        break;

    default:
        CommandRouter_SendError(PROTO_ERROR_STATE, PROTO_CMD_SET_VELOCITY);
        break;
    }
}

/**
 * A gimbal command needs a valid calibration, and must not interrupt the
 * calibration procedure itself. Arbitration comes first: a busy chassis or an
 * active emergency stop outranks a missing calibration, so the Pi sees the
 * reason the protocol requires instead of a configuration error.
 */
static bool GimbalMotionAllowed(uint8_t command, uint8_t seq, uint32_t now) {
    ProtoStatus status;
    MotionControl_GetStatus(&status, now);

    if ((status.flags & PROTO_STATUS_EMERGENCY) != 0u) {
        CommandRouter_SendError(PROTO_ERROR_STATE, command);
        return false;
    }
    if (status.owner == PROTO_OWNER_CHASSIS ||
        (status.flags & PROTO_STATUS_CHASSIS_STOPPED) == 0u) {
        SendNackBusy(command, seq, status.owner);
        return false;
    }
    if (status.gimbal_state == PROTO_GIMBAL_CALIBRATING) {
        SendNackBusy(command, seq, PROTO_OWNER_GIMBAL);
        return false;
    }
    if ((status.flags & PROTO_STATUS_CAL_VALID) == 0u) {
        CommandRouter_SendError(PROTO_ERROR_CALIBRATION_MISSING, command);
        return false;
    }
    return true;
}

static void HandleSetTarget(const ProtoFrame* frame, uint32_t now) {
    ProtoSetTarget message;

    if (!Protocol_ParseSetTarget(frame, &message)) {
        CommandRouter_SendError(PROTO_ERROR_FRAME, frame->cmd);
        return;
    }
    if (!GimbalMotionAllowed(frame->cmd, message.seq, now)) {
        return;
    }

    MotionOutcome outcome = MotionControl_SetTarget(message.seq, message.pan, message.tilt, now);
    ReplyGimbalMotion(frame->cmd, message.seq, &outcome);
}

static void HandleSetDelta(const ProtoFrame* frame, uint32_t now) {
    ProtoSetDelta message;

    if (!Protocol_ParseSetDelta(frame, &message)) {
        CommandRouter_SendError(PROTO_ERROR_FRAME, frame->cmd);
        return;
    }
    if (!GimbalMotionAllowed(frame->cmd, message.seq, now)) {
        return;
    }

    MotionOutcome outcome = MotionControl_SetDelta(message.seq, message.delta_pan, message.delta_tilt, now);
    ReplyGimbalMotion(frame->cmd, message.seq, &outcome);
}

static void HandleSetTargetStream(const ProtoFrame* frame, uint32_t now) {
    ProtoSetTargetStream message;

    if (!Protocol_ParseSetTargetStream(frame, &message)) {
        CommandRouter_SendError(PROTO_ERROR_FRAME, frame->cmd);
        return;
    }
    if (!GimbalMotionAllowed(frame->cmd, message.seq, now)) {
        return;
    }

    GimbalPoint points[PROTO_STREAM_MAX];
    for (uint8_t i = 0u; i < message.count; i++) {
        points[i].pan = message.pan[i];
        points[i].tilt = message.tilt[i];
    }

    MotionOutcome outcome = MotionControl_SetPath(message.seq, points, message.count, now);
    ReplyGimbalMotion(frame->cmd, message.seq, &outcome);
}

static void HandleSetVelocity(const ProtoFrame* frame, uint32_t now) {
    ProtoSetVelocity message;

    if (!Protocol_ParseSetVelocity(frame, &message)) {
        CommandRouter_SendError(PROTO_ERROR_FRAME, frame->cmd);
        return;
    }

    MotionOutcome outcome =
        MotionControl_SetVelocity(message.seq, message.velocity_x, message.velocity_y, message.omega, now);
    ReplyVelocity(message.seq, &outcome);
}

static void HandleSetMode(const ProtoFrame* frame, uint32_t now) {
    uint8_t mode;

    if (!Protocol_ParseSetMode(frame, &mode)) {
        CommandRouter_SendError(PROTO_ERROR_FRAME, frame->cmd);
        return;
    }

    if (mode == PROTO_MODE_EMERGENCY_STOP) {
        MotionControl_EmergencyStop();
        Telemetry_SendStatus(now);
        return;
    }

    arbiter_mode_t result = MotionControl_SetMode(mode, now);
    switch (result) {
    case ARBITER_MODE_OK:
        SendAck(frame->cmd, 0u, PROTO_ACK_ACCEPTED);
        break;

    case ARBITER_MODE_BUSY:
        SendNackBusy(frame->cmd, 0u, CurrentOwner(now));
        break;

    case ARBITER_MODE_DENIED:
        CommandRouter_SendError(PROTO_ERROR_STATE, frame->cmd);
        break;

    default:
        CommandRouter_SendError(PROTO_ERROR_FRAME, frame->cmd);
        break;
    }
}

static void HandleCalibrate(const ProtoFrame* frame, uint32_t now) {
    (void)now;

    ProtoCalibrate message;

    if (!Protocol_ParseCalibrate(frame, &message)) {
        CommandRouter_SendError(PROTO_ERROR_FRAME, frame->cmd);
        return;
    }

    switch (message.subcmd) {
    case PROTO_CALIBRATE_QUERY:
        Telemetry_SendReady();
        break;

    case PROTO_CALIBRATE_PAN:
    case PROTO_CALIBRATE_TILT: {
        if (!MotionControl_IsCalibrating()) {
            CommandRouter_SendError(PROTO_ERROR_STATE, frame->cmd);
            break;
        }
        if (!message.has_value) {
            CommandRouter_SendError(PROTO_ERROR_FRAME, frame->cmd);
            break;
        }
        if (message.subcmd == PROTO_CALIBRATE_PAN) {
            Calibration_SetPanZero(message.value);
        } else {
            Calibration_SetTiltZero(message.value);
        }
        SendAck(frame->cmd, 0u, PROTO_ACK_ACCEPTED);
        break;
    }

    case PROTO_CALIBRATE_COMMIT:
        if (!MotionControl_IsCalibrating()) {
            CommandRouter_SendError(PROTO_ERROR_STATE, frame->cmd);
            break;
        }
        Calibration_RequestCommit();
        Telemetry_RequestCommit();
        SendAck(frame->cmd, 0u, PROTO_ACK_ACCEPTED);
        break;

    default:
        CommandRouter_SendError(PROTO_ERROR_FRAME, frame->cmd);
        break;
    }
}

void CommandRouter_HandleFrame(const ProtoFrame* frame, uint32_t now) {
    if (frame == NULL) {
        return;
    }

    switch (frame->cmd) {
    case PROTO_CMD_HEARTBEAT:
        if (frame->data_len != 0u) {
            CommandRouter_SendError(PROTO_ERROR_FRAME, frame->cmd);
            break;
        }
        MotionControl_NoteHeartbeat(now);
        break;

    case PROTO_CMD_SET_TARGET:
        HandleSetTarget(frame, now);
        break;

    case PROTO_CMD_SET_DELTA:
        HandleSetDelta(frame, now);
        break;

    case PROTO_CMD_SET_TARGET_STREAM:
        HandleSetTargetStream(frame, now);
        break;

    case PROTO_CMD_SET_VELOCITY:
        HandleSetVelocity(frame, now);
        break;

    case PROTO_CMD_SET_MODE:
        HandleSetMode(frame, now);
        break;

    case PROTO_CMD_CALIBRATE:
        HandleCalibrate(frame, now);
        break;

    default:
        CommandRouter_SendError(PROTO_ERROR_UNKNOWN_CMD, frame->cmd);
        break;
    }
}
