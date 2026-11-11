#include "app/command_router.h"

#include <stddef.h>

#include "app/telemetry.h"
#include "app/tx_queue.h"
#include "motion/motion_control.h"
#include "protocol/protocol_messages.h"

static void SendFrame(proto_link_t link, const ProtoFrame* frame) {
    (void)TxQueue_Send(link, frame);
}

void CommandRouter_SendError(proto_link_t link, uint8_t code, uint8_t detail) {
    MotionControl_NoteError();

    ProtoFrame frame;
    Protocol_MakeError(&frame, code, detail);
    SendFrame(link, &frame);
}

static void SendAck(proto_link_t link, uint8_t command, uint8_t seq, uint8_t flags) {
    ProtoFrame frame;
    Protocol_MakeAck(&frame, command, seq, flags);
    SendFrame(link, &frame);
}

static void SendNackBusy(proto_link_t link, uint8_t command, uint8_t seq, uint8_t owner) {
    ProtoFrame frame;
    Protocol_MakeNackBusy(&frame, command, seq, owner);
    SendFrame(link, &frame);
}

static void SendNackSeqLost(proto_link_t link, uint8_t command, uint8_t expected_seq) {
    ProtoFrame frame;
    Protocol_MakeNackSeqLost(&frame, command, expected_seq);
    SendFrame(link, &frame);
}

static uint8_t CurrentOwner(uint32_t now) {
    MotionStatus status;
    MotionControl_GetStatus(&status, now);
    return status.owner;
}

static void ReplyGimbalMotion(proto_link_t link, uint8_t command, uint8_t seq, const MotionOutcome* outcome) {
    switch (outcome->result) {
    case MOTION_RESULT_ACCEPTED:
    case MOTION_RESULT_DUPLICATE:
        SendAck(link, command, seq, outcome->ack_flags);
        break;

    case MOTION_RESULT_BUSY:
        SendNackBusy(link, command, seq, outcome->owner);
        break;

    case MOTION_RESULT_SEQ_LOST:
        SendNackSeqLost(link, command, outcome->expected_seq);
        break;

    default:
        CommandRouter_SendError(link, PROTO_ERROR_STATE, command);
        break;
    }
}

static void ReplyVelocity(proto_link_t link, uint8_t seq, const MotionOutcome* outcome) {
    ProtoFrame frame;

    switch (outcome->result) {
    case MOTION_RESULT_ACCEPTED:
    case MOTION_RESULT_DUPLICATE:
        Protocol_MakeAckMotion(&frame, seq, outcome->motion_state, outcome->ack_flags);
        SendFrame(link, &frame);
        break;

    case MOTION_RESULT_BUSY:
        /* A rejected velocity keeps the current owner and sequence; the reply
         * is an ACK_MOTION with motion_state BUSY_REJECT, not NACK_BUSY. */
        Protocol_MakeAckMotion(&frame, seq, PROTO_MOTION_BUSY_REJECT, 0u);
        SendFrame(link, &frame);
        break;

    case MOTION_RESULT_SEQ_LOST:
        SendNackSeqLost(link, PROTO_CMD_SET_VELOCITY, outcome->expected_seq);
        break;

    default:
        CommandRouter_SendError(link, PROTO_ERROR_STATE, PROTO_CMD_SET_VELOCITY);
        break;
    }
}

/**
 * A gimbal command is gated only by the mutual-exclusion contract: no
 * emergency stop, no chassis owner, and a chassis that has fully stopped.
 */
static bool GimbalMotionAllowed(proto_link_t link, uint8_t command, uint8_t seq, uint32_t now) {
    MotionStatus status;
    MotionControl_GetStatus(&status, now);

    if ((status.flags & PROTO_STATUS_EMERGENCY) != 0u) {
        CommandRouter_SendError(link, PROTO_ERROR_STATE, command);
        return false;
    }
    if (status.owner == PROTO_OWNER_CHASSIS ||
        (status.flags & PROTO_STATUS_CHASSIS_STOPPED) == 0u) {
        SendNackBusy(link, command, seq, status.owner);
        return false;
    }
    return true;
}

static void HandleSetTarget(proto_link_t link, const ProtoFrame* frame, uint32_t now) {
    ProtoSetTarget message;

    if (!Protocol_ParseSetTarget(frame, &message)) {
        CommandRouter_SendError(link, PROTO_ERROR_FRAME, frame->cmd);
        return;
    }
    if (!GimbalMotionAllowed(link, frame->cmd, message.seq, now)) {
        return;
    }

    MotionOutcome outcome = MotionControl_SetTarget(message.seq, message.pan, message.tilt, now);
    ReplyGimbalMotion(link, frame->cmd, message.seq, &outcome);
}

static void HandleSetDelta(proto_link_t link, const ProtoFrame* frame, uint32_t now) {
    ProtoSetDelta message;

    if (!Protocol_ParseSetDelta(frame, &message)) {
        CommandRouter_SendError(link, PROTO_ERROR_FRAME, frame->cmd);
        return;
    }
    if (!GimbalMotionAllowed(link, frame->cmd, message.seq, now)) {
        return;
    }

    MotionOutcome outcome = MotionControl_SetDelta(message.seq, message.delta_pan, message.delta_tilt, now);
    ReplyGimbalMotion(link, frame->cmd, message.seq, &outcome);
}

static void HandleSetTargetStream(proto_link_t link, const ProtoFrame* frame, uint32_t now) {
    ProtoSetTargetStream message;

    if (!Protocol_ParseSetTargetStream(frame, &message)) {
        CommandRouter_SendError(link, PROTO_ERROR_FRAME, frame->cmd);
        return;
    }
    if (!GimbalMotionAllowed(link, frame->cmd, message.seq, now)) {
        return;
    }

    GimbalPoint points[PROTO_STREAM_MAX];
    for (uint8_t i = 0u; i < message.count; i++) {
        points[i].pan = message.pan[i];
        points[i].tilt = message.tilt[i];
    }

    MotionOutcome outcome = MotionControl_SetPath(message.seq, points, message.count, now);
    ReplyGimbalMotion(link, frame->cmd, message.seq, &outcome);
}

static void HandleSetVelocity(proto_link_t link, const ProtoFrame* frame, uint32_t now) {
    ProtoSetVelocity message;

    if (!Protocol_ParseSetVelocity(frame, &message)) {
        CommandRouter_SendError(link, PROTO_ERROR_FRAME, frame->cmd);
        return;
    }

    MotionOutcome outcome =
        MotionControl_SetVelocity(message.seq, message.velocity_x, message.velocity_y, message.omega, now);
    ReplyVelocity(link, message.seq, &outcome);
}

static void HandleSetMode(proto_link_t link, const ProtoFrame* frame, uint32_t now) {
    uint8_t mode;

    if (!Protocol_ParseSetMode(frame, &mode)) {
        CommandRouter_SendError(link, PROTO_ERROR_FRAME, frame->cmd);
        return;
    }

    if (mode == PROTO_MODE_GIMBAL_TRACKING && link != PROTO_LINK_PI) {
        CommandRouter_SendError(link, PROTO_ERROR_WRONG_LINK, frame->cmd);
        return;
    }

    if (mode == PROTO_MODE_EMERGENCY_STOP) {
        MotionControl_EmergencyStop();
        Telemetry_SendStatuses(now);
        return;
    }

    arbiter_mode_t result = MotionControl_SetMode(link, mode, now);
    switch (result) {
    case ARBITER_MODE_OK:
        SendAck(link, frame->cmd, 0u, PROTO_ACK_ACCEPTED);
        break;

    case ARBITER_MODE_BUSY:
        SendNackBusy(link, frame->cmd, 0u, CurrentOwner(now));
        break;

    case ARBITER_MODE_DENIED:
        CommandRouter_SendError(link, PROTO_ERROR_STATE, frame->cmd);
        break;

    default:
        CommandRouter_SendError(link, PROTO_ERROR_FRAME, frame->cmd);
        break;
    }
}

static bool CommandAllowedOnLink(proto_link_t link, uint8_t command) {
    if (link == PROTO_LINK_PI) {
        return command == PROTO_CMD_SET_TARGET || command == PROTO_CMD_SET_DELTA ||
               command == PROTO_CMD_SET_TARGET_STREAM || command == PROTO_CMD_SET_MODE ||
               command == PROTO_CMD_HEARTBEAT;
    }
    if (link == PROTO_LINK_BT) {
        return command == PROTO_CMD_SET_VELOCITY || command == PROTO_CMD_SET_MODE ||
               command == PROTO_CMD_HEARTBEAT;
    }
    return false;
}

/* Reserved and undefined command codes are errors regardless of the link; only
 * a known command sent to the wrong link is WRONG_LINK. */
static bool CommandKnown(uint8_t command) {
    switch (command) {
    case PROTO_CMD_SET_TARGET:
    case PROTO_CMD_SET_DELTA:
    case PROTO_CMD_SET_VELOCITY:
    case PROTO_CMD_SET_MODE:
    case PROTO_CMD_SET_TARGET_STREAM:
    case PROTO_CMD_HEARTBEAT:
        return true;
    default:
        return false;
    }
}

void CommandRouter_HandleFrame(proto_link_t link, const ProtoFrame* frame, uint32_t now) {
    if (frame == NULL) {
        return;
    }
    if (!CommandKnown(frame->cmd)) {
        CommandRouter_SendError(link, PROTO_ERROR_UNKNOWN_CMD, frame->cmd);
        return;
    }
    if (!CommandAllowedOnLink(link, frame->cmd)) {
        CommandRouter_SendError(link, PROTO_ERROR_WRONG_LINK, frame->cmd);
        return;
    }

    switch (frame->cmd) {
    case PROTO_CMD_HEARTBEAT:
        if (frame->data_len != 0u) {
            CommandRouter_SendError(link, PROTO_ERROR_FRAME, frame->cmd);
            break;
        }
        MotionControl_NoteHeartbeat(link, now);
        break;

    case PROTO_CMD_SET_TARGET:
        HandleSetTarget(link, frame, now);
        break;

    case PROTO_CMD_SET_DELTA:
        HandleSetDelta(link, frame, now);
        break;

    case PROTO_CMD_SET_TARGET_STREAM:
        HandleSetTargetStream(link, frame, now);
        break;

    case PROTO_CMD_SET_VELOCITY:
        HandleSetVelocity(link, frame, now);
        break;

    case PROTO_CMD_SET_MODE:
        HandleSetMode(link, frame, now);
        break;

    default:
        CommandRouter_SendError(link, PROTO_ERROR_UNKNOWN_CMD, frame->cmd);
        break;
    }
}
