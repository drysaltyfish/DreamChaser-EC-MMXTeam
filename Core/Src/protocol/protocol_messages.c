#include "protocol/protocol_messages.h"

#include <stddef.h>

static int16_t ReadInt16Le(const uint8_t* data) {
    return (int16_t)((uint16_t)data[0] | (uint16_t)((uint16_t)data[1] << 8u));
}

static void WriteInt16Le(uint8_t* data, int16_t value) {
    uint16_t raw = (uint16_t)value;
    data[0] = (uint8_t)(raw & 0xFFu);
    data[1] = (uint8_t)(raw >> 8u);
}

static void WriteUint16Le(uint8_t* data, uint16_t value) {
    data[0] = (uint8_t)(value & 0xFFu);
    data[1] = (uint8_t)(value >> 8u);
}

bool Protocol_ParseSetTarget(const ProtoFrame* frame, ProtoSetTarget* message) {
    if (frame == NULL || message == NULL || frame->data_len != 6u) {
        return false;
    }
    if (frame->data[1] != PROTO_COORD_ANGLE_001DEG) {
        return false;
    }

    message->seq = frame->data[0];
    message->coord_type = frame->data[1];
    message->pan = ReadInt16Le(&frame->data[2]);
    message->tilt = ReadInt16Le(&frame->data[4]);
    return true;
}

bool Protocol_ParseSetDelta(const ProtoFrame* frame, ProtoSetDelta* message) {
    if (frame == NULL || message == NULL || frame->data_len != 5u) {
        return false;
    }

    message->seq = frame->data[0];
    message->delta_pan = ReadInt16Le(&frame->data[1]);
    message->delta_tilt = ReadInt16Le(&frame->data[3]);
    return true;
}

bool Protocol_ParseSetVelocity(const ProtoFrame* frame, ProtoSetVelocity* message) {
    if (frame == NULL || message == NULL || frame->data_len != 7u) {
        return false;
    }

    message->seq = frame->data[0];
    message->velocity_x = ReadInt16Le(&frame->data[1]);
    message->velocity_y = ReadInt16Le(&frame->data[3]);
    message->omega = ReadInt16Le(&frame->data[5]);
    return true;
}

bool Protocol_ParseSetMode(const ProtoFrame* frame, uint8_t* mode) {
    if (frame == NULL || mode == NULL || frame->data_len != 1u) {
        return false;
    }

    uint8_t value = frame->data[0];
    if (value != PROTO_MODE_RELEASE && value != PROTO_MODE_GIMBAL_TRACKING &&
        value != PROTO_MODE_EMERGENCY_STOP) {
        return false;
    }

    *mode = value;
    return true;
}

bool Protocol_ParseSetTargetStream(const ProtoFrame* frame, ProtoSetTargetStream* message) {
    if (frame == NULL || message == NULL || frame->data_len < 3u) {
        return false;
    }

    uint8_t count = frame->data[2];
    if (count == 0u || count > PROTO_STREAM_MAX) {
        return false;
    }
    if (frame->data[1] != PROTO_COORD_ANGLE_001DEG) {
        return false;
    }
    if (frame->data_len != (uint8_t)(3u + 4u * count)) {
        return false;
    }

    message->seq = frame->data[0];
    message->coord_type = frame->data[1];
    message->count = count;

    for (uint8_t i = 0u; i < count; i++) {
        const uint8_t* point = &frame->data[3u + 4u * i];
        message->pan[i] = ReadInt16Le(point);
        message->tilt[i] = ReadInt16Le(&point[2]);
    }

    return true;
}

static void PrepareFrame(ProtoFrame* frame, uint8_t command, uint8_t data_len) {
    frame->cmd = command;
    frame->data_len = data_len;
}

void Protocol_MakeAck(ProtoFrame* frame, uint8_t command, uint8_t seq, uint8_t flags) {
    PrepareFrame(frame, PROTO_CMD_ACK, 3u);
    frame->data[0] = command;
    frame->data[1] = seq;
    frame->data[2] = flags;
}

void Protocol_MakeAckMotion(ProtoFrame* frame, uint8_t seq, uint8_t motion_state, uint8_t flags) {
    PrepareFrame(frame, PROTO_CMD_ACK_MOTION, 3u);
    frame->data[0] = seq;
    frame->data[1] = motion_state;
    frame->data[2] = flags;
}

void Protocol_MakeNackBusy(ProtoFrame* frame, uint8_t command, uint8_t seq, uint8_t owner) {
    PrepareFrame(frame, PROTO_CMD_NACK_BUSY, 3u);
    frame->data[0] = command;
    frame->data[1] = seq;
    frame->data[2] = owner;
}

void Protocol_MakeNackSeqLost(ProtoFrame* frame, uint8_t command, uint8_t expected_seq) {
    PrepareFrame(frame, PROTO_CMD_NACK_SEQ_LOST, 2u);
    frame->data[0] = command;
    frame->data[1] = expected_seq;
}

void Protocol_MakeError(ProtoFrame* frame, uint8_t code, uint8_t detail) {
    PrepareFrame(frame, PROTO_CMD_ERROR, 2u);
    frame->data[0] = code;
    frame->data[1] = detail;
}

void Protocol_MakeGimbalStatus(ProtoFrame* frame, const ProtoGimbalStatus* status) {
    PrepareFrame(frame, PROTO_CMD_GIMBAL_STATUS, 9u);
    frame->data[0] = status->gimbal_state;
    frame->data[1] = status->owner;
    WriteInt16Le(&frame->data[2], status->pan);
    WriteInt16Le(&frame->data[4], status->tilt);
    WriteUint16Le(&frame->data[6], status->error_count);
    frame->data[8] = status->flags;
}

void Protocol_MakeChassisStatus(ProtoFrame* frame, const ProtoChassisStatus* status) {
    PrepareFrame(frame, PROTO_CMD_CHASSIS_STATUS, 11u);
    frame->data[0] = status->motion_state;
    frame->data[1] = status->owner;
    WriteInt16Le(&frame->data[2], status->velocity_x);
    WriteInt16Le(&frame->data[4], status->velocity_y);
    WriteInt16Le(&frame->data[6], status->omega);
    WriteUint16Le(&frame->data[8], status->error_count);
    frame->data[10] = status->flags;
}

void Protocol_MakeReady(ProtoFrame* frame, const ProtoReady* ready) {
    PrepareFrame(frame, PROTO_CMD_READY, 3u);
    frame->data[0] = ready->version_major;
    frame->data[1] = ready->version_minor;
    frame->data[2] = ready->flags;
}

void Protocol_MakeMotorStatus(ProtoFrame* frame, const ProtoMotorStatus* status) {
    PrepareFrame(frame, PROTO_CMD_MOTOR_STATUS, 1u + 2u * PROTO_MOTOR_COUNT);
    frame->data[0] = status->present_mask;
    for (uint32_t i = 0u; i < PROTO_MOTOR_COUNT; i++) {
        WriteInt16Le(&frame->data[1u + 2u * i], status->rpm[i]);
    }
}