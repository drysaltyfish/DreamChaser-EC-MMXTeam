#ifndef PROTOCOL_MESSAGES_H
#define PROTOCOL_MESSAGES_H

#include <stdbool.h>
#include <stdint.h>

#include "protocol/protocol_defs.h"

typedef struct {
    uint8_t seq;
    uint8_t coord_type;
    int16_t pan;
    int16_t tilt;
} ProtoSetTarget;

typedef struct {
    uint8_t seq;
    int16_t delta_pan;
    int16_t delta_tilt;
} ProtoSetDelta;

typedef struct {
    uint8_t seq;
    int16_t velocity_x;
    int16_t velocity_y;
    int16_t omega;
} ProtoSetVelocity;

typedef struct {
    uint8_t seq;
    uint8_t coord_type;
    uint8_t count;
    int16_t pan[PROTO_STREAM_MAX];
    int16_t tilt[PROTO_STREAM_MAX];
} ProtoSetTargetStream;

typedef struct {
    uint8_t gimbal_state;
    uint8_t owner;
    uint8_t motion_state;
    int16_t pan;
    int16_t tilt;
    uint16_t error_count;
    uint8_t flags;
} ProtoStatus;

typedef struct {
    uint8_t version_major;
    uint8_t version_minor;
    uint8_t flags;
} ProtoReady;

bool Protocol_ParseSetTarget(const ProtoFrame* frame, ProtoSetTarget* message);
bool Protocol_ParseSetDelta(const ProtoFrame* frame, ProtoSetDelta* message);
bool Protocol_ParseSetVelocity(const ProtoFrame* frame, ProtoSetVelocity* message);
bool Protocol_ParseSetMode(const ProtoFrame* frame, uint8_t* mode);
bool Protocol_ParseSetTargetStream(const ProtoFrame* frame, ProtoSetTargetStream* message);

void Protocol_MakeAck(ProtoFrame* frame, uint8_t command, uint8_t seq, uint8_t flags);
void Protocol_MakeAckMotion(ProtoFrame* frame, uint8_t seq, uint8_t motion_state, uint8_t flags);
void Protocol_MakeNackBusy(ProtoFrame* frame, uint8_t command, uint8_t seq, uint8_t owner);
void Protocol_MakeNackSeqLost(ProtoFrame* frame, uint8_t command, uint8_t expected_seq);
void Protocol_MakeError(ProtoFrame* frame, uint8_t code, uint8_t detail);
void Protocol_MakeStatus(ProtoFrame* frame, const ProtoStatus* status);
void Protocol_MakeReady(ProtoFrame* frame, const ProtoReady* ready);

#endif /* PROTOCOL_MESSAGES_H */
