#ifndef PROTOCOL_DECODER_H
#define PROTOCOL_DECODER_H

#include <stdbool.h>
#include <stdint.h>

#include "protocol/protocol_defs.h"

typedef enum {
    PROTO_DECODER_WAIT_SOF,
    PROTO_DECODER_WAIT_SOF_SECOND,
    PROTO_DECODER_LENGTH,
    PROTO_DECODER_BODY,
    PROTO_DECODER_CRC_LOW,
    PROTO_DECODER_CRC_HIGH,
    PROTO_DECODER_EOF
} proto_decoder_state_t;

typedef struct {
    proto_decoder_state_t state;
    uint8_t body[PROTO_BODY_MAX];
    uint8_t body_length;
    uint8_t body_expected;
    uint16_t crc_received;
    bool escaped;
    bool frame_ready;
    bool crc_alarm;
    ProtoFrame frame;
    uint8_t consecutive_crc_errors;
    uint32_t crc_errors;
    uint32_t format_errors;
    uint32_t dropped_frames;
} ProtoDecoder;

void ProtocolDecoder_Init(ProtoDecoder* decoder);

/**
 * Feeds one wire byte.
 *
 * Returns true when a frame has just completed; the caller must then call
 * ProtocolDecoder_TakeFrame before feeding more bytes. A frame is never queued.
 */
bool ProtocolDecoder_Feed(ProtoDecoder* decoder, uint8_t byte);

/** Copies the completed frame out and clears the pending flag. */
bool ProtocolDecoder_TakeFrame(ProtoDecoder* decoder, ProtoFrame* frame);

/**
 * Returns true once after five consecutive CRC failures. Reading clears it;
 * any valid frame also clears the consecutive counter.
 */
bool ProtocolDecoder_TakeCrcAlarm(ProtoDecoder* decoder);

#endif /* PROTOCOL_DECODER_H */
