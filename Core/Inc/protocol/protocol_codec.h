#ifndef PROTOCOL_CODEC_H
#define PROTOCOL_CODEC_H

#include <stddef.h>
#include <stdint.h>

#include "protocol/protocol_defs.h"

/**
 * Encodes one frame, escaping the body and the CRC.
 *
 * Returns the number of wire bytes written, or 0 when the arguments are
 * invalid or the destination buffer is too small.
 */
size_t Protocol_EncodeFrame(uint8_t cmd, const uint8_t* data, uint8_t data_len, uint8_t* out, size_t out_capacity);

/** Convenience wrapper for a ProtoFrame. */
size_t Protocol_EncodeProtoFrame(const ProtoFrame* frame, uint8_t* out, size_t out_capacity);

#endif /* PROTOCOL_CODEC_H */
