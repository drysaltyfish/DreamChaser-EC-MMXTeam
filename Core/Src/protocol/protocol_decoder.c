#include "protocol/protocol_decoder.h"

#include <stddef.h>
#include <string.h>

#include "utils/crc.h"

static void ProtoDecoder_ResetFrame(ProtoDecoder* decoder, proto_decoder_state_t state) {
    decoder->state = state;
    decoder->body_length = 0u;
    decoder->body_expected = 0u;
    decoder->crc_received = 0u;
    decoder->escaped = false;
}

static bool ProtoDecoder_FinishFrame(ProtoDecoder* decoder) {
    uint16_t crc = Crc16Modbus(decoder->body, decoder->body_length);

    if (crc != decoder->crc_received) {
        decoder->crc_errors++;
        if (decoder->consecutive_crc_errors < 5u) {
            decoder->consecutive_crc_errors++;
            if (decoder->consecutive_crc_errors == 5u) {
                /* Report the threshold once; a valid frame resets the count. */
                decoder->crc_alarm = true;
            }
        }
        ProtoDecoder_ResetFrame(decoder, PROTO_DECODER_WAIT_SOF);
        return false;
    }

    uint8_t length = decoder->body[0];
    decoder->frame.cmd = decoder->body[1];
    decoder->frame.data_len = (uint8_t)(length - 1u);
    if (decoder->frame.data_len > 0u) {
        memcpy(decoder->frame.data, &decoder->body[2], decoder->frame.data_len);
    }

    decoder->consecutive_crc_errors = 0u;
    decoder->frame_ready = true;
    ProtoDecoder_ResetFrame(decoder, PROTO_DECODER_WAIT_SOF);
    return true;
}

static bool ProtoDecoder_FeedBody(ProtoDecoder* decoder, uint8_t byte) {
    switch (decoder->state) {
    case PROTO_DECODER_LENGTH:
        if (byte == 0u || byte > PROTO_LEN_MAX) {
            decoder->format_errors++;
            ProtoDecoder_ResetFrame(decoder, PROTO_DECODER_WAIT_SOF);
            return false;
        }
        decoder->body[0] = byte;
        decoder->body_length = 1u;
        /* LEN counts CMD plus DATA; the body also stores the LEN byte. */
        decoder->body_expected = (uint8_t)(byte + 1u);
        decoder->state = PROTO_DECODER_BODY;
        return false;

    case PROTO_DECODER_BODY:
        decoder->body[decoder->body_length] = byte;
        decoder->body_length++;
        if (decoder->body_length >= decoder->body_expected) {
            decoder->state = PROTO_DECODER_CRC_LOW;
        }
        return false;

    case PROTO_DECODER_CRC_LOW:
        decoder->crc_received = byte;
        decoder->state = PROTO_DECODER_CRC_HIGH;
        return false;

    case PROTO_DECODER_CRC_HIGH:
        decoder->crc_received |= (uint16_t)((uint16_t)byte << 8u);
        decoder->state = PROTO_DECODER_EOF;
        return false;

    case PROTO_DECODER_EOF:
        if (byte != PROTO_EOF) {
            decoder->format_errors++;
            ProtoDecoder_ResetFrame(decoder, PROTO_DECODER_WAIT_SOF);
            return false;
        }
        return ProtoDecoder_FinishFrame(decoder);

    default:
        ProtoDecoder_ResetFrame(decoder, PROTO_DECODER_WAIT_SOF);
        return false;
    }
}

void ProtocolDecoder_Init(ProtoDecoder* decoder) {
    memset(decoder, 0, sizeof(*decoder));
    decoder->state = PROTO_DECODER_WAIT_SOF;
}

bool ProtocolDecoder_Feed(ProtoDecoder* decoder, uint8_t byte) {
    if (decoder->state == PROTO_DECODER_WAIT_SOF) {
        if (byte == PROTO_SOF) {
            decoder->state = PROTO_DECODER_WAIT_SOF_SECOND;
        }
        return false;
    }

    if (decoder->state == PROTO_DECODER_WAIT_SOF_SECOND) {
        if (byte == PROTO_SOF) {
            return false;
        }
        decoder->state = (byte == PROTO_SOF_SECOND) ? PROTO_DECODER_LENGTH : PROTO_DECODER_WAIT_SOF;
        return false;
    }

    if (decoder->escaped) {
        decoder->escaped = false;
        if (byte == PROTO_ESCAPE_SECOND) {
            return ProtoDecoder_FeedBody(decoder, PROTO_SOF);
        }
        if (byte == PROTO_SOF_SECOND) {
            /* AA 55 inside a frame is a new start of frame: drop the current
             * frame and resume at the LEN state, as if the SOF had just been
             * consumed. */
            ProtoDecoder_ResetFrame(decoder, PROTO_DECODER_LENGTH);
            return false;
        }
        decoder->format_errors++;
        ProtoDecoder_ResetFrame(decoder, PROTO_DECODER_WAIT_SOF);
        return false;
    }

    if (byte == PROTO_SOF) {
        /* Either an escape pair or the beginning of a new frame. The next byte
         * decides; until then the current frame is kept so that a lone 0xAA in
         * the body is handled as data once the escape byte arrives. */
        decoder->escaped = true;
        return false;
    }

    return ProtoDecoder_FeedBody(decoder, byte);
}

bool ProtocolDecoder_TakeFrame(ProtoDecoder* decoder, ProtoFrame* frame) {
    if (!decoder->frame_ready) {
        return false;
    }

    *frame = decoder->frame;
    decoder->frame_ready = false;
    return true;
}

bool ProtocolDecoder_TakeCrcAlarm(ProtoDecoder* decoder) {
    bool alarm = decoder->crc_alarm;
    decoder->crc_alarm = false;
    return alarm;
}
