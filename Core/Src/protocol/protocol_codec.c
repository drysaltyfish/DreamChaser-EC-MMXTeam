#include "protocol/protocol_codec.h"

#include <stdbool.h>

#include "utils/crc.h"

typedef struct {
    uint8_t* out;
    size_t capacity;
    size_t length;
    bool ok;
} WireWriter;

static void WireWriter_Put(WireWriter* writer, uint8_t byte) {
    if (writer->length >= writer->capacity) {
        writer->ok = false;
        return;
    }

    writer->out[writer->length] = byte;
    writer->length++;
}

static void WireWriter_PutEscaped(WireWriter* writer, uint8_t byte) {
    WireWriter_Put(writer, byte);

    if (byte == PROTO_SOF) {
        WireWriter_Put(writer, PROTO_ESCAPE_SECOND);
    }
}

size_t Protocol_EncodeFrame(uint8_t cmd, const uint8_t* data, uint8_t data_len, uint8_t* out, size_t out_capacity) {
    if (out == NULL || (data == NULL && data_len > 0u) || data_len > PROTO_DATA_MAX) {
        return 0u;
    }

    WireWriter writer = {.out = out, .capacity = out_capacity, .length = 0u, .ok = true};

    uint8_t length = (uint8_t)(data_len + 1u);
    uint16_t crc = Crc16Modbus_Update(0xFFFFu, &length, 1u);
    crc = Crc16Modbus_Update(crc, &cmd, 1u);
    crc = Crc16Modbus_Update(crc, data, data_len);

    WireWriter_Put(&writer, PROTO_SOF);
    WireWriter_Put(&writer, PROTO_SOF_SECOND);
    WireWriter_PutEscaped(&writer, length);
    WireWriter_PutEscaped(&writer, cmd);

    for (uint8_t i = 0u; i < data_len; i++) {
        WireWriter_PutEscaped(&writer, data[i]);
    }

    WireWriter_PutEscaped(&writer, (uint8_t)(crc & 0xFFu));
    WireWriter_PutEscaped(&writer, (uint8_t)(crc >> 8u));
    WireWriter_Put(&writer, PROTO_EOF);

    return writer.ok ? writer.length : 0u;
}

size_t Protocol_EncodeProtoFrame(const ProtoFrame* frame, uint8_t* out, size_t out_capacity) {
    if (frame == NULL) {
        return 0u;
    }

    return Protocol_EncodeFrame(frame->cmd, frame->data, frame->data_len, out, out_capacity);
}
