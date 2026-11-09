#include "utils/crc.h"

uint16_t Crc16Modbus_Update(uint16_t crc, const uint8_t* data, size_t length) {
    for (size_t i = 0u; i < length; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t bit = 0u; bit < 8u; bit++) {
            if ((crc & 0x0001u) != 0u) {
                crc = (uint16_t)((crc >> 1u) ^ 0xA001u);
            } else {
                crc = (uint16_t)(crc >> 1u);
            }
        }
    }

    return crc;
}

uint16_t Crc16Modbus(const uint8_t* data, size_t length) {
    return Crc16Modbus_Update(0xFFFFu, data, length);
}
