#ifndef CRC_H
#define CRC_H

#include <stddef.h>
#include <stdint.h>

/** CRC-16/MODBUS: poly 0xA001 reflected, init 0xFFFF, no final xor. */
uint16_t Crc16Modbus(const uint8_t* data, size_t length);

/** Continues a CRC-16/MODBUS calculation over another block. */
uint16_t Crc16Modbus_Update(uint16_t crc, const uint8_t* data, size_t length);

#endif /* CRC_H */
