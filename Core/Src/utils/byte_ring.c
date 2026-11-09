#include "utils/byte_ring.h"

#include <stddef.h>

void ByteRing_Init(ByteRing* ring, uint8_t* buffer, uint16_t capacity) {
    ring->buffer = buffer;
    ring->capacity = capacity;
    ring->mask = (uint16_t)(capacity - 1u);
    ring->head = 0u;
    ring->tail = 0u;
}

bool ByteRing_Push(ByteRing* ring, uint8_t byte) {
    uint16_t head = ring->head;
    uint16_t next = (uint16_t)((head + 1u) & ring->mask);

    if (next == ring->tail) {
        return false;
    }

    ring->buffer[head] = byte;
    ring->head = next;
    return true;
}

uint16_t ByteRing_PushBlock(ByteRing* ring, const uint8_t* data, uint16_t length) {
    uint16_t accepted = 0u;

    while (accepted < length && ByteRing_Push(ring, data[accepted])) {
        accepted++;
    }

    return accepted;
}

uint16_t ByteRing_PopBlock(ByteRing* ring, uint8_t* data, uint16_t length) {
    uint16_t copied = 0u;

    while (copied < length && ring->tail != ring->head) {
        data[copied] = ring->buffer[ring->tail];
        ring->tail = (uint16_t)((ring->tail + 1u) & ring->mask);
        copied++;
    }

    return copied;
}

uint16_t ByteRing_Count(const ByteRing* ring) {
    return (uint16_t)((ring->head - ring->tail) & ring->mask);
}

bool ByteRing_IsEmpty(const ByteRing* ring) {
    return ring->head == ring->tail;
}
