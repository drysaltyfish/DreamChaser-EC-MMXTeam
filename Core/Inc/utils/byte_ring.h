#ifndef BYTE_RING_H
#define BYTE_RING_H

#include <stdbool.h>
#include <stdint.h>

/**
 * Single-producer/single-consumer byte ring.
 *
 * The capacity must be a power of two. One slot is always kept free, so the
 * usable capacity is capacity - 1 bytes.
 *
 * Push may be called from an ISR when the consumer is a task; Pop must stay in
 * exactly one consumer context. head is owned by the producer, tail by the
 * consumer, which makes the index exchange safe without a critical section.
 */
typedef struct {
    uint8_t* buffer;
    uint16_t capacity;
    uint16_t mask;
    volatile uint16_t head;
    volatile uint16_t tail;
} ByteRing;

void ByteRing_Init(ByteRing* ring, uint8_t* buffer, uint16_t capacity);

bool ByteRing_Push(ByteRing* ring, uint8_t byte);

/** Pushes as many bytes as fit and returns how many were accepted. */
uint16_t ByteRing_PushBlock(ByteRing* ring, const uint8_t* data, uint16_t length);

/** Pops at most length bytes and returns how many were copied. */
uint16_t ByteRing_PopBlock(ByteRing* ring, uint8_t* data, uint16_t length);

uint16_t ByteRing_Count(const ByteRing* ring);

bool ByteRing_IsEmpty(const ByteRing* ring);

#endif /* BYTE_RING_H */
