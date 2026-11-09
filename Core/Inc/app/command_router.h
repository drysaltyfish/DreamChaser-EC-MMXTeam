#ifndef COMMAND_ROUTER_H
#define COMMAND_ROUTER_H

#include <stdint.h>

#include "protocol/protocol_defs.h"

/**
 * Dispatches one decoded command frame.
 *
 * All replies go through the transmit queue, so this function never blocks on
 * the UART. It is called only from the command task.
 */
void CommandRouter_HandleFrame(const ProtoFrame* frame, uint32_t now);

/** Queues an ERROR frame. Safe to call from any task. */
void CommandRouter_SendError(uint8_t code, uint8_t detail);

#endif /* COMMAND_ROUTER_H */
