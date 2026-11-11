#ifndef COMMAND_ROUTER_H
#define COMMAND_ROUTER_H

#include <stdint.h>

#include "protocol/protocol_defs.h"

/**
 * Dispatches one decoded command frame from one physical link.
 *
 * All replies go back to the link that carried the command, through its
 * transmit queue. This function runs only in the command task.
 */
void CommandRouter_HandleFrame(proto_link_t link, const ProtoFrame* frame, uint32_t now);

/** Queues an ERROR frame on one link. Safe to call from any task. */
void CommandRouter_SendError(proto_link_t link, uint8_t code, uint8_t detail);

#endif /* COMMAND_ROUTER_H */
