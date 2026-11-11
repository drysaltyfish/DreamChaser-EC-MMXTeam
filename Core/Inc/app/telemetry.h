#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

#include "protocol/protocol_defs.h"

/**
 * Periodic status reporting.
 *
 * The Pi link receives READY and GIMBAL_STATUS; the Bluetooth link receives
 * READY, CHASSIS_STATUS and MOTOR_STATUS. Both status frames are sent when the
 * arbitration state changes and periodically otherwise.
 */
void Telemetry_Task(void* argument);

/** Sends READY on one link. */
void Telemetry_SendReady(proto_link_t link);

/** Sends both status frames, for example right after an emergency stop. */
void Telemetry_SendStatuses(uint32_t now);

void Telemetry_SendGimbalStatus(uint32_t now);
void Telemetry_SendChassisStatus(uint32_t now);

#endif /* TELEMETRY_H */
