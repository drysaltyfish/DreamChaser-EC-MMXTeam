#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

/**
 * Periodic status reporting.
 *
 * STATUS is sent once per second and immediately when the arbitration state
 * changes.
 */
void Telemetry_Task(void* argument);

void Telemetry_SendStatus(uint32_t now);

void Telemetry_SendReady(void);

#endif /* TELEMETRY_H */
