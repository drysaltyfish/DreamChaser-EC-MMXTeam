#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

/**
 * Periodic status reporting and the low priority flash commit.
 *
 * STATUS is sent once per second and immediately when the arbitration state
 * changes. Flash programming runs here so the control and command tasks are
 * never blocked by an erase.
 */
void Telemetry_Task(void* argument);

void Telemetry_SendStatus(uint32_t now);

void Telemetry_SendReady(void);

/** Queues a calibration commit; processed by Telemetry_Task. */
void Telemetry_RequestCommit(void);

#endif /* TELEMETRY_H */
