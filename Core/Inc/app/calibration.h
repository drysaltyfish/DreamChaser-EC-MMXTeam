#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool valid;
    uint32_t sequence;
    int16_t pan_zero;
    int16_t tilt_zero;
    int16_t pan_min;
    int16_t pan_max;
    int16_t tilt_min;
    int16_t tilt_max;
} CalibrationData;

typedef enum {
    CALIBRATION_COMMIT_NONE = 0,
    CALIBRATION_COMMIT_OK,
    CALIBRATION_COMMIT_FAILED
} calibration_commit_t;

/**
 * Loads the newest valid record from the two flash banks.
 *
 * The record with the highest sequence number and a matching CRC wins. When
 * neither bank is valid the defaults from app_config are used and the data is
 * reported invalid, so the pi can run the calibration procedure.
 */
void Calibration_Init(void);

bool Calibration_IsValid(void);

/** Copies a consistent snapshot of the current calibration data. */
void Calibration_GetSnapshot(CalibrationData* data);

void Calibration_SetPanZero(int16_t value);
void Calibration_SetTiltZero(int16_t value);
void Calibration_SetLimits(int16_t pan_min, int16_t pan_max, int16_t tilt_min, int16_t tilt_max);

/**
 * Queues a flash commit. The actual erase/program must run from a low
 * priority task, never from the command or control path.
 */
void Calibration_RequestCommit(void);

/** Performs a queued commit. Call from the storage task. */
calibration_commit_t Calibration_ProcessCommit(void);

#endif /* CALIBRATION_H */
