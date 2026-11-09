#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdbool.h>
#include <stdint.h>

#define GIMBAL_PATH_MAX 10u

typedef struct {
    int16_t pan_min;
    int16_t pan_max;
    int16_t tilt_min;
    int16_t tilt_max;
    uint16_t max_speed_lsb_per_s;
    uint16_t arrival_tolerance_lsb;
    uint16_t max_delta_lsb;
} GimbalConfig;

typedef struct {
    int16_t pan;
    int16_t tilt;
} GimbalPoint;

/**
 * Two-axis gimbal model in 0.01 degree units.
 *
 * Commands move the target; the actual position approaches it at a limited
 * speed. A path is a short list of waypoints executed in order.
 */
typedef struct {
    GimbalConfig config;
    int16_t actual_pan;
    int16_t actual_tilt;
    int16_t target_pan;
    int16_t target_tilt;
    GimbalPoint path[GIMBAL_PATH_MAX];
    uint8_t path_count;
    uint8_t path_index;
    bool path_active;
    bool enabled;
} GimbalState;

void Gimbal_Init(GimbalState* gimbal, const GimbalConfig* config);

void Gimbal_Enable(GimbalState* gimbal);
void Gimbal_Disable(GimbalState* gimbal);

/** Returns true when the requested angle had to be clamped. */
bool Gimbal_SetTarget(GimbalState* gimbal, int16_t pan, int16_t tilt);

/** Adds a delta to the current target position; returns true when clamped. */
bool Gimbal_AddDelta(GimbalState* gimbal, int16_t delta_pan, int16_t delta_tilt);

/** Replaces the active path and starts at its first waypoint. */
bool Gimbal_SetPath(GimbalState* gimbal, const GimbalPoint* points, uint8_t count);

void Gimbal_Step(GimbalState* gimbal, uint32_t elapsed_ms);

bool Gimbal_AtTarget(const GimbalState* gimbal);

void Gimbal_GetActual(const GimbalState* gimbal, int16_t* pan, int16_t* tilt);

#endif /* GIMBAL_H */
