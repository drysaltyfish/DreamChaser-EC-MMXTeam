#include "motion/gimbal.h"

#include <stddef.h>

static int16_t ClampAngle(int16_t value, int16_t minimum, int16_t maximum) {
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int16_t ClampDelta(int16_t value, uint16_t limit) {
    if (value > (int16_t)limit) {
        return (int16_t)limit;
    }
    if (value < (int16_t)(-limit)) {
        return (int16_t)(-limit);
    }
    return value;
}

static int16_t MoveTowards(int16_t actual, int16_t target, int32_t max_step) {
    int32_t difference = (int32_t)target - (int32_t)actual;

    if (difference > max_step) {
        difference = max_step;
    } else if (difference < -max_step) {
        difference = -max_step;
    }

    return (int16_t)((int32_t)actual + difference);
}

static bool AxisAtTarget(int16_t actual, int16_t target, uint16_t tolerance) {
    int32_t difference = (int32_t)actual - (int32_t)target;
    if (difference < 0) {
        difference = -difference;
    }
    return difference <= (int32_t)tolerance;
}

void Gimbal_Init(GimbalState* gimbal, const GimbalConfig* config) {
    *gimbal = (GimbalState){0};
    gimbal->config = *config;
    gimbal->enabled = false;
}

void Gimbal_Enable(GimbalState* gimbal) {
    if (gimbal->enabled) {
        return;
    }

    /* Entering tracking starts from the present position; an already enabled
     * gimbal keeps its target so consecutive deltas accumulate. */
    gimbal->target_pan = gimbal->actual_pan;
    gimbal->target_tilt = gimbal->actual_tilt;
    gimbal->path_active = false;
    gimbal->path_count = 0u;
    gimbal->path_index = 0u;
    gimbal->enabled = true;
}

void Gimbal_Disable(GimbalState* gimbal) {
    gimbal->target_pan = gimbal->actual_pan;
    gimbal->target_tilt = gimbal->actual_tilt;
    gimbal->path_active = false;
    gimbal->path_count = 0u;
    gimbal->path_index = 0u;
    gimbal->enabled = false;
}

bool Gimbal_SetTarget(GimbalState* gimbal, int16_t pan, int16_t tilt) {
    int16_t clamped_pan = ClampAngle(pan, gimbal->config.pan_min, gimbal->config.pan_max);
    int16_t clamped_tilt = ClampAngle(tilt, gimbal->config.tilt_min, gimbal->config.tilt_max);

    gimbal->path_active = false;
    gimbal->path_count = 0u;
    gimbal->path_index = 0u;
    gimbal->target_pan = clamped_pan;
    gimbal->target_tilt = clamped_tilt;
    return (clamped_pan != pan) || (clamped_tilt != tilt);
}

bool Gimbal_AddDelta(GimbalState* gimbal, int16_t delta_pan, int16_t delta_tilt) {
    int16_t limited_pan = ClampDelta(delta_pan, gimbal->config.max_delta_lsb);
    int16_t limited_tilt = ClampDelta(delta_tilt, gimbal->config.max_delta_lsb);

    int32_t pan = (int32_t)gimbal->target_pan + limited_pan;
    int32_t tilt = (int32_t)gimbal->target_tilt + limited_tilt;
    int16_t clamped_pan = ClampAngle((int16_t)pan, gimbal->config.pan_min, gimbal->config.pan_max);
    int16_t clamped_tilt = ClampAngle((int16_t)tilt, gimbal->config.tilt_min, gimbal->config.tilt_max);

    gimbal->path_active = false;
    gimbal->path_count = 0u;
    gimbal->path_index = 0u;
    gimbal->target_pan = clamped_pan;
    gimbal->target_tilt = clamped_tilt;

    return (limited_pan != delta_pan) || (limited_tilt != delta_tilt) || (clamped_pan != (int16_t)pan) ||
           (clamped_tilt != (int16_t)tilt);
}

bool Gimbal_SetPath(GimbalState* gimbal, const GimbalPoint* points, uint8_t count) {
    if (points == NULL || count == 0u || count > GIMBAL_PATH_MAX) {
        return false;
    }

    bool clamped = false;
    for (uint8_t i = 0u; i < count; i++) {
        int16_t pan = ClampAngle(points[i].pan, gimbal->config.pan_min, gimbal->config.pan_max);
        int16_t tilt = ClampAngle(points[i].tilt, gimbal->config.tilt_min, gimbal->config.tilt_max);
        clamped = clamped || (pan != points[i].pan) || (tilt != points[i].tilt);
        gimbal->path[i].pan = pan;
        gimbal->path[i].tilt = tilt;
    }

    gimbal->path_count = count;
    gimbal->path_index = 0u;
    gimbal->path_active = true;
    gimbal->target_pan = gimbal->path[0].pan;
    gimbal->target_tilt = gimbal->path[0].tilt;
    return clamped;
}

static bool Gimbal_TargetReached(const GimbalState* gimbal) {
    return AxisAtTarget(gimbal->actual_pan, gimbal->target_pan, gimbal->config.arrival_tolerance_lsb) &&
           AxisAtTarget(gimbal->actual_tilt, gimbal->target_tilt, gimbal->config.arrival_tolerance_lsb);
}

void Gimbal_Step(GimbalState* gimbal, uint32_t elapsed_ms) {
    if (!gimbal->enabled) {
        return;
    }

    if (gimbal->path_active && Gimbal_TargetReached(gimbal)) {
        gimbal->path_index++;
        if (gimbal->path_index >= gimbal->path_count) {
            gimbal->path_active = false;
        } else {
            gimbal->target_pan = gimbal->path[gimbal->path_index].pan;
            gimbal->target_tilt = gimbal->path[gimbal->path_index].tilt;
        }
        return;
    }

    int32_t max_step = (int32_t)(((uint32_t)gimbal->config.max_speed_lsb_per_s * elapsed_ms) / 1000u);
    if (max_step <= 0) {
        return;
    }

    gimbal->actual_pan = MoveTowards(gimbal->actual_pan, gimbal->target_pan, max_step);
    gimbal->actual_tilt = MoveTowards(gimbal->actual_tilt, gimbal->target_tilt, max_step);
}

bool Gimbal_AtTarget(const GimbalState* gimbal) {
    return !gimbal->path_active && Gimbal_TargetReached(gimbal);
}

void Gimbal_GetActual(const GimbalState* gimbal, int16_t* pan, int16_t* tilt) {
    if (pan != NULL) {
        *pan = gimbal->actual_pan;
    }
    if (tilt != NULL) {
        *tilt = gimbal->actual_tilt;
    }
}
