#include "motion/chassis.h"

#include <math.h>
#include <stddef.h>

static float ClampUnit(float value) {
    if (value > 1.0f) {
        return 1.0f;
    }
    if (value < -1.0f) {
        return -1.0f;
    }
    return value;
}

static float NormalizeInput(int16_t value) {
    float normalized = (float)value / 1000.0f;
    return ClampUnit(normalized);
}

static float MoveTowards(float value, float target, float max_delta) {
    float difference = target - value;

    if (difference > max_delta) {
        difference = max_delta;
    } else if (difference < -max_delta) {
        difference = -max_delta;
    }

    return value + difference;
}

static float ApplyDeadzone(float value, float deadzone) {
    return (fabsf(value) < deadzone) ? 0.0f : value;
}

void Chassis_Init(ChassisState* chassis, const ChassisConfig* config) {
    *chassis = (ChassisState){0};
    chassis->config = *config;
}

void Chassis_SetCommand(ChassisState* chassis, int16_t vx, int16_t vy, int16_t omega) {
    chassis->target_vx = ApplyDeadzone(NormalizeInput(vx), chassis->config.deadzone);
    chassis->target_vy = ApplyDeadzone(NormalizeInput(vy), chassis->config.deadzone);
    chassis->target_omega = ApplyDeadzone(NormalizeInput(omega), chassis->config.deadzone);
    chassis->emergency = false;
}

void Chassis_Stop(ChassisState* chassis) {
    chassis->target_vx = 0.0f;
    chassis->target_vy = 0.0f;
    chassis->target_omega = 0.0f;
    chassis->emergency = false;
}

void Chassis_EmergencyStop(ChassisState* chassis) {
    chassis->target_vx = 0.0f;
    chassis->target_vy = 0.0f;
    chassis->target_omega = 0.0f;
    chassis->emergency = true;
}

static float AxisAcceleration(const ChassisState* chassis, float current, float target) {
    if (chassis->emergency) {
        return chassis->config.emergency_decel_per_s;
    }
    if (target * current < 0.0f || fabsf(target) < fabsf(current)) {
        return chassis->config.decel_per_s;
    }
    return chassis->config.accel_per_s;
}

void Chassis_Step(ChassisState* chassis, uint32_t elapsed_ms) {
    float seconds = (float)elapsed_ms / 1000.0f;

    float vx_delta = AxisAcceleration(chassis, chassis->vx, chassis->target_vx) * seconds;
    float vy_delta = AxisAcceleration(chassis, chassis->vy, chassis->target_vy) * seconds;
    float omega_delta = AxisAcceleration(chassis, chassis->omega, chassis->target_omega) * seconds;

    chassis->vx = MoveTowards(chassis->vx, chassis->target_vx, vx_delta);
    chassis->vy = MoveTowards(chassis->vy, chassis->target_vy, vy_delta);
    chassis->omega = MoveTowards(chassis->omega, chassis->target_omega, omega_delta);
}

bool Chassis_IsStopped(const ChassisState* chassis) {
    const float epsilon = 0.0005f;
    return fabsf(chassis->vx) < epsilon && fabsf(chassis->vy) < epsilon && fabsf(chassis->omega) < epsilon;
}

void Chassis_GetOutput(const ChassisState* chassis, float* vx, float* vy, float* omega) {
    if (vx != NULL) {
        *vx = chassis->vx;
    }
    if (vy != NULL) {
        *vy = chassis->vy;
    }
    if (omega != NULL) {
        *omega = chassis->omega;
    }
}
