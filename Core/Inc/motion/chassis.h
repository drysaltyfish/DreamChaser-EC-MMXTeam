#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float accel_per_s;
    float decel_per_s;
    float emergency_decel_per_s;
    float deadzone;
} ChassisConfig;

/**
 * Normalized three-axis chassis command with acceleration limiting.
 *
 * Commands enter in the -1000..1000 range used on the wire; the output is
 * normalized to -1..1. An emergency stop always ramps to zero and latches
 * until the next normal command or Chassis_Stop.
 */
typedef struct {
    ChassisConfig config;
    float target_vx;
    float target_vy;
    float target_omega;
    float vx;
    float vy;
    float omega;
    bool emergency;
} ChassisState;

void Chassis_Init(ChassisState* chassis, const ChassisConfig* config);
void Chassis_SetCommand(ChassisState* chassis, int16_t vx, int16_t vy, int16_t omega);
void Chassis_Stop(ChassisState* chassis);
void Chassis_EmergencyStop(ChassisState* chassis);
void Chassis_Step(ChassisState* chassis, uint32_t elapsed_ms);
bool Chassis_IsStopped(const ChassisState* chassis);
void Chassis_GetOutput(const ChassisState* chassis, float* vx, float* vy, float* omega);

#endif /* CHASSIS_H */
