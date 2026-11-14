#include "app/chassis_port.h"

#include "vehicle.h"

void ChassisPort_Init(void) { Motor_Init(); }

void ChassisPort_Apply(float vx, float vy, float omega) { Vehicle_SetVelocity(vx, vy, omega); }

void ChassisPort_Disable(void) { Vehicle_SetVelocity(0.0f, 0.0f, 0.0f); }
