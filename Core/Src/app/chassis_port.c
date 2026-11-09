#include "app/chassis_port.h"

#include "app/app_config.h"
#include "motor.h"
// #include "vehicle.h"

void ChassisPort_Init(void) { Motor_Init(); }

void ChassisPort_Apply(float vx, float vy, float omega) {
    // Vehicle_SetVelocity(vx, vy, omega);
    MotorA_Handle.target_rpm = vx * 110.0f;
}

void ChassisPort_Disable(void) {
    MotorA_Handle.target_rpm = 0.0f;
    // Vehicle_SetVelocity(0.0f, 0.0f, 0.0f);
}
