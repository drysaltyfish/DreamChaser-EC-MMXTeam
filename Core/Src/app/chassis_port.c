#include "app/chassis_port.h"

#include "app/app_config.h"

#if APP_CHASSIS_DRIVE_ENABLED
#include "motor.h"
#include "vehicle.h"
#endif

void ChassisPort_Init(void) {
#if APP_CHASSIS_DRIVE_ENABLED
    /* The original demo task used to do this. Keep the motor control task and
     * its 10 ms tick alive when chassis driving is enabled. */
    Motor_Init();
#endif
}

void ChassisPort_Apply(float vx, float vy, float omega) {
#if APP_CHASSIS_DRIVE_ENABLED
    Vehicle_SetVelocity(vx, vy, omega);
#else
    (void)vx;
    (void)vy;
    (void)omega;
#endif
}

void ChassisPort_Disable(void) {
#if APP_CHASSIS_DRIVE_ENABLED
    Vehicle_SetVelocity(0.0f, 0.0f, 0.0f);
#endif
}
