#ifndef CHASSIS_PORT_H
#define CHASSIS_PORT_H

/**
 * Hardware boundary for the chassis.
 *
 * The default build has APP_CHASSIS_DRIVE_ENABLED set to 0 and the port is a
 * no-op, because the four motor handles do not exist yet. Set it to 1 once
 * MotorB/C/D and Vehicle_SetVelocity are implemented; only this file changes.
 */
void ChassisPort_Init(void);

/**
 * Normalized command in -1..1 on each axis.
 *
 * Called with the motion-control mutex held. Keep this non-blocking and do
 * not call back into MotionControl from here.
 */
void ChassisPort_Apply(float vx, float vy, float omega);

/** Cuts the drive outputs. */
void ChassisPort_Disable(void);

#endif /* CHASSIS_PORT_H */
