#ifndef CHASSIS_PORT_H
#define CHASSIS_PORT_H

/**
 * Hardware boundary for the chassis.
 *
 * All four wheels are driven: Apply hands the normalized body command
 * (vx, vy, omega each in -1..1) to Vehicle_SetVelocity, which runs the
 * omni-wheel kinematics and writes the four wheel RPM targets. The 10 ms
 * motor task closes the loop on each wheel.
 * The BT READY always advertises HAS_CHASSIS because this port answers
 * velocity commands.
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
