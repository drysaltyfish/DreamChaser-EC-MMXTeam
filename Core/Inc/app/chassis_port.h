#ifndef CHASSIS_PORT_H
#define CHASSIS_PORT_H

/**
 * Hardware boundary for the chassis.
 *
 * Only MotorA exists today: Apply maps vx to MotorA.target_rpm and the 10 ms
 * motor task closes the loop. vy and omega are accepted by the protocol but
 * have no actuator yet; MotorB/C/D and Vehicle_SetVelocity are not implemented.
 * The BT READY always advertises HAS_CHASSIS because this port answers velocity
 * commands, even though the drive is still partial.
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
