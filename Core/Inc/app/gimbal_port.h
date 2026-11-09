#ifndef GIMBAL_PORT_H
#define GIMBAL_PORT_H

#include <stdint.h>

/**
 * Hardware boundary for the gimbal.
 *
 * The protocol and motion layers are independent of the actual actuators.
 * Fill these functions in when the gimbal driver is available; the default
 * implementation is a no-op so the project builds and the model still runs.
 */
void GimbalPort_Init(void);

/**
 * Writes the two axis positions, in 0.01 degree units.
 *
 * Called with the motion-control mutex held. Keep this non-blocking and do
 * not call back into MotionControl from here.
 */
void GimbalPort_Write(int16_t pan, int16_t tilt);

/** Disables the gimbal outputs (emergency stop). */
void GimbalPort_Disable(void);

#endif /* GIMBAL_PORT_H */
