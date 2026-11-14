#ifndef VEHICLE_H
#define VEHICLE_H
#include "motor.h"

#define WHEEL_RADIUS 0.045f  // m
#define LX 0.13571f          // 轮心到中心横向距离 (m)
#define LY 0.1550f           // 轮心到中心纵向距离 (m)

typedef struct {
    Motor_Handle *LF, *RF, *LR, *RR;
    float target_vx_n, target_vy_n, target_wz_n;
} Vehicle;

extern Vehicle vehicle;

void Vehicle_SetVelocity(float vx_n, float vy_n, float wz_n);

#endif