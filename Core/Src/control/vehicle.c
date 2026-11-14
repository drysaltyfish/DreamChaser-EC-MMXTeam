#include "vehicle.h"

Vehicle vehicle = {
    /* Wheel positions follow the wiring: A = right-rear, B = right-front,
     * C = left-front, D = left-rear. */
    .RR = &MotorA_Handle, .RF = &MotorB_Handle, .LF = &MotorC_Handle, .LR = &MotorD_Handle,
    .target_vx_n = 0.0f,  .target_vy_n = 0.0f,  .target_wz_n = 0.0f,
};

#define TWO_PI 6.2831853f
#define WHEEL_MAX_V (MOTOR_MAX_RPM * TWO_PI * WHEEL_RADIUS / 60.0f)  // 单轮线速度上限 m/s
#define V_MAX WHEEL_MAX_V
#define W_MAX (V_MAX / (LX + LY))                     // rad/s
#define RAD_TO_RPM (60.0f / (TWO_PI * WHEEL_RADIUS))  // 线速度 m/s → RPM

void Vehicle_SetVelocity(float vx_n, float vy_n, float wz_n) {
    /* 1) 输入饱和 */
    if (vx_n > 1.0f)
        vx_n = 1.0f;
    else if (vx_n < -1.0f)
        vx_n = -1.0f;
    if (vy_n > 1.0f)
        vy_n = 1.0f;
    else if (vy_n < -1.0f)
        vy_n = -1.0f;
    if (wz_n > 1.0f)
        wz_n = 1.0f;
    else if (wz_n < -1.0f)
        wz_n = -1.0f;

    vehicle.target_vx_n = vx_n;
    vehicle.target_vy_n = vy_n;
    vehicle.target_wz_n = wz_n;

    /* 2) 归一化量 → 物理量 */
    const float vx = vx_n * V_MAX;
    const float vy = vy_n * V_MAX;
    const float wz = wz_n * W_MAX;
    const float R = LX + LY;

    /* 3) 全向轮运动学 */
    float LF_rpm = (vx - vy - wz * R) * RAD_TO_RPM;
    float RF_rpm = (vx + vy - wz * R) * RAD_TO_RPM;
    float LR_rpm = (vx + vy + wz * R) * RAD_TO_RPM;
    float RR_rpm = (vx - vy + wz * R) * RAD_TO_RPM;

    /* 4) 四轮统一限幅 */
    float max_rpm = fmaxf(fmaxf(fabsf(LF_rpm), fabsf(RF_rpm)), fmaxf(fabsf(LR_rpm), fabsf(RR_rpm)));
    if (max_rpm > MOTOR_MAX_RPM) {
        float scale = MOTOR_MAX_RPM / max_rpm;
        LF_rpm *= scale;
        RF_rpm *= scale;
        LR_rpm *= scale;
        RR_rpm *= scale;
    }

    vehicle.LF->target_rpm = -LF_rpm;
    vehicle.RF->target_rpm = RF_rpm;
    vehicle.LR->target_rpm = -LR_rpm;
    vehicle.RR->target_rpm = RR_rpm;
}