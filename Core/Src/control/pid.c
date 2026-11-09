#include "pid.h"

void PID_Init(PID_Handle* pid) {
    for (int i = 0; i < 3; i++) {
        pid->errors[i] = 0;
    }
    pid->last_output = 0;
}

float PID_Calculate(PID_Handle* pid, float setpoint, float feedback) {
    // 1. 记录历史误差
    pid->errors[2] = pid->errors[1];  // e(k-2)
    pid->errors[1] = pid->errors[0];  // e(k-1)

    // 2. 计算当前误差 e(k)
    pid->errors[0] = setpoint - feedback;

    // 3. 防止除零错误，计算采样周期 Ts (单位：秒)
    float ts = (pid->def.t_mill > 0) ? (pid->def.t_mill / 1000.0f) : 0.01f;

    // 4. 计算增量 Δu
    // Δu = Kp * [e(k) - e(k-1)] + Ki * Ts * e(k) + (Kd / Ts) * [e(k) - 2*e(k-1) + e(k-2)]
    float delta_u = pid->def.Kp * (pid->errors[0] - pid->errors[1]) + pid->def.Ki * ts * pid->errors[0] +
                    (pid->def.Kd / ts) * (pid->errors[0] - 2.0f * pid->errors[1] + pid->errors[2]);

    // 5. 增量限幅 (可选：防止单次调节步子太大，引起剧烈震荡)
    float delta_max = (pid->def.output_max - pid->def.output_min) * 0.3f;
    if (delta_u > delta_max) delta_u = delta_max;
    if (delta_u < -delta_max) delta_u = -delta_max;

    // 6. 计算当前绝对输出 u(k) = u(k-1) + Δu
    float output = pid->last_output + delta_u;

    // 7. 总输出限幅 (抗积分饱和的核心)
    if (output > pid->def.output_max) {
        output = pid->def.output_max;
    } else if (output < pid->def.output_min) {
        output = pid->def.output_min;
    }

    // 8. 保存本次输出，供下次计算使用
    pid->last_output = output;

    return output;
}