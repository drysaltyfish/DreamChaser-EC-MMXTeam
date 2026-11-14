#include "motor.h"

#include <stddef.h>

#include "cmsis_os.h"
#include "main.h"

// 11 线编码器 × 90 减速比 × 4（TI12 模式对 TI1/TI2 的上下沿都计数）。
#define MOTOR_CPR (11 * 90 * 4)

/* 测速/PID 采样周期：TIM10 = 167.936 MHz / 84 / 20000 ≈ 10.004 ms（见 Fusai.ioc），
 * 必须与 PID 的 t_mill = 10 和 Motor_Update 里的 60.0f / 0.01f 保持一致。 */

Motor_Handle MotorA_Handle = {
    .def = {.in1_htim = &htim8,
            .in1_channel = TIM_CHANNEL_4,

            .in2_gpiox = MOTORA_IN2_GPIO_Port,
            .in2_gpiopin = MOTORA_IN2_Pin,

            .encoder_htim = &htim1,
            .encoder_reversed = True,
            .CPR = MOTOR_CPR},
    .pid =
        {
            .def =
                {
                    .Kp = 0.75f,
                    .Ki = 8.5f,
                    .Kd = 0.00f,
                    .t_mill = 10,
                    .output_min = -MOTOR_MAX_RPM,
                    .output_max = MOTOR_MAX_RPM,
                },
        },
};

Motor_Handle MotorB_Handle = {
    .def = {.in1_htim = &htim8,
            .in1_channel = TIM_CHANNEL_3,

            .in2_gpiox = MOTORB_IN2_GPIO_Port,
            .in2_gpiopin = MOTORB_IN2_Pin,

            .encoder_htim = &htim2,
            .encoder_reversed = True,
            .CPR = MOTOR_CPR},
    .pid =
        {
            .def =
                {
                    .Kp = 0.75f,
                    .Ki = 8.5f,
                    .Kd = 0.00f,
                    .t_mill = 10,
                    .output_min = -MOTOR_MAX_RPM,
                    .output_max = MOTOR_MAX_RPM,
                },
        },
};

Motor_Handle MotorC_Handle = {
    .def = {.in1_htim = &htim8,
            .in1_channel = TIM_CHANNEL_2,

            .in2_gpiox = MOTORC_IN2_GPIO_Port,
            .in2_gpiopin = MOTORC_IN2_Pin,

            .encoder_htim = &htim3,
            .encoder_reversed = False,
            .CPR = MOTOR_CPR},
    .pid =
        {
            .def =
                {
                    .Kp = 0.75f,
                    .Ki = 8.5f,
                    .Kd = 0.00f,
                    .t_mill = 10,
                    .output_min = -MOTOR_MAX_RPM,
                    .output_max = MOTOR_MAX_RPM,
                },
        },
};

Motor_Handle MotorD_Handle = {
    .def = {.in1_htim = &htim8,
            .in1_channel = TIM_CHANNEL_1,

            .in2_gpiox = MOTORD_IN2_GPIO_Port,
            .in2_gpiopin = MOTORD_IN2_Pin,

            .encoder_htim = &htim4,
            .encoder_reversed = True,
            .CPR = MOTOR_CPR},
    .pid =
        {
            .def =
                {
                    .Kp = 0.75f,
                    .Ki = 8.5f,
                    .Kd = 0.00f,
                    .t_mill = 10,
                    .output_min = -MOTOR_MAX_RPM,
                    .output_max = MOTOR_MAX_RPM,
                },
        },
};
/* Wheels exposed over the wire, in A/B/C/D order; every slot has a driver now. */
static Motor_Handle* const s_wheel_handles[MOTOR_WHEEL_COUNT] = {&MotorA_Handle, &MotorB_Handle, &MotorC_Handle,
                                                                 &MotorD_Handle};

TaskHandle_t xMotorTaskHandle = NULL;
void vMotorControlTask(void* pvParameters);

void Motor_Init(void) {
    for (int i = 0; i < NUM_OF_MOTORS; i++) {
        s_wheel_handles[i]->braking = False;

        Motor_Def* motor_def = &s_wheel_handles[i]->def;
        HAL_TIM_PWM_Start(motor_def->in1_htim, motor_def->in1_channel);
        __HAL_TIM_SET_COMPARE(motor_def->in1_htim, motor_def->in1_channel, 0);
        HAL_GPIO_WritePin(motor_def->in2_gpiox, motor_def->in2_gpiopin, GPIO_PIN_RESET);

        HAL_TIM_Encoder_Start(motor_def->encoder_htim, TIM_CHANNEL_ALL);

        PID_Init(&s_wheel_handles[i]->pid);
    }

    BaseType_t ret = xTaskCreate(vMotorControlTask,  // 任务入口函数
                                 "MotorCtrlTask",    // 任务名称
                                 512,                // 栈深度：512 字 = 2048 字节（四路浮点 PID 留余量）
                                 NULL,               // 不传递参数
                                 4,                  // 优先级设为 4（高于空闲任务即可）
                                 &xMotorTaskHandle   // 保存任务句柄（后续给中断唤醒用）
    );
    if (ret != pdPASS) {
        // 任务创建失败，处理错误
        Error_Handler();
    }
}

void Motor_GetTelemetry(MotorTelemetry* telemetry) {
    if (telemetry == NULL) {
        return;
    }

    uint8_t present_mask = 0u;
    taskENTER_CRITICAL();
    for (uint32_t i = 0u; i < MOTOR_WHEEL_COUNT; i++) {
        Motor_Handle* hmotor = s_wheel_handles[i];
        if (hmotor != NULL) {
            present_mask |= (uint8_t)(1u << i);
            telemetry->rpm[i] = hmotor->current_rpm;
        } else {
            telemetry->rpm[i] = 0.0f;
        }
    }
    telemetry->present_mask = present_mask;
    taskEXIT_CRITICAL();
}

void Motor_Update(Motor_Handle* hmotor) {
    PID_Handle* pid = &hmotor->pid;
    Motor_Def* motor_def = &hmotor->def;
    float raw_rpm = (float)hmotor->enc_counter / motor_def->CPR * (60.0f / 0.01f);
    // 2. 一阶低通滤波（带边界优化）
    if (hmotor->target_rpm == 0.0f && raw_rpm == 0.0f) {
        // 停机状态：直接清零，防止滤波拖尾产生反向刹车
        hmotor->current_rpm = 0.0f;
    } else if (hmotor->current_rpm == 0.0f) {
        // 刚启动：跳过滤波，直接使用真实值，提高起步响应
        hmotor->current_rpm = raw_rpm;
    } else {
        // 正常运行：执行一阶低通
        hmotor->current_rpm = lpf(raw_rpm, hmotor->current_rpm, 0.3);
    }

    float filtered_target_rpm = lpf(hmotor->target_rpm, hmotor->current_rpm, 0.6);

    float output =
        PID_Calculate(pid, filtered_target_rpm, hmotor->current_rpm) * (MOTOR_MAX_ARR * 1.0f / MOTOR_MAX_RPM);
    Motor_SetPWM(hmotor, (int)output);
}
void Motor_SetPWM(Motor_Handle* hmotor, int arr) {
    Motor_Def* motor_def = &hmotor->def;
    hmotor->braking = False;
    if (abs(arr) > MOTOR_MAX_ARR) {
        arr = sgn(arr) * MOTOR_MAX_ARR;
    } else if (abs(arr) < 100) {
        arr = sgn(arr) * 100;
    }
    if (arr > 0) {
        HAL_GPIO_WritePin(motor_def->in2_gpiox, motor_def->in2_gpiopin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor_def->in1_htim, motor_def->in1_channel, arr);
    } else if (arr < 0) {
        __HAL_TIM_SET_COMPARE(motor_def->in1_htim, motor_def->in1_channel, MOTOR_MAX_ARR + arr);
        HAL_GPIO_WritePin(motor_def->in2_gpiox, motor_def->in2_gpiopin, GPIO_PIN_SET);
    } else {  // 0
        __HAL_TIM_SET_COMPARE(motor_def->in1_htim, motor_def->in1_channel, 0);
        HAL_GPIO_WritePin(motor_def->in2_gpiox, motor_def->in2_gpiopin, GPIO_PIN_RESET);
    }
}
void Motor_Brake(Motor_Handle* hmotor) {
    hmotor->braking = True;
    Motor_Def* motor_def = &hmotor->def;
    HAL_GPIO_WritePin(motor_def->in2_gpiox, motor_def->in2_gpiopin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(motor_def->in1_htim, motor_def->in1_channel, MOTOR_MAX_ARR);
}

int16_t Motor_ConsumeCounter(Motor_Handle* hmotor) {
    Motor_Def* motor_def = &hmotor->def;
    int16_t counter = (int16_t)__HAL_TIM_GET_COUNTER(motor_def->encoder_htim);
    __HAL_TIM_SET_COUNTER(motor_def->encoder_htim, 0);
    if (motor_def->encoder_reversed) {
        counter = -counter;
    }
    return counter;
}

void vMotorControlTask(void* pvParameters) {
    HAL_TIM_Base_Start_IT(&htim10);

    for (;;) {
        // 等待 10ms 周期的通知
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        for (int i = 0; i < NUM_OF_MOTORS; i++) {
            Motor_Update(s_wheel_handles[i]);
        }
    }
}

void Motor_10ms_Tick() {
    UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
    for (int i = 0; i < NUM_OF_MOTORS; i++) {
        s_wheel_handles[i]->enc_counter = Motor_ConsumeCounter(s_wheel_handles[i]);
    }
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);

    /* 2. 发送任务通知 (FreeRTOS 安全 API) */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(xMotorTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
