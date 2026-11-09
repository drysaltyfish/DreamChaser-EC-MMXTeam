#ifndef __MOTOR_H
#define __MOTOR_H
#include "pid.h"
#include "stm32f4xx_hal.h"
#include "tools.h"

typedef struct {
    TIM_HandleTypeDef* in1_htim;
    uint32_t in1_channel;
    GPIO_TypeDef* in2_gpiox;
    uint16_t in2_gpiopin;

    TIM_HandleTypeDef* encoder_htim;
    Bool encoder_reversed;

    uint16_t CPR;  // Counts Per Revolution
} Motor_Def;
typedef struct {
    Motor_Def def;
    PID_Handle pid;
    Bool braking;
    float target_rpm;
    float current_rpm;

    int16_t enc_counter;
} Motor_Handle;

#define MOTOR_MAX_ARR 8400
#define MOTOR_MAX_RPM 110
#define NUM_OF_MOTORS 1

void Motor_Init(void);
void Motor_SetPWM(Motor_Handle* hmotor, int arr);
int16_t Motor_ConsumeCounter(Motor_Handle* hmotor);
void Motor_10ms_Tick();

extern Motor_Handle* MotorHandles[NUM_OF_MOTORS];
extern Motor_Handle MotorA_Handle;
extern Motor_Handle MotorB_Handle;
extern Motor_Handle MotorC_Handle;
extern Motor_Handle MotorD_Handle;
#endif