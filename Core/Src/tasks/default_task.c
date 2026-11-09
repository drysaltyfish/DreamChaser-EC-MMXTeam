#include "default_task.h"

void StartDefaultTask(void* argument) {
    Motor_Init();

    int state = 0;
    for (int i = 0;; i++) {
        state = (state + 1) % 5;
        if (state == 0) {
            MotorHandles[0]->target_rpm = 0;
        } else if (state == 1) {
            MotorHandles[0]->target_rpm = 55;
        } else if (state == 2) {
            MotorHandles[0]->target_rpm = -55;
        } else if (state == 3) {
            MotorHandles[0]->target_rpm = 110;
        } else if (state == 4) {
            MotorHandles[0]->target_rpm = -110;
        }
        osDelay(2000);
    }
    // MotorHandles[0]->target_rpm = 84;
    // int speed = 0;
    // for (int i = 0;; i++) {
    //   speed = speed * 0.9 + MotorHandles[0]->current_speed * 0.1;
    //   osDelay(20);
    //   if (i % 100 == 0) {
    //     // printf("Current Speed: %d, Filtered Speed: %d\n",
    //     // MotorHandles[0]->current_speed, speed);
    //     i++;
    //   }
    // }
}
