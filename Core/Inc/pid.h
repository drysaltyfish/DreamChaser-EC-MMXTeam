#ifndef __PID_H_
#define __PID_H_
#include "stdint.h"

typedef struct {
  float Kp, Ki, Kd;
  uint8_t t_mill;
  float output_min;
  float output_max;

} PID_Def;
typedef struct {
  PID_Def def;
  float last_output;
  float errors[3];
} PID_Handle;
void PID_Init(PID_Handle *pid);
float PID_Calculate(PID_Handle *pid, float setpoint, float feedback);
#endif