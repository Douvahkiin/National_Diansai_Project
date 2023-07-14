#include "F28x_Project.h"

#ifndef _PWM_
#define _PWM_

struct _pid {
  // 实际上, uk其实并不是必要的
  float32 err_last;
  float32 kp, ki, kd;
  float32 uk;
  float32 integral;
  float32 derivative;
  float32 Ts;
};

void pid_pll_Init(float32 p, float32 i, float32 d);

float32 pid_pll_Run(float32 err);

void pid_n1_Init(float32 p, float32 i, float32 d);

float32 pid_n1_Run(float32 err);

#endif  // !_PWM_