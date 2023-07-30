#include "DAC_setup.h"
#include "EPWM_setup.h"
#include "F28x_Project.h"
#include "MACRO.h"
#include "filters.h"

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
  float32 upper_limit;
  float32 lower_limit;
};

void pid_pll_Init(float32 p, float32 i, float32 d);

float32 pid_pll_Run(float32 err);

void pid_nx_Init(float32 p, float32 i, float32 d, float32 upper_limit, float32 lower_limit, struct _pid* pid_nx);

float32 pid_nx_Run(float32 err, struct _pid* pid_nx);

#endif  // !_PWM_
