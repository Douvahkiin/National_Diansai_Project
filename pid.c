#include "pid.h"

struct _pid pid_pll;
struct _pid pid_n1;
struct _pid pid_n2;

void pid_pll_Init(float32 p, float32 i, float32 d) {
  pid_pll.err_last = 0;
  pid_pll.uk = 0;
  pid_pll.integral = 0;
  pid_pll.derivative = 0;
  pid_pll.kp = p;
  pid_pll.ki = i;
  pid_pll.kd = d;
  pid_pll.Ts = 0.00005 * SW_FREQ;
}

float32 pid_pll_Run(float32 err) {
  pid_pll.integral += err * pid_pll.Ts;  // 之前居然忘了乘以采样时间!
  pid_pll.derivative = (err - pid_pll.err_last) / pid_pll.Ts;
  if (pid_pll.integral > 100) {
    pid_pll.integral = 100;
  } else if (pid_pll.integral < -100) {
    pid_pll.integral = -100;
  }

  // pid_pll.uk is u(k)
  pid_pll.uk = pid_pll.kp * err + pid_pll.ki * pid_pll.integral + pid_pll.kd * pid_pll.derivative;
  pid_pll.err_last = err;
  return pid_pll.uk;  // uk ---> pwm gerenator
}

void pid_nx_Init(float32 p, float32 i, float32 d, struct _pid* pid_nx) {
  pid_nx->err_last = 0;
  pid_nx->uk = 0;
  pid_nx->integral = 0;
  pid_nx->kp = p;
  pid_nx->ki = i;
  pid_nx->kd = d;
  pid_nx->Ts = 0.00005 * SW_FREQ;
}

float32 pid_nx_Run(float32 err, struct _pid* pid_nx) {
  pid_nx->integral += err * pid_nx->Ts;
  pid_nx->derivative = (err - pid_nx->err_last) / pid_nx->Ts;
  if (pid_nx->integral > 10) {
    pid_nx->integral = 10;
  } else if (pid_nx->integral < -10) {
    pid_nx->integral = -10;
  }

  // pid_nx->uk is u(k)
  pid_nx->uk = pid_nx->kp * err + pid_nx->ki * pid_nx->integral + pid_nx->kd * pid_nx->derivative;
  pid_nx->err_last = err;
  return pid_nx->uk;  // uk ---> pwm gerenator
}
