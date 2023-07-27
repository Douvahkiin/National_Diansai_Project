#include "pid.h"

struct _pid pid_pll;
struct _pid pid_n1;

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

void pid_n1_Init(float32 p, float32 i, float32 d) {
  pid_n1.err_last = 0;
  pid_n1.uk = 0;
  pid_n1.integral = 0;
  pid_n1.kp = p;
  pid_n1.ki = i;
  pid_n1.kd = d;
  pid_n1.Ts = 0.00005 * SW_FREQ;
}

float32 pid_n1_Run(float32 err) {
  pid_n1.integral += err * pid_n1.Ts;
  pid_n1.derivative = (err - pid_n1.err_last) / pid_n1.Ts;
  if (pid_n1.integral > 10) {
    pid_n1.integral = 10;
  } else if (pid_n1.integral < -10) {
    pid_n1.integral = -10;
  }

  // pid_n1.uk is u(k)
  pid_n1.uk = pid_n1.kp * err + pid_n1.ki * pid_n1.integral + pid_n1.kd * pid_n1.derivative;
  pid_n1.err_last = err;
  return pid_n1.uk;  // uk ---> pwm gerenator
}
