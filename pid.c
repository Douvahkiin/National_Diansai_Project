#include "pid.h"

struct _pid pid_pll1;
struct _pid pid_pll2;
struct _pid pid_n1;
struct _pid pid_n2;

void pid_pll_Init(float32 p, float32 i, float32 d, struct _pid* pid_pllx) {
  pid_pllx->err_last = 0;
  pid_pllx->uk = 0;
  pid_pllx->integral = 0;
  pid_pllx->derivative = 0;
  pid_pllx->kp = p;
  pid_pllx->ki = i;
  pid_pllx->kd = d;
  pid_pllx->Ts = 0.00005 * SW_FREQ;
}

float32 pid_pll_Run(float32 err, struct _pid* pid_pllx) {
  pid_pllx->integral += err * pid_pllx->Ts;  // 之前居然忘了乘以采样时间!
  pid_pllx->derivative = (err - pid_pllx->err_last) / pid_pllx->Ts;
  pid_pllx->integral = saturation(pid_pllx->integral, 100, -100);

  // pid_pllx->uk is u(k)
  pid_pllx->uk = pid_pllx->kp * err + pid_pllx->ki * pid_pllx->integral + pid_pllx->kd * pid_pllx->derivative;
  pid_pllx->err_last = err;
  return pid_pllx->uk;  // uk ---> pwm gerenator
}

void pid_nx_Init(float32 p, float32 i, float32 d, float32 integrator_upper_limit, float32 integrator_lower_limit, struct _pid* pid_nx) {
  pid_nx->err_last = 0;
  pid_nx->uk = 0;
  pid_nx->integral = 0;
  pid_nx->kp = p;
  pid_nx->ki = i;
  pid_nx->kd = d;
  pid_nx->integrator_lower_limit = integrator_lower_limit;
  pid_nx->integrator_upper_limit = integrator_upper_limit;
  pid_nx->Ts = 0.00005 * SW_FREQ;
}

float32 pid_nx_Run(float32 err, struct _pid* pid_nx) {
  pid_nx->integral += err * pid_nx->Ts;
  pid_nx->derivative = (err - pid_nx->err_last) / pid_nx->Ts;
  pid_nx->integral = saturation(pid_nx->integral, pid_nx->integrator_upper_limit, pid_nx->integrator_lower_limit);

  // pid_nx->uk is u(k)
  pid_nx->uk = pid_nx->kp * err + pid_nx->ki * pid_nx->integral + pid_nx->kd * pid_nx->derivative;
  pid_nx->err_last = err;
  return pid_nx->uk;  // uk ---> pwm gerenator
}
