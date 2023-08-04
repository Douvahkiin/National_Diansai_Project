#include "pll.h"

struct _pll pll;
extern struct _pid pid_pll;
extern float32 U2_q;
float32 SOGI_K;
float32 SOGI_Integral1;
float32 SOGI_Integral2;
float32 SOGI_w0;
float32 SOGI_IntLimit;
float32 SOGI_Ts;
float32 alpha_1;
float32 alpha_2;
float32 beta_1;
float32 beta_2;

/// @brief 锁相环的初始化
/// @param w0 角速度
/// @param Vm 幅值
void pll_Init(float32 w0, float32 Vm) {
  pll.inputVal_alpha = 0;
  pll.inputVal_beta = 0;
  pll.negSine = 0;
  pll.posCosine = 0;
  pll.integral = 0;
  pll.Vm = Vm;
  pll.w0 = w0;  // 50*2*pi
  pll.Ts = 0.00005 * SW_FREQ;
  pid_pll_Init(180, 3200, 0);
  SOGI_K = 0.5;  // SOGI算法的增益K
  SOGI_Integral1 = 0;
  SOGI_Integral2 = 0;
  SOGI_w0 = w0;  // 50*2*pi
  SOGI_IntLimit = 40;
  SOGI_Ts = 0.00005 * SW_FREQ;
}

void SOGI(float32 input, float32 *alpha, float32 *beta) {
  float32 pre_input = input - *alpha;
  float32 next_input = pre_input * SOGI_K - *beta;

  // 特别注意: 积分必须乘以采样时间
  SOGI_Integral1 += next_input * SOGI_w0 * SOGI_Ts;
  // 限制 integral1 积分值
  SOGI_Integral1 = saturation(SOGI_Integral1, SOGI_IntLimit, -SOGI_IntLimit);

  *alpha = SOGI_Integral1;  // 输出 V_alpha

  SOGI_Integral2 += *alpha * SOGI_w0 * SOGI_Ts;
  // 限制 integral2 积分值
  SOGI_Integral2 = saturation(SOGI_Integral2, SOGI_IntLimit, -SOGI_IntLimit);

  *beta = SOGI_Integral2;  // 输出 V_beta
}

float32 pll_Run(float32 input) {
  // 调用 SOGI, 滤波并产生正交的V_alpha与V_beta
  SOGI(input, &pll.inputVal_alpha, &pll.inputVal_beta);
  float32 alpha = alpha_2;
  alpha_2 = alpha_1;
  alpha_1 = pll.inputVal_alpha;
  float32 beta = beta_2;
  beta_2 = beta_1;
  beta_1 = pll.inputVal_beta;
  /* 幅值自适应PLL */
  // Vm = sqrt(alpha^2+beta^2); 然后Vm=max(Vm, 0.5), 防止Vm算出来为0造成除法错误
  float32 Vm = sqrt(alpha * alpha + beta * beta);
  Vm = fmaxf(Vm, 0.5);
  // changeDACAVal(2048 + 2000.0 * alpha / Vm);
  // changeDACAVal(2048 + 2000.0 * temp / Vm);
  float32 pid_input = (alpha * pll.negSine + beta * pll.posCosine) / Vm;
  float32 uk = pid_pll_Run(pid_input);
  float32 preOut = uk + pll.w0;
  pll.integral += preOut * pll.Ts;
  if (pll.integral >= 2 * PI) {
    pll.integral -= 2 * PI;
    // GpioDataRegs.GPATOGGLE.bit.GPIO22 = 1;  // toggle GPIO22
  }
  pll.negSine = -sin(pll.integral);
  pll.posCosine = cos(pll.integral);
  U2_q = alpha_beta2dq(alpha, beta, pll.integral);
  return pll.integral;
}
