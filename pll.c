#include "pll.h"

struct _pll pll1;
struct _sogi sogi1;
extern struct _pid pid_pll1;
struct _pll pll2;
struct _sogi sogi2;
extern struct _pid pid_pll2;
extern float32 U2_q;
float32 SOGI_K = 0.5;
float32 SOGI_IntLimit = 40;

/// @brief 锁相环的初始化
/// @param w0 角速度
/// @param Vm 幅值
void pll_Init(float32 w0, float32 Vm, struct _pll *pllx, struct _sogi *sogix, struct _pid *pid_pllx) {
  pllx->inputVal_alpha = 0;
  pllx->inputVal_beta = 0;
  pllx->negSine = 0;
  pllx->posCosine = 0;
  pllx->integral = 0;
  pllx->Vm = Vm;
  pllx->w0 = w0;  // 50*2*pi
  pllx->Ts = 0.00005 * SW_FREQ;
  pid_pll_Init(180, 3200, 0, pid_pllx);
  sogix->SOGI_K = SOGI_K;  // SOGI算法的增益K
  sogix->SOGI_Integral1 = 0;
  sogix->SOGI_Integral2 = 0;
  sogix->SOGI_w0 = w0;  // 50*2*pi
  sogix->SOGI_IntLimit = SOGI_IntLimit;
  sogix->SOGI_Ts = 0.00005 * SW_FREQ;
}

void SOGI(float32 input, float32 *alpha, float32 *beta, struct _sogi *sogix) {
  float32 pre_input = input - *alpha;
  float32 next_input = pre_input * sogix->SOGI_K - *beta;

  // 特别注意: 积分必须乘以采样时间
  sogix->SOGI_Integral1 += next_input * sogix->SOGI_w0 * sogix->SOGI_Ts;
  // 限制 integral1 积分值
  sogix->SOGI_Integral1 = saturation(sogix->SOGI_Integral1, sogix->SOGI_IntLimit, -sogix->SOGI_IntLimit);

  *alpha = sogix->SOGI_Integral1;  // 输出 V_alpha

  sogix->SOGI_Integral2 += *alpha * sogix->SOGI_w0 * sogix->SOGI_Ts;
  // 限制 integral2 积分值
  sogix->SOGI_Integral2 = saturation(sogix->SOGI_Integral2, sogix->SOGI_IntLimit, -sogix->SOGI_IntLimit);

  *beta = sogix->SOGI_Integral2;  // 输出 V_beta
}

float32 pll_Run(float32 input, struct _pll *pllx, struct _sogi *sogix, struct _pid *pid_pllx, float32 *dq_q) {
  // 调用 SOGI, 滤波并产生正交的V_alpha与V_beta
  SOGI(input, &pllx->inputVal_alpha, &pllx->inputVal_beta, sogix);
  float32 alpha = pllx->alpha_2;
  pllx->alpha_2 = pllx->alpha_1;
  pllx->alpha_1 = pllx->inputVal_alpha;
  float32 beta = pllx->beta_2;
  pllx->beta_2 = pllx->beta_1;
  pllx->beta_1 = pllx->inputVal_beta;
  /* 幅值自适应PLL */
  // Vm = sqrt(alpha^2+beta^2); 然后Vm=max(Vm, 0.5), 防止Vm算出来为0造成除法错误
  float32 Vm = sqrt(alpha * alpha + beta * beta);
  Vm = fmaxf(Vm, 0.1);
  // changeDACAVal(2048 + 2000.0 * alpha / Vm);
  // changeDACAVal(2048 + 2000.0 * temp / Vm);
  float32 pid_input = (alpha * pllx->negSine + beta * pllx->posCosine) / Vm;
  float32 uk = pid_pll_Run(pid_input, pid_pllx);
  float32 preOut = uk + pllx->w0;
  pllx->integral += preOut * pllx->Ts;
  if (pllx->integral >= 2 * PI) {
    pllx->integral -= 2 * PI;
    // GpioDataRegs.GPATOGGLE.bit.GPIO22 = 1;  // toggle GPIO22
  }
  pllx->negSine = -sin(pllx->integral);
  pllx->posCosine = cos(pllx->integral);
  *dq_q = alpha_beta2dq(alpha, beta, pllx->integral);
  return pllx->integral;
}
