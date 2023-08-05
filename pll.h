#include "EPWM_setup.h"
#include "F28x_Project.h"
#include "filters.h"
#include "math.h"
#include "pid.h"
#include "utils.h"

#ifndef _PLL_
#define _PLL_

struct _pll {
  float32 inputVal_alpha;
  float32 inputVal_beta;
  float32 negSine;
  float32 posCosine;
  float32 integral;
  float32 Vm;
  float32 w0;
  float32 Ts;
  float32 alpha_1;
  float32 alpha_2;
  float32 beta_1;
  float32 beta_2;
};

struct _sogi {
  float32 SOGI_K;
  float32 SOGI_Integral1;
  float32 SOGI_Integral2;
  float32 SOGI_w0;
  float32 SOGI_IntLimit;
  float32 SOGI_Ts;
};

void pll_Init(float32 w0, float32 Vm, struct _pll *pllx, struct _sogi *sogix, struct _pid *pid_pllx);

float32 pll_Run(float32 input, struct _pll *pllx, struct _sogi *sogix, struct _pid *pid_pllx, float32 *dq_q);

void SOGI(float32 input, float32 *alpha, float32 *beta, struct _sogi *sogix);

#endif  // _PLL_
