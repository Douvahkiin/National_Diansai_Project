#include "EPWM_setup.h"
#include "F28x_Project.h"
#include "math.h"
#include "pid.h"

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
};

struct _sogi {
  float32 D_d1;
  float32 D_d2;
  float32 D_d3;
  float32 D_n1;
  float32 D_n2;

  float32 Q_d1;
  float32 Q_d2;
  float32 Q_d3;
  float32 Q_n1;
  float32 Q_n2;
  float32 Q_n3;
};

void pll_Init(float32 w0, float32 Vm);

float32 pll_Run(float32 input);

void sogid_init(float32 d_d1, float32 d_d2, float32 d_d3, float32 d_n1, float32 d_n2);
void sogiq_init(float32 q_d1, float32 q_d2, float32 q_d3, float32 q_n1, float32 q_n2, float32 q_n3);

void SOGI(float32 input, float32 *alpha, float32 *beta);

#endif /* _PLL_ */
