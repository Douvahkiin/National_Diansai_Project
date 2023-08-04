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
};

void pll_Init(float32 w0, float32 Vm);

float32 pll_Run(float32 input);

void SOGI(float32 input, float32 *alpha, float32 *beta);

#endif  // _PLL_
