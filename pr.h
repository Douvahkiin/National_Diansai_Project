#include "F28x_Project.h"

#ifndef _PR_
#define _PR_
// \omega_c == 5*2*pi (5Hz)
// \omega_0 == 50*2*pi (50Hz)
// Ts == 0.0001s
struct _pr {
  float32 d1;
  float32 d2;
  float32 d3;
  float32 n1;
  float32 n2;
  float32 n3;
  float32 yk_1;
  float32 yk_2;
  float32 uk_1;
  float32 uk_2;
};

void pr_init(float32 d1, float32 d2, float32 d3, float32 n1, float32 n2, float32 n3, struct _pr* prx);

float32 pr_run(float32 uk, struct _pr* prx);

#endif /* _PR_ */
