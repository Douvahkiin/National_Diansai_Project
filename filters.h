#include "F28x_Project.h"

#ifndef _FILTERS_
#define _FILTERS_

float32 low_pass_filter(float32 input, float32 *outputPre, float32 alpha);

float32 saturation(float32 input, float32 max, float32 min);

#endif  // !_FILTERS_
