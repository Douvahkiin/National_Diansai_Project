#include "filters.h"

float32 low_pass_filter(float32 input, float32 *outputPre, float32 alpha) {
  float32 output = alpha * input + (1 - alpha) * *outputPre;
  *outputPre = output;
  return output;
}

float32 saturation(float32 input, float32 max, float32 min) {
  input = fminf(input, max);
  input = fmaxf(input, min);
  return input;
}
