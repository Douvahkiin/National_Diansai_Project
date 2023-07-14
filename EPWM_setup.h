#include "F28x_Project.h"
#include "math.h"

#ifndef _EPWM_SET_
#define _EPWM_SET_

#define PWM_MAX_COUNT 10000
#define PWM_MID_COUNT (PWM_MAX_COUNT / 2)

void ConfigureEPWM(void);
void changeDuty_phase(float32 wt);
void changeDuty_value(float32 val);

#endif  // !_EPWM_SET_
