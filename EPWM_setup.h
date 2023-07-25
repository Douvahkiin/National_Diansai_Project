#include "F28x_Project.h"
#include "math.h"

#ifndef _EPWM_SET_
#define _EPWM_SET_

#define PWM_MAX_COUNT 5000
#define PWM_MID_COUNT (PWM_MAX_COUNT / 2)

void ConfigureEPWM(void);
void changeCMP_phase(float32 wt);
void changeCMP_value(float32 val);
void disableEpwm1Gpio(void);
void disableEpwm2Gpio(void);
void enableEpwm1Gpio(void);
void enableEpwm2Gpio(void);
void changeCMP_EPWM1_phase(float32 wt);
void changeCMP_EPWM2_phase(float32 wt);
void changeCMP_EPWM3_phase(float32 wt);
#endif  // !_EPWM_SET_
