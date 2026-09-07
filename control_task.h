#ifndef _CONTROL_TASK_
#define _CONTROL_TASK_

#include "F28x_Project.h"
#include "MACRO.h"
#include "FreeRTOS.h"
#include "task.h"

//
// ControlTask: 20 kHz real-time control loop (PLL / PR / PID).
// ADC ISR samples and notifies; this task performs every control
// calculation formerly placed inside adca1_isr.
//

//
// Shared state (written by IF-task/KeyTask, read by ControlTask).
// All 32-bit accesses are protected by critical sections - see writers.
//
extern volatile int MMOODDEE;   // defined in utils.c
extern bool b1, b2, b3, b4;
extern float32 Uref_u2;
extern float32 Uref_i;
extern float32 std_U2;
extern float32 time_elapsed;
extern float32 inverter_std_Io;
extern float32 inverter_K;
extern float32 inverter_std_Io1;
extern float32 inverter_std_Io2;
extern float32 inverter_std_I;
extern float32 ADCAResult14_mean;
extern float32 ADCBResult3_mean;
extern float32 U2_result[BUFFER_SIZE];
extern float32 ig_result[BUFFER_SIZE];

//
// Error counter: set when one control period exceeded 45 us (trip).
//
extern volatile Uint16 control_overrun_cnt;

//
// Sampling results produced by the ADC ISR for the current frame
// (single writer: adca1_isr; single reader: ControlTask).
//
extern volatile Uint16 adc_isr_a14;
extern volatile Uint16 adc_isr_b1;
extern volatile Uint16 adc_isr_frame_index;
extern volatile Uint32 adc_isr_tick0;

void control_task_init(void);
void control_create_task(void);
void control_task(void *pvParameters);

//
// One control step, called by ControlTask.  Raw ADC words are the
// samples captured by adca1_isr for this frame.
//
void control_step(Uint16 adca14, Uint16 adcb1, Uint16 frameIndex);

#endif  // _CONTROL_TASK_
