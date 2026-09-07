#ifndef _UI_TASK_
#define _UI_TASK_

#include "F28x_Project.h"
#include "FreeRTOS.h"
#include "task.h"

//
// UITask - OLED display refresh at 10 Hz.  Replaces the original super-loop
// body (OLED_Refresh + DELAY_US(100000)) with a vTaskDelayUntil() periodic
// task.  All 32-bit reads from shared parameters are wrapped in critical
// sections to avoid torn values.
//
void ui_create_task(void);
void ui_task(void *pvParameters);

#endif  // _UI_TASK_
