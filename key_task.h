#ifndef _KEY_TASK_
#define _KEY_TASK_

#include "F28x_Project.h"
#include "FreeRTOS.h"
#include "task.h"

//
// KeyTask - handles all user interaction at task level.
//
//  - polls the mode switches (GPIO124/125/29) at 20 ms like the original
//    modeChange() did in the super-loop;
//  - consumes the XINT1..5 event-group bits set by the ISRs and applies the
//    same logic the original xint1..5_isr() did;
//  - every 32-bit shared value it writes inside taskENTER_CRITICAL so the
//    ControlTask never observes a torn read.
//
void key_create_task(void);
void key_task(void *pvParameters);

//
// Selected display digit / parameter (0..7), shared with UITask.
//
extern volatile int digitPos;

#endif  // _KEY_TASK_
