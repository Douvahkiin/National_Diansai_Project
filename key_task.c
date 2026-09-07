#include "key_task.h"
#include "app_tasks.h"
#include "control_task.h"
#include "MACRO.h"
#include "math.h"
#include "pid.h"
#include "utils.h"

//
// Original ISR reset the PLL output limiter integral when toggling modes.
//
extern struct _pid pid_n2;

//
// KeyTask static stack/TCB.
//
#define KEY_STACK_WORDS 256
static StackType_t uxKeyTaskStack[KEY_STACK_WORDS];
#pragma DATA_SECTION(uxKeyTaskStack, ".freertosStaticStack")
#pragma DATA_ALIGN(uxKeyTaskStack, 2)
static StaticTask_t xKeyTaskTCB;

//
// Digit select state (original main.c: digitPos cycled 0..7).
//
volatile int digitPos = 1;

void key_create_task(void) {
  xKeyTaskHandle = xTaskCreateStatic(key_task, "key", KEY_STACK_WORDS, NULL, TASK_PRIO_KEY, uxKeyTaskStack, &xKeyTaskTCB);
}

void key_task(void *pvParameters) {
  uint32_t bits = 0;

  (void)pvParameters;

  for (;;) {
    //
    // 1. Poll the mode-select switches (GPIO124/125/29).
    //    Same semantics as the original modeChange() in the super-loop.
    //
    modeChange();

    //
    // 2. Wait for XINT key notifications, or a 20 ms timer tick.
    //    Bits are OR-ed into the notification value by the ISRs.
    //
    xTaskNotifyWait(0, KEY_EV_ALL, &bits, pdMS_TO_TICKS(20));

    //
    // 3. Apply the original ISR logic - now in task context.
    //
    if (bits & KEY_EV_XINT1) {
      //
      // 校零: 记录低通滤波后的均值作为新的参考
      //
      taskENTER_CRITICAL();
      Uref_u2 = ADCAResult14_mean;
      Uref_i = ADCBResult3_mean;
      taskEXIT_CRITICAL();
    }

    if (bits & KEY_EV_XINT2) {
      taskENTER_CRITICAL();
      if (MMOODDEE == 1) {
        b2 = !b2;
        pid_n2.integral = 0;
      } else if (MMOODDEE == 2) {
        if (INVERTER_NO == 1) {
          b2 = 1;
          std_U2 = 0;
          GpioDataRegs.GPASET.bit.GPIO0 = 1;
          GpioDataRegs.GPASET.bit.GPIO2 = 1;
        } else if (INVERTER_NO == 2) {
          b4 = 1;
          b3 = 0;
          b2 = 0;
          b1 = 0;
        }
      } else if (MMOODDEE == 3) {
        b4 = !b4;
        b3 = 0;
        b2 = 0;
        b1 = 0;
        time_elapsed = 0;
      }
      taskEXIT_CRITICAL();
    }

    if (bits & KEY_EV_XINT3) {
      if (MMOODDEE == 2) {
        taskENTER_CRITICAL();
        if (INVERTER_NO == 1) {
          b2 = 0;
          std_U2 = 0;
          GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;
          GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
        } else if (INVERTER_NO == 2) {
          b4 = 0;
          b3 = 0;
          b2 = 0;
          b1 = 0;
        }
        taskEXIT_CRITICAL();
      }

      if (MMOODDEE == 3) {
        taskENTER_CRITICAL();
        digitPos++;
        digitPos %= 8;
        taskEXIT_CRITICAL();
      }
    }

    if (bits & KEY_EV_XINT4) {
      if (MMOODDEE == 3) {
        // minus
        taskENTER_CRITICAL();
        if (digitPos < 4) {
          inverter_std_Io -= powf(10, -digitPos);
        } else {
          inverter_K -= powf(10, -(digitPos - 4));
        }
        taskEXIT_CRITICAL();
      }
    }

    if (bits & KEY_EV_XINT5) {
      if (MMOODDEE == 3) {
        // plus
        taskENTER_CRITICAL();
        if (digitPos < 4) {
          inverter_std_Io += powf(10, -digitPos);
        } else {
          inverter_K += powf(10, -(digitPos - 4));
        }
        taskEXIT_CRITICAL();
      }
    }
  }
}
