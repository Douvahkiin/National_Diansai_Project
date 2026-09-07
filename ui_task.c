#include "ui_task.h"
#include "app_tasks.h"
#include "control_task.h"
#include "key_task.h"
#include "MACRO.h"
#include "OLED.h"
#include "utils.h"

//
// UITask static stack/TCB.
//
#define UI_STACK_WORDS 512
static StackType_t uxUITaskStack[UI_STACK_WORDS];
#pragma DATA_SECTION(uxUITaskStack, ".freertosStaticStack")
#pragma DATA_ALIGN(uxUITaskStack, 2)
static StaticTask_t xUITaskTCB;

void ui_create_task(void) {
  xUITaskHandle = xTaskCreateStatic(ui_task, "ui", UI_STACK_WORDS, NULL, TASK_PRIO_UI, uxUITaskStack, &xUITaskTCB);
}

void ui_task(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  unsigned char s1[16] = {0};
  unsigned char s_mode[3] = "  ";
  int map[4] = {0, 2, 3, 4};
  int map2[4] = {7, 9, 10, 11};
  int Display_numArray[5];
  int Display_numArray2[5];
  bool blink = 1;
  float32 io_val;
  float32 k_val;
  int i;

  (void)pvParameters;

  for (i = 0; i < 16; i++) {
    s1[i] = ' ';
  }

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));

    s_mode[0] = INVERTER_NO + 0x30;
    s_mode[1] = MMOODDEE + 0x30;
    OLED_ClearGRAM();

    placeString(s1, s_mode, 14);

    taskENTER_CRITICAL();
    io_val = inverter_std_Io;
    k_val = inverter_K;
    taskEXIT_CRITICAL();

    float2numarray(io_val, Display_numArray);
    float2numarray(k_val, Display_numArray2);
    numarray2str(s1, Display_numArray);
    numarray2str2(s1, Display_numArray2);

    if (blink) {
      if (digitPos < 4) {
        s1[map[digitPos]] = ' ';
      } else {
        s1[map2[digitPos - 4]] = ' ';
      }
      blink = false;
    } else {
      blink = true;
    }

    OLED_ShowString(0, 0, s1, 16, 1);
    OLED_Refresh();
  }
}
