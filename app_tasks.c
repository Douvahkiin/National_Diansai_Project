#include "app_tasks.h"
#include "control_task.h"
#include "key_task.h"
#include "ui_task.h"
#include "F28x_Project.h"

//
// FreeRTOS kernel object handles (static allocation).
//
TaskHandle_t xControlTaskHandle = NULL;
TaskHandle_t xKeyTaskHandle = NULL;
TaskHandle_t xUITaskHandle = NULL;

//
// create_app_tasks - create each task with its own static stack/TCB and then
// set up the ISR->task notification path.  Called once by main() while the
// scheduler is not running.  Tasks use static memory only (no pvPortMalloc
// in this project).
//
void create_app_tasks(void) {
  xControlTaskHandle = NULL;
  xKeyTaskHandle = NULL;
  xUITaskHandle = NULL;

  control_create_task();
  key_create_task();
  ui_create_task();
}

//
// Idle task memory (must live in lower 64K - C28x SP window).
//
static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];
#pragma DATA_SECTION(uxIdleTaskStack, ".freertosStaticStack")
#pragma DATA_ALIGN(uxIdleTaskStack, 2)

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   configSTACK_DEPTH_TYPE *pulIdleTaskStackSize) {
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
  *ppxIdleTaskStackBuffer = uxIdleTaskStack;
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

//
// Application hooks (configUSE_IDLE_HOOK / configCHECK_FOR_STACK_OVERFLOW).
//
void vApplicationIdleHook(void) {
  //
  // Lowest priority runner - keep the CPU idle here.
  //
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
  //
  // Stack overflow: shut the MOSFET enable pins down and stay dead.
  //
  (void)xTask;
  (void)pcTaskName;
  GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;
  GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
  for (;;) {
  }
}
