#ifndef _APP_TASKS_
#define _APP_TASKS_

#include "FreeRTOS.h"
#include "task.h"

//
// Task layout (priority: idle(0) < UI(1) < Key(3) < Control(4))
//
#define TASK_PRIO_UI        1
#define TASK_PRIO_KEY       3
#define TASK_PRIO_CONTROL   4

extern TaskHandle_t xControlTaskHandle;
extern TaskHandle_t xKeyTaskHandle;
extern TaskHandle_t xUITaskHandle;

//
// Key event bits (set in xint isr via task notification, read by KeyTask)
//
#define KEY_EV_XINT1        ( 1UL << 0 )   // 校零: 记录 ADCAResult14_mean / ADCBResult3_mean
#define KEY_EV_XINT2        ( 1UL << 1 )   // 启停 / 模式切换
#define KEY_EV_XINT3        ( 1UL << 2 )   // 停止 / 选位
#define KEY_EV_XINT4        ( 1UL << 3 )   // 参数-
#define KEY_EV_XINT5        ( 1UL << 4 )   // 参数+
#define KEY_EV_ALL          ( KEY_EV_XINT1 | KEY_EV_XINT2 | KEY_EV_XINT3 | KEY_EV_XINT4 | KEY_EV_XINT5 )

void create_app_tasks(void);   // 静态创建全部内核对象, 在 vTaskStartScheduler 之前调用

#endif  // _APP_TASKS_
