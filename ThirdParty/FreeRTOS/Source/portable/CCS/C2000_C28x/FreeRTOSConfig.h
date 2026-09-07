/*
 * FreeRTOSConfig.h for TI C2000 C28x (F28379D, 100 MHz SYSCLKOUT).
 *
 * - Static allocation only: every task / kernel object has fixed memory.
 * - Tick: CPU Timer 2, 1 kHz.
 * - Preemptive scheduling; priorities: idle(0) < UI(1) < Key(3) < Control(4).
 */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*
 * Fixed-width types used by the FreeRTOS kernel.
 *
 * The C2000 compiler's stdint.h deliberately does NOT define int8_t/
 * uint8_t: on C28x a "char" is 16 bits and TI's standard headers only
 * provide 16/32/64-bit types.  FreeRTOS (and TI's own C28x demos, which
 * get them from driverlib) expects these anyway - map 8-bit to 16-bit
 * char like TI-C2000 always does.
 */
#include <stdint.h>
#ifndef __C28X_FREERTOS_INT8_TYPES__
#define __C28X_FREERTOS_INT8_TYPES__
typedef unsigned char uint8_t;
typedef char int8_t;
#endif

#define configUSE_PREEMPTION                1
#define configUSE_IDLE_HOOK                 1
#define configUSE_TICK_HOOK                 0
#define configCPU_CLOCK_HZ                  ( ( unsigned long ) 100000000 )
#define configTICK_RATE_HZ                  ( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES                ( 5 )
#define configMINIMAL_STACK_SIZE            ( ( unsigned short ) 128 )
#define configTOTAL_HEAP_SIZE               ( ( size_t ) ( 0 ) )
#define configMAX_TASK_NAME_LEN             ( 16 )
#define configUSE_TRACE_FACILITY            0
#define configUSE_16_BIT_TICKS              0
#define configIDLE_SHOULD_YIELD             0
#define configCHECK_FOR_STACK_OVERFLOW      2
#define configSUPPORT_STATIC_ALLOCATION     1
#define configSUPPORT_DYNAMIC_ALLOCATION    0
#define configUSE_TASK_NOTIFICATIONS        1
#define configUSE_EVENT_GROUPS              1

// API selection.
#define INCLUDE_vTaskPrioritySet            0
#define INCLUDE_uxTaskPriorityGet           0
#define INCLUDE_vTaskDelete                 0
#define INCLUDE_vTaskCleanUpResources       0
#define INCLUDE_vTaskSuspend                0
#define INCLUDE_vTaskDelayUntil             1
#define INCLUDE_vTaskDelay                  1

#endif /* FREERTOS_CONFIG_H */
