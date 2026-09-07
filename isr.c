#include "isr.h"
#include "app_tasks.h"
#include "control_task.h"
#include "MACRO.h"
#include "FreeRTOS.h"
#include "task.h"

//
// Frame data produced by adca1_isr (single writer - ISR context).
//
volatile Uint16 adc_isr_a14 = 0;
volatile Uint16 adc_isr_b1 = 0;
volatile Uint16 adc_isr_frame_index = 0;
volatile Uint32 adc_isr_tick0 = 0;

Uint16 frameIndex = 0;  // ring-buffer index over the 16-frame windows

//
// adca1_isr - Read ADC buffer in ISR, then hand the frame over to the
// ControlTask.  All control math moved into control_step().
//
interrupt void adca1_isr(void) {
  while (AdcaRegs.ADCINTFLG.bit.ADCINT1 == 0 || AdcbRegs.ADCINTFLG.bit.ADCINT1 == 0 || AdccRegs.ADCINTFLG.bit.ADCINT1 == 0) {
  }
  AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
  AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
  AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

  GpioDataRegs.GPATOGGLE.bit.GPIO22 = 1;   // 原“算力探针”位置保持不变 (ISR 入口)

  adc_isr_tick0 = CpuTimer0Regs.TIM.all;
  GpioDataRegs.GPASET.bit.GPIO24 = 1;      // RTOS 路径探针: 高电平宽度 = ISR 入口到 PWM 写入

  adc_isr_a14 = AdcaResultRegs.ADCRESULT14;
  adc_isr_b1 = AdcbResultRegs.ADCRESULT1;
  adc_isr_frame_index = frameIndex;

  //
  // Frame advance (original main.c indexing).
  //
  frameIndex++;
  frameIndex %= BUFFER_SIZE;

  //
  // Check if overflow has occurred
  //
  if (1 == AdcaRegs.ADCINTOVF.bit.ADCINT1) {
    AdcaRegs.ADCINTOVFCLR.bit.ADCINT1 = 1;  // clear INT1 overflow flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  // clear INT1 flag
  }
  if (1 == AdcbRegs.ADCINTOVF.bit.ADCINT1) {
    AdcbRegs.ADCINTOVFCLR.bit.ADCINT1 = 1;  // clear INT1 overflow flag
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  // clear INT1 flag
  }

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;  // 该组其他中断才能再次触发
  vTaskNotifyGiveFromISR(xControlTaskHandle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

//
// xint1..5_isr - key handling reduced to task notifications; the actual
// logic (mode switching, digit adjusting, start/stop) runs in KeyTask.
//
interrupt void xint1_isr(void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xTaskNotifyFromISR(xKeyTaskHandle, KEY_EV_XINT1, eSetBits, &xHigherPriorityTaskWoken);

  // 差点忘了这个! 没有这个的话, 这个以及其它同组的中断都不会再被触发了
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

interrupt void xint2_isr(void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xTaskNotifyFromISR(xKeyTaskHandle, KEY_EV_XINT2, eSetBits, &xHigherPriorityTaskWoken);

  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

interrupt void xint3_isr(void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xTaskNotifyFromISR(xKeyTaskHandle, KEY_EV_XINT3, eSetBits, &xHigherPriorityTaskWoken);

  PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

interrupt void xint4_isr(void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xTaskNotifyFromISR(xKeyTaskHandle, KEY_EV_XINT4, eSetBits, &xHigherPriorityTaskWoken);

  PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

interrupt void xint5_isr(void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xTaskNotifyFromISR(xKeyTaskHandle, KEY_EV_XINT5, eSetBits, &xHigherPriorityTaskWoken);

  PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
