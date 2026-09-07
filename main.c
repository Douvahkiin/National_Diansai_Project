//
// Included Files
//
#include "ADC_setup.h"
#include "DAC_setup.h"
#include "EPWM_setup.h"
#include "F28x_Project.h"
#include "MACRO.h"
#include "OLED.h"
#include "app_tasks.h"
#include "control_task.h"
#include "isr.h"
#include "keys.h"
#include "utils.h"

//
// Function Prototypes (ISRs live in isr.c, included via isr.h)
//

//
// Globals
//
// (MMOODDEE is defined in utils.c, as in the original project)
//

// main - Hard-real-time 20 kHz control loop is now a FreeRTOS task
// (ControlTask).  This function only performs device init, creates the tasks
// and starts the scheduler.
//
void main(void) {
  // Initialize System Control: PLL, WatchDog, enable Peripheral Clocks
  InitSysCtrl();

  // Initialize GPIO:
  InitGpio();
  InitEPwm1Gpio();
  InitEPwm2Gpio();
  InitEPwm3Gpio();
  InitEPwm4Gpio();
  ConfigureDAC();

  // GPIO22为“探针”，在每次中断开始时toggle一次，接在示波器上可用于判断是否有足够的算力
  // GPIO0与GPIO2为MOS管驱动的使能信号。低电位失能，高电位使能。
  EALLOW;
  GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;    // Enable pullup on GPIO22
  GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;   // GPIO22 = GPIO22
  GpioCtrlRegs.GPADIR.bit.GPIO22 = 1;    // GPIO22 = output
  GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;  // Load output latch

  GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;    // Enable pullup on GPIO0
  GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;   // GPIO0 = GPIO0
  GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;    // GPIO0 = output
  GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;  // Load output latch

  GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;    // Enable pullup on GPIO2
  GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;   // GPIO2 = GPIO2
  GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;    // GPIO2 = output
  GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;  // Load output latch

  // GPIO24为RTOS端到端路径探针: adca1_isr进入时置位,
  // ControlTask完成PWM写入后清零. 高电平宽度=全部时延.
  GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;   // Enable pullup on GPIO24
  GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0;  // GPIO24 = GPIO24
  GpioCtrlRegs.GPADIR.bit.GPIO24 = 1;   // GPIO24 = output
  GpioDataRegs.GPACLEAR.bit.GPIO24 = 1; // Load output latch
  EDIS;

  // Clear all interrupts and initialize PIE vector table: Disable CPU interrupts
  DINT;

  // Initialize the PIE control registers to their default state.
  // The default state is all PIE interrupts disabled and flags are cleared.
  InitPieCtrl();

  // Disable CPU interrupts and clear all CPU interrupt flags:
  IER = 0x0000;
  IFR = 0x0000;

  // Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR).
  InitPieVectTable();

  // Map ISR functions
  EALLOW;
  PieVectTable.ADCA1_INT = &adca1_isr;  // function for ADCA interrupt 1
  PieVectTable.XINT1_INT = &xint1_isr;
  PieVectTable.XINT2_INT = &xint2_isr;
  PieVectTable.XINT3_INT = &xint3_isr;
  PieVectTable.XINT4_INT = &xint4_isr;
  PieVectTable.XINT5_INT = &xint5_isr;
  EDIS;

  // Configure the ADC and power it up
  ConfigureADC();

  // Configure the ePWM
  ConfigureEPWM();

  // Setup the ADC for ePWM triggered conversions
  SetupADCEpwm();

  // Enable global Interrupts and higher priority real-time debug events:
  IER |= M_INT1;   // ADC_A1 + XINT1/2 + FreeRTOS yield
  IER |= M_INT12;  // XINT3/4/5
  IER |= M_INT14;  // FreeRTOS tick interrupt (CPU Timer 2)
  EINT;            // Enable Global interrupt INTM
  ERTM;            // Enable Global realtime interrupt DBGM

  // enable PIE interrupt
  PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
  PieCtrlRegs.PIEIER1.bit.INTx4 = 1;   // Enable PIE Group 1 INT4
  PieCtrlRegs.PIEIER1.bit.INTx5 = 1;   // Enable PIE Group 1 INT5
  PieCtrlRegs.PIEIER12.bit.INTx1 = 1;  // XINT3
  PieCtrlRegs.PIEIER12.bit.INTx2 = 1;  // XINT4
  PieCtrlRegs.PIEIER12.bit.INTx3 = 1;  // XINT5

  // sync ePWM
  EALLOW;
  CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
  EDIS;

  // configure keys
  configure_keys();

  // init OLED
  OLED_Init();
  OLED_ShowString(0, 0, "1919810", 16, 1);
  OLED_ShowString(0, 16, "114514", 16, 1);
  OLED_Refresh();

  //
  // Control algorithms boot-time init (same as the old adca1_isr setup).
  //
  control_task_init();

  //
  // Unfreeze PWM and start conversions - the 20 kHz sampling line from here
  // on is entirely driven by the ADC ISR -> ControlTask.
  //
  EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // unfreeze, and enter updown count mode
  EPwm1Regs.ETSEL.bit.SOCAEN = 1;                 // enable SOCA
  EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
  EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
  EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;

  //
  // Create all kernel objects (static allocation) and start the scheduler.
  //
  create_app_tasks();

  vTaskStartScheduler();

  //
  // The scheduler never returns.  Reach this point only on a fatal failure.
  //
  while (1) {
  }
}
