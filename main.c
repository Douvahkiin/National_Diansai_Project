//
// Included Files
//
#include "ADC_setup.h"
#include "DAC_setup.h"
#include "EPWM_setup.h"
#include "F28x_Project.h"
#include "MACRO.h"
#include "filters.h"
#include "math.h"
#include "pid.h"
#include "pll.h"
#include "pr.h"

//
// Function Prototypes
//
interrupt void adca1_isr(void);

//
// externs
//
extern struct _pr pr1;
extern struct _pr pr2;
extern struct _pid pid_n1;
extern struct _pid pid_n2;
extern struct _pll pll;

//
// Globals
//
Uint16 ADCAResults0[BUFFER_SIZE];
float32 ADCAResults0_converted[BUFFER_SIZE];
Uint16 ADCAResults1[BUFFER_SIZE];
float32 ADCAResults1_converted[BUFFER_SIZE];
Uint16 ADCAResults2[BUFFER_SIZE];
float32 ADCAResults2_converted[BUFFER_SIZE];

Uint16 ADCBResults0[BUFFER_SIZE];
float32 ADCBResults0_converted[BUFFER_SIZE];
Uint16 ADCBResults1[BUFFER_SIZE];
float32 ADCBResults1_converted[BUFFER_SIZE];

Uint16 ADCCResults0[BUFFER_SIZE];
float32 ADCCResults0_converted[BUFFER_SIZE];

float32 wt = 0;
float32 wt1 = 0;
float32 wt2 = 0;
float32 wt3 = 0;

Uint16 frameIndex;
Uint16 largeIndex;

volatile Uint16 bufferFull;

float32 Uref_u2 = 1.044;
float32 K_u2 = 140;
float32 Uref_i = 1.777;
float32 K_i = 3.5;
float32 Uref_udc = 1.044;
float32 K_udc = 140;
float32 std_ig;
float32 Udc;
float32 std_Udc = 10;

float32 U2_result[BUFFER_SIZE];
float32 Udc_result[BUFFER_SIZE];
float32 ig_result[BUFFER_SIZE];
float32 pll_result;
float32 pid_n1_out;
float32 err1;
float32 err2;
float32 pr1_out;
float32 pr2_out;

float32 alpha1 = 1;
float32 alpha2 = 1;
float32 alpha3 = 1;
float32 alpha4 = 1;

float32 outputPre1 = 0;
float32 outputPre2 = 0;
float32 outputPre3 = 0;
float32 outputPre4 = 0;

float32 inverter_std_I = 1;
float32 inverter_std_U2 = 21.2132;
float32 rectifier_std_I = 5;

/* 启动判断的相关变量 */
bool b1;
bool b2;
bool b3 = 0;
bool b4 = 0;

void main(void) {
  // Initialize System Control: PLL, WatchDog, enable Peripheral Clocks
  InitSysCtrl();

  // Initialize GPIO:
  InitGpio();
  InitEPwm1Gpio();
  InitEPwm2Gpio();
  InitEPwm3Gpio();
  ConfigureDAC();
  //
  // Enable an GPIO output on GPIO22, set it high/low
  //
  EALLOW;
  GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;    // Enable pullup on GPIO22
  GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;   // GPIO22 = GPIO22
  GpioCtrlRegs.GPADIR.bit.GPIO22 = 1;    // GPIO22 = output
  GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;  // Load output latch

  GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;   // Enable pullup on GPIO1
  GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;  // GPIO1 = GPIO1
  GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;   // GPIO1 = output
  GpioDataRegs.GPASET.bit.GPIO1 = 1;   // Load output latch

  GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;   // Enable pullup on GPIO3
  GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;  // GPIO3 = GPIO3
  GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;   // GPIO3 = output
  GpioDataRegs.GPASET.bit.GPIO3 = 1;   // Load output latch

  GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;   // Enable pullup on GPIO5
  GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;  // GPIO5 = GPIO5
  GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;   // GPIO5 = output
  GpioDataRegs.GPASET.bit.GPIO5 = 1;   // Load output latch
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
  EDIS;

  // Configure the ADC and power it up
  ConfigureADC();

  // Configure the ePWM
  ConfigureEPWM();

  // Setup the ADC for ePWM triggered conversions
  SetupADCEpwm();

  // Enable global Interrupts and higher priority real-time debug events:
  IER |= M_INT1;  // Enable group 1 interrupts
  EINT;           // Enable Global interrupt INTM
  ERTM;           // Enable Global realtime interrupt DBGM

  frameIndex = 0;
  largeIndex = 0;
  bufferFull = 0;

  // enable PIE interrupt
  PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

  // sync ePWM
  EALLOW;
  CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;

  // pll, pid init
  pll_Init(2 * PI * 50, 2);  // 50Hz
  pid_nx_Init(1, 0, 0, &pid_n1);

  //
  // pr1 init
  //
  /* pr init, Ts = 0.0001*/
  // pr_init(1, -1.9928, 0.99374, 1.3131, -1.9928, 0.68064, &pr1);  // p=1, r=100
  // pr_init(1, -1.9928, 0.99374, 1.1565, -1.9928, 0.83719, &pr1);  // p=1, r=50
  // pr_init(1, -1.9928, 0.99374, 1.0313, -1.9928, 0.96243, &pr1);  // p=1, r=10
  // pr_init(1, -1.9928, 0.99374, 1.0157, -1.9928, 0.97808, &pr1);  // p=1, r=5

  /* pr_init, Ts = 0.00005*/
  // pr_init(1, -1.9966, 0.99686, 1.0016, -1.9966, 0.99530, &pr1);  // p=1, r=1
  // pr_init(1, -1.9966, 0.99686, 1.0078, -1.9966, 0.98902, &pr1);  // p=1, r=5
  // pr_init(1, -1.9966, 0.99686, 1.0157, -1.9966, 0.98118, &pr1);  // p=1, r=10
  // pr_init(1, -1.9966, 0.99686, 1.0314, -1.9966, 0.96550, &pr1);  // p=1, r=20
  // pr_init(1, -1.9966, 0.99686, 0.53136, -0.99831, 0.46707, &pr1);  // p=0.5, r=20
  // pr_init(1, -1.9966, 0.99686, 0.13136, -0.19966, 0.068322, &pr1);  // p=0.1, r=20
  // pr_init(1, -1.9966, 0.99686, 0.10784, -0.19966, 0.091845, &pr1);  // p=0.1, r=5
  // pr_init(1, -1.9966, 0.99686, 0.50784, -0.99831, 0.49059, &pr1);  // p=0.5, r=5
  pr_init(1, -1.9966, 0.99686, 0.20784, -0.39932, 0.19153, &pr1);  // p=0.2, r=5

  //
  // pr2 init
  //
  // pr_init(1, -1.9966, 0.99686, 1.0016, -1.9966, 0.99530, &pr2);  // p=1, r=1
  // pr_init(1, -1.9966, 0.99686, 1.0078, -1.9966, 0.98902, &pr2);  // p=1, r=5
  // pr_init(1, -1.9966, 0.99686, 1.0157, -1.9966, 0.98118, &pr2);  // p=1, r=10
  // pr_init(1, -1.9966, 0.99686, 1.0314, -1.9966, 0.96550, &pr2);  // p=1, r=20
  // pr_init(1, -1.9966, 0.99686, 0.53136, -0.99831, 0.46707, &pr2);  // p=0.5, r=20
  // pr_init(1, -1.9966, 0.99686, 0.13136, -0.19966, 0.068322, &pr2);  // p=0.1, r=20
  // pr_init(1, -1.9966, 0.99686, 0.10784, -0.19966, 0.091845, &pr2);  // p=0.1, r=5
  // pr_init(1, -1.9966, 0.99686, 0.50784, -0.99831, 0.49059, &pr2);  // p=0.5, r=5
  pr_init(1, -1.9966, 0.99686, 0.20784, -0.39932, 0.19153, &pr2);  // p=0.2, r=5

  wt1 = 0;
  wt2 = -2 * PI / 3;
  wt3 = 2 * PI / 3;

  // take conversions indefinitely in loop
  EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // unfreeze, and enter updown count mode
  EPwm1Regs.ETSEL.bit.SOCAEN = 1;                 // enable SOCA
  EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // unfreeze, and enter updown count mode
  EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // unfreeze, and enter updown count mode
  do {
    // wait while ePWM causes ADC conversions, which then cause interrupts,
    // which fill the results buffer, eventually setting the bufferFull flag
    while (!bufferFull) {
    }
    bufferFull = 0;  // clear the buffer full flag

    // software breakpoint, hit run again to get updated conversions
    // asm("   ESTOP0");
  } while (1);
}

//
// adca1_isr - Read ADC Buffer in ISR
//
interrupt void adca1_isr(void) {
  while (AdcaRegs.ADCINTFLG.bit.ADCINT1 == 0 || AdcbRegs.ADCINTFLG.bit.ADCINT1 == 0 || AdccRegs.ADCINTFLG.bit.ADCINT1 == 0) {
  }
  AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
  AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
  AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

  ADCAResults0[frameIndex] = AdcaResultRegs.ADCRESULT0;
  ADCAResults0_converted[frameIndex] = ADCAResults0[frameIndex] * 3.0 / 4096.0;
  ADCAResults1[frameIndex] = AdcaResultRegs.ADCRESULT1;
  ADCAResults1_converted[frameIndex] = ADCAResults1[frameIndex] * 3.0 / 4096.0;
  ADCAResults2[frameIndex] = AdcaResultRegs.ADCRESULT14;
  ADCAResults2_converted[frameIndex] = ADCAResults2[frameIndex] * 3.0 / 4096.0;
  // changeDACAVal(ADCAResults2[frameIndex]);

  ADCBResults0[frameIndex] = AdcbResultRegs.ADCRESULT0;
  ADCBResults0_converted[frameIndex] = ADCBResults0[frameIndex] * 3.0 / 4096.0;
  ADCBResults1[frameIndex] = AdcbResultRegs.ADCRESULT1;
  ADCBResults1_converted[frameIndex] = ADCBResults1[frameIndex] * 3.0 / 4096.0;

  ADCCResults0[frameIndex] = AdccResultRegs.ADCRESULT0;
  ADCCResults0_converted[frameIndex] = ADCCResults0[frameIndex] * 3.0 / 4096.0;

  ADCAResults0_converted[frameIndex] = low_pass_filter(ADCAResults0_converted[frameIndex], &outputPre1, alpha1);
  ADCAResults1_converted[frameIndex] = low_pass_filter(ADCAResults1_converted[frameIndex], &outputPre2, alpha2);
  // ADCAResults2[frameIndex] = low_pass_filter(ADCAResults2[frameIndex], &outputPre2, alpha2);
  ADCBResults0_converted[frameIndex] = low_pass_filter(ADCBResults0_converted[frameIndex], &outputPre3, alpha3);
  ADCBResults1_converted[frameIndex] = low_pass_filter(ADCBResults1_converted[frameIndex], &outputPre4, alpha4);

  /* 这是周期为50Hz的正弦波表示 */
  wt = wt + PI / 100 / 2 * SW_FREQ;
  if (wt > PI * 2) wt -= PI * 2;

  U2_result[frameIndex] = (ADCAResults2_converted[frameIndex] - Uref_u2) * K_u2;
  ig_result[frameIndex] = -(ADCBResults1_converted[frameIndex] - Uref_i) * K_i;
  Udc_result[frameIndex] = (ADCCResults0_converted[frameIndex] - Uref_udc) * K_udc;

  // pll input 为交流侧电压
  // float32 pll_input = U2_result[frameIndex];
  float32 pll_input = inverter_std_U2 * sin(wt);
  // pll 的结果
  pll_result = pll_Run(pll_input);
  // 用正弦便于判断正确
  pll_result = cos(pll_result);
  changeDACBVal(2048 + 2000.0 * sin(wt));
  changeDACAVal(2048 + 2000.0 * pll_result);

  // /* PR控制器启动判断, 启动后变量 b2 自锁 */
  // b1 = fabsf(U2_result[frameIndex]) >= 5;
  // b2 = b1 || b3;
  // b3 = b2;

  //
  // 交流电压环
  //
  err1 = sin(wt) * inverter_std_U2 - U2_result[frameIndex];
  float32 pr1_input = err1;
  pr1_out = pr_run(pr1_input, &pr1);

  //
  // 交流电流环
  //
  // err2 = sin(wt) * inverter_std_I - ig_result[frameIndex];
  err2 = pr1_out - ig_result[frameIndex];
  float32 pr2_input = err2;
  pr2_out = pr_run(pr2_input, &pr2);

  float32 pid_n1_input = err2;
  pid_n1_out = pid_nx_Run(pid_n1_input, &pid_n1);

  //
  // change PWM duty
  //
  // changeCMP_value(pr1_out);
  changeCMP_value(pr2_out);
  // changeCMP_value(pid_n1_out);
  // changeCMP_phase(wt);
  // changeCMP_value(0.8);

  frameIndex++;
  largeIndex++;
  largeIndex %= LARGE_BUFFER;
  if (BUFFER_SIZE <= frameIndex) {
    frameIndex = 0;
    bufferFull = 1;
  }
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

  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//
// End of file
//
