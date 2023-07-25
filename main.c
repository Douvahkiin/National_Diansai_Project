//
// Included Files
//
#include "ADC_setup.h"
#include "EPWM_setup.h"
#include "F28x_Project.h"
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
// Defines
//
#define BUFFER_SIZE 1024
#define LITTLE_BUFFER 16

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

Uint16 frameIndex;

volatile Uint16 bufferFull;

float32 Uref_u2 = 1.251;
float32 K_u2 = 11;
float32 Uref_i = 1.466;
float32 K_i = 4.175;
float32 Uref_udc = 0;
float32 K_udc = 11;
float32 std_ig;
float32 Udc;
float32 std_Udc = 10;
float32 PWM_Input;
float32 pid_n1_input;
float32 pll_input;

float32 U2_result[BUFFER_SIZE];
float32 Udc_result[BUFFER_SIZE];
float32 ig_result[BUFFER_SIZE];
float32 pll_result[BUFFER_SIZE];
float32 pid_n1_result[LITTLE_BUFFER];
float32 err[LITTLE_BUFFER];
float32 pr_out[LITTLE_BUFFER];

float32 alpha1 = 1;
float32 alpha2 = 1;
float32 alpha3 = 1;
float32 alpha4 = 1;

float32 outputPre1 = 0;
float32 outputPre2 = 0;
float32 outputPre3 = 0;
float32 outputPre4 = 0;

float32 inverter_std_I = 2;
float32 inverter_std_U2 = 10;
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
  //
  // Enable an GPIO output on GPIO22, set it high/low
  //
  EALLOW;
  GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;   // Enable pullup on GPIO22
  GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;  // GPIO22 = GPIO22
  GpioCtrlRegs.GPADIR.bit.GPIO22 = 1;   // GPIO22 = output
  GpioDataRegs.GPASET.bit.GPIO22 = 1;   // Load output latch
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
  bufferFull = 0;

  // enable PIE interrupt
  PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

  // sync ePWM
  EALLOW;
  CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;

  // pll, pid init
  pll_Init(314.1593, 2);     // 50Hz
  pid_n1_Init(0.5, 0.1, 0);  // 直流端电压PI控制
  // pr_init(1, -1.9928, 0.99374, 1.3131, -1.9928, 0.68064);  // p=1, r=100
  // pr_init(1, -1.9928, 0.99374, 1.1565, -1.9928, 0.83719);  // p=1, r=50
  // pr_init(1, -1.9928, 0.99374, 1.0313, -1.9928, 0.96243);  // p=1, r=10
  pr_init(1, -1.9928, 0.99374, 1.0157, -1.9928, 0.97808);  // p=1, r=5

  // disableEpwm1Gpio();
  // disableEpwm2Gpio();
  // take conversions indefinitely in loop
  EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // unfreeze, and enter updown count mode
  EPwm1Regs.ETSEL.bit.SOCAEN = 1;                 // enable SOCA
  EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // unfreeze, and enter updown count mode
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

  // ADCAResults0[frameIndex] = AdcaResultRegs.ADCRESULT0;
  // ADCAResults0_converted[frameIndex] = ADCAResults0[frameIndex] * 3.0 / 4096.0;
  // ADCAResults1[frameIndex] = AdcaResultRegs.ADCRESULT1;
  // ADCAResults1_converted[frameIndex] = ADCAResults1[frameIndex] * 3.0 / 4096.0;
  ADCAResults2[frameIndex] = AdcaResultRegs.ADCRESULT14;
  ADCAResults2_converted[frameIndex] = ADCAResults2[frameIndex] * 3.0 / 4096.0;

  ADCBResults0[frameIndex] = AdcbResultRegs.ADCRESULT0;
  ADCBResults0_converted[frameIndex] = ADCBResults0[frameIndex] * 3.0 / 4096.0;
  // ADCBResults1[frameIndex] = AdcbResultRegs.ADCRESULT1;
  // ADCBResults1_converted[frameIndex] = ADCBResults1[frameIndex] * 3.0 / 4096.0;

  ADCCResults0[frameIndex] = AdccResultRegs.ADCRESULT0;
  ADCCResults0_converted[frameIndex] = ADCCResults0[frameIndex] * 3.0 / 4096.0;

  // ADCAResults0_converted[frameIndex] = low_pass_filter(ADCAResults0_converted[frameIndex], &outputPre1, alpha1);
  // ADCAResults1_converted[frameIndex] = low_pass_filter(ADCAResults1_converted[frameIndex], &outputPre2, alpha2);
  // ADCBResults0_converted[frameIndex] = low_pass_filter(ADCBResults0_converted[frameIndex], &outputPre3, alpha3);
  // ADCBResults1_converted[frameIndex] = low_pass_filter(ADCBResults1_converted[frameIndex], &outputPre4, alpha4);

  /* 这是周期为50Hz的正弦波表示 */
  wt = wt + 0.0314159269;
  if (wt > 3.14159269 * 2) wt -= 3.14159269 * 2;

  U2_result[frameIndex] = (ADCAResults2_converted[frameIndex] - Uref_u2) * K_u2;
  ig_result[frameIndex] = -(ADCBResults0_converted[frameIndex] - Uref_i) * K_i;
  Udc_result[frameIndex] = (ADCCResults0_converted[frameIndex] - Uref_udc) * K_udc;

  // /* 整流器控电压 */
  // float32 std_U2 = inverter_std_U2 * sin(wt);
  // pid_n1_input = std_U2 - U2_result[frameIndex];
  // pid_n1_result[frameIndex] = pid_n1_Run(pid_n1_input);
  // pid_n1_result[frameIndex] = saturation(pid_n1_result[frameIndex], 1, -1);

  // // pll input 为电网电压
  // pll_input = U2_result[frameIndex];
  // // pll 的结果
  // pll_result[frameIndex] = pll_Run(pll_input);
  // // 用正弦便于判断正确
  // pll_result[frameIndex] = cos(pll_result[frameIndex]);

  // /* 对Udc进行PID控制 */
  // pid_n1_input = (std_Udc - Udc_result[frameIndex]);
  // pid_n1_result[frameIndex] = pid_n1_Run(pid_n1_input);
  // pid_n1_result[frameIndex] = saturation(pid_n1_result[frameIndex], 5, -5);

  // /* PR控制器启动判断, 启动后变量 b2 自锁 */
  // b1 = fabsf(U2_result[frameIndex]) >= 5;
  // b2 = b1 || b3;
  // b3 = b2;

  err[frameIndex] = sin(wt) * inverter_std_I - ig_result[frameIndex];  // 未并网, 跟踪软件产生的波
  // // Udc的PID控制输出值作为电流的跟踪幅值
  // // err[frameIndex] = pll_result[frameIndex] * pid_n1_result[frameIndex] - ig_result[frameIndex];  // 已并网, 跟踪电网的波
  // err[frameIndex] = pll_result[frameIndex] * rectifier_std_I - ig_result[frameIndex];  // 已并网, 跟踪电网的波

  float32 pr_input = err[frameIndex];
  pid_n1_input = err[frameIndex];
  pr_out[frameIndex] = pr_run(pr_input);
  pid_n1_result[frameIndex] = pid_n1_Run(pid_n1_input);
  // changeCMP_value(pr_out[frameIndex]);
  // changeCMP_value(pid_n1_result[frameIndex]);
  changeCMP_value(err[frameIndex]);  // pure P
  // if (b2) {                              // b2为真时, 打开PR以及PWM输出
  //   float32 pr_input = err[frameIndex];  // 直接通过 err
  //   pr_out[frameIndex] = pr_run(pr_input);
  //   // pr_out[frameIndex] = pr_input;  // test pure P controller
  //   if (!b4) {
  //     enableEpwm1Gpio();
  //     enableEpwm2Gpio();
  //     b4 = 1;
  //   }
  //   changeCMP_value(pr_out[frameIndex]);
  // } else {
  //   disableEpwm1Gpio();
  //   disableEpwm2Gpio();
  // }

  // changeCMP_value(pid_n1_result[frameIndex]);
  // changeCMP_phase(3 * 3.14159 / 2 * 0.9);
  // changeCMP_phase(wt);
  // changeCMP_value(0.8);
  // GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
  // GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;

  frameIndex++;
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
