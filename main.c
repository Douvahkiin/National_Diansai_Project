//
// Included Files
//
#include "ADC_setup.h"
#include "DAC_setup.h"
#include "EPWM_setup.h"
#include "F28x_Project.h"
#include "MACRO.h"
#include "OLED.h"
#include "filters.h"
#include "keys.h"
#include "math.h"
#include "pid.h"
#include "pll.h"
#include "pr.h"
#include "string.h"
#include "utils.h"

// MODE
extern int MMOODDEE;

//
// Function Prototypes
//
interrupt void adca1_isr(void);
interrupt void xint1_isr(void);
interrupt void xint2_isr(void);
interrupt void xint3_isr(void);
interrupt void xint4_isr(void);
interrupt void xint5_isr(void);

//
// externs
//
extern struct _pr pr1;
extern struct _pr pr2;
extern struct _pr pr3;
extern struct _pr pr4;
extern struct _pr pr_origin;
extern struct _pid pid_n1;
extern struct _pid pid_n2;
extern struct _pll pll1;
extern struct _sogi sogi1;
extern struct _pid pid_pll1;
extern struct _pll pll2;
extern struct _sogi sogi2;
extern struct _pid pid_pll2;

//
// Globals
//
// Uint16 ADCAResults0[BUFFER_SIZE];
// float32 ADCAResults0_converted[BUFFER_SIZE];
// Uint16 ADCAResults2[BUFFER_SIZE];
// float32 ADCAResults2_converted[BUFFER_SIZE];
// Uint16 ADCAResults3[BUFFER_SIZE];
// float32 ADCAResults3_converted[BUFFER_SIZE];
Uint16 ADCAResults14[BUFFER_SIZE];
float32 ADCAResults14_converted[BUFFER_SIZE];
// Uint16 ADCAResults15[BUFFER_SIZE];
// float32 ADCAResults15_converted[BUFFER_SIZE];

// Uint16 ADCBResults2[BUFFER_SIZE];
// float32 ADCBResults2_converted[BUFFER_SIZE];
Uint16 ADCBResults3[BUFFER_SIZE];
float32 ADCBResults3_converted[BUFFER_SIZE];

// Uint16 ADCCResults3[BUFFER_SIZE];
// float32 ADCCResults3_converted[BUFFER_SIZE];

// float32 ADCAResult0_mean = 0;
// float32 ADCAResult2_mean = 0;
// float32 ADCAResult3_mean = 0;
float32 ADCAResult14_mean = 0;
// float32 ADCAResult15_mean = 0;

// float32 ADCBResult2_mean = 0;
float32 ADCBResult3_mean = 0;

// float32 ADCCResult3_mean = 0;

float32 wt = 0;

Uint16 frameIndex;
Uint16 largeIndex;

float32 Uref_u2 = 1.047;
float32 K_u2 = 69.2;
float32 Uref_i = 1.777;
float32 K_i = 3.195;
float32 Uref_udc = 1.044;
float32 K_udc = 140;

float32 U2_result[BUFFER_SIZE];
float32 Udc_result[BUFFER_SIZE];
float32 ig_result[BUFFER_SIZE];
float32 pll_result1;
float32 pll_result2;
float32 pid_n1_out;
float32 err1;
float32 err2;
float32 pr1_out;
float32 pr2_out;

float32 alpha1 = 1;
float32 alpha2 = 1;
float32 alpha3 = 1;
float32 alpha4 = 1;
float32 alpha_for_avg = 0.1;

float32 outputPre1 = 0;
float32 outputPre2 = 0;
float32 outputPre3 = 0;
float32 outputPre4 = 0;
float32 outputPre_A0 = 0;
float32 outputPre_A2 = 0;
float32 outputPre_A3 = 0;
float32 outputPre_A14 = 0;
float32 outputPre_A15 = 0;
float32 outputPre_B2 = 0;
float32 outputPre_B3 = 0;
float32 outputPre_C3 = 0;

float32 rampInterval = 10;  // 10s
float32 inverter_std_Io = 4;
float32 inverter_std_Io1 = 2;
float32 inverter_std_Io2 = 2;
float32 inverter_K = 1;
float32 inverter_std_I = 2.828427;
// float32 inverter_std_I = 2;
// float32 inverter_std_I = 1.41421356;
// float32 inverter_std_I = 1;
float32 inverter_std_I_MODE2 = 2.828427;
float32 inverter_std_I_rms = 2;
float32 inverter_std_U2 = 33.941125;
// float32 inverter_std_U2 = 21.2132034;
// float32 inverter_std_U2 = 14.1421356;  // 10*sqrt(2)
// float32 inverter_std_U2 = 7.0711;
// float32 inverter_std_U2 = 2.828;
// float32 rectifier_std_I = 2;
// float32 rectifier_std_Udc = 30;
// float32 rectifier_std_Udc = 20;
// float32 rectifier_std_Udc = 10;

// float32 triggerV = 180;
// float32 triggerV = 50;
// float32 triggerV = 30;
// float32 triggerV = 18;
float32 triggerV = 16.9705627;  // 12*sqrt(2)
// float32 triggerV = 10;
// float32 triggerV = 7.0711;  // 5*sqrt(2)

int intcount = 0;

float32 pid_n1_limit = 1;
float32 pid_n2_limit = 0.5;

float32 DAADCAL_receiver = 0;

int Display_numArray[5];
int Display_numArray2[5];

float32 U2_d = 0;
float32 ig_q = 0;

int digitPos = 1;

/* 启动判断的相关变量 */
bool b1 = 0;
bool b2 = 0;
bool b3 = 0;
bool b4 = 0;

float32 std_U2 = 0;
float32 time_elapsed = 0;
float32 openLoopRatio = 0.711;

float32 kpp = 0.5;
float32 krr = 60;

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
  IER |= M_INT1;   // Enable group 1 interrupts
  IER |= M_INT12;  // Enable group 12 interrupts
  EINT;            // Enable Global interrupt INTM
  ERTM;            // Enable Global realtime interrupt DBGM

  frameIndex = 0;
  largeIndex = 0;

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

  // value control
  digitPos = 1;

  // init OLED
  OLED_Init();
  OLED_ShowString(0, 0, "1919810", 16, 1);
  OLED_ShowString(0, 16, "114514", 16, 1);
  OLED_Refresh();

  // pll, pid init
  pll_Init(2 * PI * 50, 2, &pll1, &sogi1, &pid_pll1);  // 50Hz
  pll_Init(2 * PI * 50, 2, &pll2, &sogi2, &pid_pll2);  // 50Hz
  pid_nx_Init(0.15, 0, 0, pid_n1_limit, -pid_n1_limit, &pid_n1);
  pid_nx_Init(0.01, 7, 0, pid_n2_limit / 7, -pid_n2_limit / 7, &pid_n2);

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
  pr_init(1, -1.9966, 0.99686, 0.065682, -0.099831, 0.034161, &pr1);  // p=0.05, r=5
  // pr_init(1, -1.9966, 0.99686, 0.032841, -0.049915, 0.01708, &pr1);  // p=0.025, r=5
  // pr_init(1, -1.9966, 0.99686, 0.11568, -0.19966, 0.084004, &pr1);  // p=0.1, r=10
  // pr_init(1, -1.9966, 0.99686, 0.50784, -0.99831, 0.49059, &pr1);  // p=0.5, r=5
  // pr_init(1, -1.9966, 0.99686, 0.20784, -0.39932, 0.19153, &pr1);  // p=0.2, r=5
  // pr_init(1, -1.9966, 0.99686, 0.20157, -0.39932, 0.1978, &pr1);  // p=0.2, r=1

  //
  // pr2 init
  //
  // pr_init(1, -1.9966, 0.99686, 1.0016, -1.9966, 0.99530, &pr2);  // p=1, r=1
  // pr_init(1, -1.9966, 0.99686, 1.0078, -1.9966, 0.98902, &pr2);  // p=1, r=5
  // pr_init(1, -1.9966, 0.99686, 1.0157, -1.9966, 0.98118, &pr2);  // p=1, r=10
  // pr_init(1, -1.9966, 0.99686, 1.0314, -1.9966, 0.96550, &pr2);  // p=1, r=20
  // pr_init(1, -1.9966, 0.99686, 0.53136, -0.99831, 0.46707, &pr2);  // p=0.5, r=20
  // pr_init(1, -1.9966, 0.99686, 0.13136, -0.19966, 0.068322, &pr2);  // p=0.1, r=20
  pr_init(1, -1.9966, 0.99686, 0.10784, -0.19966, 0.091845, &pr2);  // p=0.1, r=5
  // pr_init(1, -1.9966, 0.99686, 0.065682, -0.099831, 0.034161, &pr2);  // p=0.05, r=5
  // pr_init(1, -1.9966, 0.99686, 0.11568, -0.19966, 0.084004, &pr2);  // p=0.1, r=10
  // pr_init(1, -1.9966, 0.99686, 0.50784, -0.99831, 0.49059, &pr2);  // p=0.5, r=5
  // pr_init(1, -1.9966, 0.99686, 0.20784, -0.39932, 0.19153, &pr2);  // p=0.2, r=5
  // pr_init(1, -1.9966, 0.99686, 0.30784, -0.59899, 0.29122, &pr2);  // p=0.3, r=5
  // pr_init(1, -1.9966, 0.99686, 0.20157, -0.39932, 0.1978, &pr2);  // p=0.2, r=1

  //
  // pr3 init
  //
  // pr_init(1, -1.9966, 0.99686, 1.0016, -1.9966, 0.99530, &pr3);  // p=1, r=1
  // pr_init(1, -1.9966, 0.99686, 1.0078, -1.9966, 0.98902, &pr3);  // p=1, r=5
  // pr_init(1, -1.9966, 0.99686, 1.0157, -1.9966, 0.98118, &pr3);  // p=1, r=10
  // pr_init(1, -1.9966, 0.99686, 1.0314, -1.9966, 0.96550, &pr3);  // p=1, r=20
  // pr_init(1, -1.9966, 0.99686, 0.53136, -0.99831, 0.46707, &pr3);  // p=0.5, r=20
  // pr_init(1, -1.9966, 0.99686, 0.13136, -0.19966, 0.068322, &pr3);  // p=0.1, r=20
  pr_init(1, -1.9966, 0.99686, 0.10784, -0.19966, 0.091845, &pr3);  // p=0.1, r=5
  // pr_init(1, -1.9966, 0.99686, 0.065682, -0.099831, 0.034161, &pr3);  // p=0.05, r=5
  // pr_init(1, -1.9966, 0.99686, 0.11568, -0.19966, 0.084004, &pr3);  // p=0.1, r=10
  // pr_init(1, -1.9966, 0.99686, 0.50784, -0.99831, 0.49059, &pr3);  // p=0.5, r=5
  // pr_init(1, -1.9966, 0.99686, 0.20784, -0.39932, 0.19153, &pr3);  // p=0.2, r=5
  // pr_init(1, -1.9966, 0.99686, 0.20157, -0.39932, 0.1978, &pr3);  // p=0.2, r=1

  // pr4 init
  // pr_init(1, -1.9966, 0.99686, 0.20784, -0.39932, 0.19153, &pr4);  // p=0.2, r=5
  // pr_init(1, -1.9966, 0.99686, 0.21568, -0.39932, 0.18369, &pr4);  // p=0.2, r=10
  pr_init(1, -1.9966, 0.99686, 0.22352, -0.39932, 0.17585, &pr4);  // p=0.2, r=15

  // pr origin init
  pr_init(1, -1.9966, 0.99686, 0.0015682, 0, -0.0015682, &pr_origin);

  b1 = 0;
  b2 = 0;
  b3 = 0;
  b4 = 0;

  std_U2 = 0;

  unsigned char s1[16] = {0};
  for (int i = 0; i < 16; i++) {
    s1[i] = ' ';
  }

  // char const* s_stdI = "std I = ";
  // unsigned const char* s_U2_d = "U2 q:";

  MMOODDEE = 0;

  unsigned char s_mode[3] = "  ";
  int map[4] = {0, 2, 3, 4};
  int map2[4] = {7, 9, 10, 11};

  // take conversions indefinitely in loop
  EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // unfreeze, and enter updown count mode
  EPwm1Regs.ETSEL.bit.SOCAEN = 1;                 // enable SOCA
  EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // unfreeze, and enter updown count mode
  EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // unfreeze, and enter updown count mode
  EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // unfreeze, and enter updown count mode
  do {
    static bool blink = 1;
    modeChange();
    s_mode[0] = INVERTER_NO + 0x30;
    s_mode[1] = MMOODDEE + 0x30;
    OLED_ClearGRAM();
    // clearString(s1);
    // placeString(s1, "Io1 rms", 0);
    placeString(s1, s_mode, 14);
    float2numarray(inverter_std_Io, Display_numArray);
    float2numarray(inverter_K, Display_numArray2);
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

    DELAY_US(100000);

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

  GpioDataRegs.GPATOGGLE.bit.GPIO22 = 1;

  // modeChange();

  ADCAResults14[frameIndex] = AdcaResultRegs.ADCRESULT14;
  ADCAResults14_converted[frameIndex] = ADCAResults14[frameIndex] * 3.0 / 4096.0;

  ADCBResults3[frameIndex] = AdcbResultRegs.ADCRESULT1;
  ADCBResults3_converted[frameIndex] = ADCBResults3[frameIndex] * 3.0 / 4096.0;

  ADCAResult14_mean = low_pass_filter(ADCAResults14_converted[frameIndex], &outputPre_A14, alpha_for_avg);

  ADCBResult3_mean = low_pass_filter(ADCBResults3_converted[frameIndex], &outputPre_B3, alpha_for_avg);

  /* 这是周期为50Hz的正弦波表示 */
  wt = wt + PI / 100 / 2 * SW_FREQ;
  if (wt > PI * 2) wt -= PI * 2;

  U2_result[frameIndex] = (ADCAResults14_converted[frameIndex] - Uref_u2) * K_u2;
  ig_result[frameIndex] = -(ADCBResults3_converted[frameIndex] - Uref_i) * K_i;

  // float32 pll2_input = 33.9 * sin(wt);
  // float32 pll2_output = pll_Run(pll2_input, &pll2, &sogi2, &pid_pll2, &ig_q);
  // changeDACAVal(2048 + 2000.0 * sin(wt));
  // changeDACBVal(2048 + 2000.0 * cos(pll2_output));

  if (MMOODDEE == 1) {
    // U2 pll
    float32 pll_input1 = U2_result[frameIndex];
    // float32 pll_input1 = inverter_std_U2 * sin(wt);
    // pll 的结果
    pll_result1 = pll_Run(pll_input1, &pll1, &sogi1, &pid_pll1, &U2_d);
    // 用正弦便于判断正确
    pll_result1 = cos(pll_result1);
    changeDACAVal(2048 + 2000.0 * pll_result1);
    // changeDACAVal(ADCBResults3[frameIndex]);
    // changeDACBVal(ADCAResults14[frameIndex]);

    // ig pll
    float32 pll_input2 = ig_result[frameIndex];
    pll_result2 = pll_Run(pll_input2, &pll2, &sogi2, &pid_pll2, &ig_q);
    pll_result2 = cos(pll_result2);
    changeDACBVal(2048 + 2000.0 * pll_result2);

    float32 err_U2_d = inverter_std_U2 - U2_d;
    float32 pid_n2_input = b2 ? err_U2_d : 0;
    float32 pid_n2_out = pid_nx_Run(pid_n2_input, &pid_n2);
    pid_n2_out = saturation(pid_n2_out, pid_n2_limit, -pid_n2_limit);
    float32 pwm_sig = (pid_n2_out + 0.5) * sin(wt);

    if (ig_q < 0.5) {
      pwm_sig = openLoopRatio * sin(wt);
    }

    changeCMP_value(pwm_sig);
    if (b2) {
      GpioDataRegs.GPASET.bit.GPIO0 = 1;
      GpioDataRegs.GPASET.bit.GPIO2 = 1;
    } else {
      GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;
      GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
    }
  }

  if (MMOODDEE == 2) {
    if (INVERTER_NO == 1) {
      if (b2) {
        if (std_U2 < inverter_std_U2) {
          std_U2 += inverter_std_U2 / rampInterval * 0.00005 * SW_FREQ;
        }

        //
        // (逆变侧)交流电压环
        //
        err1 = sin(wt) * std_U2 - U2_result[frameIndex];
        float32 pr1_input = err1;
        pr1_out = pr_run(pr1_input, &pr1);

        //
        // (逆变侧)交流电流环
        //
        err2 = pr1_out - ig_result[frameIndex];
        float32 pr2_input = err2;
        pr2_out = pr_run(pr2_input, &pr2);
        changeCMP_value(pr2_out);
      }
    }
    if (INVERTER_NO == 2) {
      /* PR控制器启动判断, 启动后变量 b2 自锁 */
      if (b4) {
        b1 = fabsf(U2_result[frameIndex]) >= triggerV;
        b2 = b1 || b3;
        b3 = b2;
      }

      // U2 pll
      float32 pll_input1 = U2_result[frameIndex];
      // float32 pll_input1 = inverter_std_U2 * sin(wt);
      // pll 的结果
      pll_result1 = pll_Run(pll_input1, &pll1, &sogi1, &pid_pll1, &U2_d);
      // 用正弦便于判断正确
      pll_result1 = cos(pll_result1);
      changeDACAVal(2048 + 2000.0 * pll_result1);
      // changeDACBVal(ADCAResults14[frameIndex]);

      //
      // (逆变侧)交流电流环
      //
      err2 = pll_result1 * inverter_std_I_MODE2 - ig_result[frameIndex];
      float32 pr2_input;
      if (b2) {
        pr2_input = err2;
      } else {
        pr2_input = 0;
      }
      pr2_out = pr_run(pr2_input, &pr2);

      changeCMP_value(pr2_out);

      if (b2) {
        GpioDataRegs.GPASET.bit.GPIO0 = 1;
        GpioDataRegs.GPASET.bit.GPIO2 = 1;
      } else {
        GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;
        GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
      }
    }
  }

  if (MMOODDEE == 3) {
    // U2 pll
    float32 pll_input1 = U2_result[frameIndex];
    // float32 pll_input1 = inverter_std_U2 * sin(wt);
    // pll 的结果
    pll_result1 = pll_Run(pll_input1, &pll1, &sogi1, &pid_pll1, &U2_d);
    // 用正弦便于判断正确
    pll_result1 = cos(pll_result1);
    // changeDACBVal(2048 + 2000.0 * pll_result1);
    changeDACAVal(2048 + 2000.0 * pll_result1);
    // changeDACBVal(ADCAResults14[frameIndex]);

    if (b4) {
      if (U2_d >= 30 && time_elapsed < 5) {
        time_elapsed += 0.00005 * SW_FREQ;
      }
      if (time_elapsed >= 5) {
        b2 = true;
      }
    }

    //
    // (逆变侧)交流电流环
    //
    inverter_std_Io1 = inverter_std_Io / (1 + 1 / inverter_K);
    inverter_std_Io2 = inverter_std_Io / (1 + inverter_K);
    inverter_std_Io1 = 0.9207 * inverter_std_Io1 + 0.0331;
    inverter_std_Io2 = 0.9207 * inverter_std_Io2 + 0.0331;
    if (INVERTER_NO == 1) {
      inverter_std_I = inverter_std_Io1 * sqrt(2);
    } else if (INVERTER_NO == 2) {
      inverter_std_I = inverter_std_Io2 * sqrt(2);
    }
    err2 = pll_result1 * inverter_std_I - ig_result[frameIndex];
    float32 pr4_input;
    if (b2) {
      pr4_input = err2;
    } else {
      pr4_input = 0;
    }
    float32 pr4_out = pr_run(pr4_input, &pr4);
    float32 pr5_out = pr_run(pr4_input, &pr_origin);
    float32 pr_out = kpp * pr4_input + krr * pr5_out;
    changeDACBVal(2048 + 2000.0 * err2);

    changeCMP_value(pr_out);

    if (b2) {
      GpioDataRegs.GPASET.bit.GPIO0 = 1;
      GpioDataRegs.GPASET.bit.GPIO2 = 1;
    } else {
      GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;
      GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
    }
  }

  frameIndex++;
  largeIndex++;
  largeIndex %= LARGE_BUFFER;
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

  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

interrupt void xint1_isr(void) {
  Uref_u2 = ADCAResult14_mean;
  Uref_i = ADCBResult3_mean;

  // 差点忘了这个! 没有这个的话, 这个以及其它同组的中断都不会再被触发了
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

interrupt void xint2_isr(void) {
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

  PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

interrupt void xint3_isr(void) {
  if (MMOODDEE == 2) {
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
  }

  if (MMOODDEE == 3) {
    digitPos++;
    digitPos %= 8;
  }

  PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
}
//
// End of file
//

interrupt void xint4_isr(void) {
  if (MMOODDEE == 3) {
    // minus
    if (digitPos < 4) {
      inverter_std_Io -= powf(10, -digitPos);
    } else {
      inverter_K -= powf(10, -(digitPos - 4));
    }
  }

  PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
}
interrupt void xint5_isr(void) {
  if (MMOODDEE == 3) {
    // plus
    if (digitPos < 4) {
      inverter_std_Io += powf(10, -digitPos);
    } else {
      inverter_K += powf(10, -(digitPos - 4));
    }
  }

  PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
}
