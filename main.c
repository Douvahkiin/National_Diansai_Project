//
// Included Files
//
#include "ADC_setup.h"
#include "EPWM_setup.h"
#include "F28x_Project.h"
#include "math.h"

//
// Function Prototypes
//
interrupt void adca1_isr(void);

//
// Defines
//
#define RESULTS_BUFFER_SIZE 500

//
// Globals
//
// Uint16 ADCAResults0[RESULTS_BUFFER_SIZE];
// float32 ADCAResults0_converted[RESULTS_BUFFER_SIZE];
Uint16 ADCAResults1[RESULTS_BUFFER_SIZE];
float32 ADCAResults1_converted[RESULTS_BUFFER_SIZE];

// Uint16 ADCBResults0[RESULTS_BUFFER_SIZE];
// float32 ADCBResults0_converted[RESULTS_BUFFER_SIZE];
// Uint16 ADCBResults1[RESULTS_BUFFER_SIZE];
// float32 ADCBResults1_converted[RESULTS_BUFFER_SIZE];

float32 wt = 0;

Uint16 frameIndex;

volatile Uint16 bufferFull;

void main(void) {
  //
  // Step 1. Initialize System Control:
  // PLL, WatchDog, enable Peripheral Clocks
  // This example function is found in the F2837xD_SysCtrl.c file.
  //
  InitSysCtrl();

  //
  // Step 2. Initialize GPIO:
  // This example function is found in the F2837xD_Gpio.c file and
  // illustrates how to set the GPIO to it's default state.
  //
  InitGpio();
  InitEPwm1Gpio();
  InitEPwm2Gpio();

  //
  // Step 3. Clear all interrupts and initialize PIE vector table:
  // Disable CPU interrupts
  //
  DINT;

  //
  // Initialize the PIE control registers to their default state.
  // The default state is all PIE interrupts disabled and flags
  // are cleared.
  // This function is found in the F2837xD_PieCtrl.c file.
  //
  InitPieCtrl();

  //
  // Disable CPU interrupts and clear all CPU interrupt flags:
  //
  IER = 0x0000;
  IFR = 0x0000;

  //
  // Initialize the PIE vector table with pointers to the shell Interrupt
  // Service Routines (ISR).
  // This will populate the entire table, even if the interrupt
  // is not used in this example.  This is useful for debug purposes.
  // The shell ISR routines are found in F2837xD_DefaultIsr.c.
  // This function is found in F2837xD_PieVect.c.
  //
  InitPieVectTable();

  //
  // Map ISR functions
  //
  EALLOW;
  PieVectTable.ADCA1_INT = &adca1_isr;  // function for ADCA interrupt 1
  EDIS;

  //
  // Configure the ADC and power it up
  //
  ConfigureADC();

  //
  // Configure the ePWM
  //
  ConfigureEPWM();

  //
  // Setup the ADC for ePWM triggered conversions
  //
  SetupADCEpwm();

  //
  // Enable global Interrupts and higher priority real-time debug events:
  //
  IER |= M_INT1;  // Enable group 1 interrupts
  EINT;           // Enable Global interrupt INTM
  ERTM;           // Enable Global realtime interrupt DBGM

  //
  // Initialize results buffer
  //
  // for (frameIndex = 0; frameIndex < RESULTS_BUFFER_SIZE; frameIndex++) {
  //   ADCAResults0[frameIndex] = 0;
  // }
  frameIndex = 0;
  bufferFull = 0;

  //
  // enable PIE interrupt
  //
  PieCtrlRegs.PIEIER1.bit.INTx1 = 1;

  //
  // sync ePWM
  //
  EALLOW;
  CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;

  //
  // Initialize adca results converted
  //
  //  for (int i = 0; i < RESULTS_BUFFER_SIZE; i++) {
  //    ADCAResults0_converted[i] = 0;
  //  }

  //
  // take conversions indefinitely in loop
  //
  EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // unfreeze, and enter updown count mode
  EPwm1Regs.ETSEL.bit.SOCAEN = 1;                 // enable SOCA
  EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // unfreeze, and enter updown count mode
  do {
    //
    // start ePWM
    //

    //
    // wait while ePWM causes ADC conversions, which then cause interrupts,
    // which fill the results buffer, eventually setting the bufferFull flag
    //
    while (!bufferFull) {
    }
    bufferFull = 0;  // clear the buffer full flag

    //
    // stop ePWM
    //
    // EPwm1Regs.ETSEL.bit.SOCAEN = 0;  // disable SOCA

    //
    // at this point, ADCAResults0[] contains a sequence of conversions
    // from the selected channel
    //

    //
    // software breakpoint, hit run again to get updated conversions
    //
    // asm("   ESTOP0");
  } while (1);
}

//
// adca1_isr - Read ADC Buffer in ISR
//
interrupt void adca1_isr(void) {
  while (AdcaRegs.ADCINTFLG.bit.ADCINT1 == 0 || AdcbRegs.ADCINTFLG.bit.ADCINT1 == 0) {
  }
  AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
  AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

  // ADCAResults0[frameIndex] = AdcaResultRegs.ADCRESULT0;
  // ADCAResults0_converted[frameIndex] = ADCAResults0[frameIndex] * 3.0 / 4096.0;
  ADCAResults1[frameIndex] = AdcaResultRegs.ADCRESULT1;
  ADCAResults1_converted[frameIndex] = ADCAResults1[frameIndex] * 3.0 / 4096.0;

  // ADCBResults0[frameIndex] = AdcbResultRegs.ADCRESULT0;
  // ADCBResults0_converted[frameIndex] = ADCBResults0[frameIndex] * 3.0 / 4096.0;
  // ADCBResults1[frameIndex] = AdcbResultRegs.ADCRESULT1;
  // ADCBResults1_converted[frameIndex] = ADCBResults1[frameIndex] * 3.0 / 4096.0;

  /* 这是周期为50Hz的正弦波表示 */
  wt = wt + 0.0314159269;
  if (wt > 3.14159269 * 2) wt -= 3.14159269 * 2;

  changeDuty_phase(0);

  frameIndex++;
  if (RESULTS_BUFFER_SIZE <= frameIndex) {
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
