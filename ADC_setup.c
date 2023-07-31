#include "ADC_setup.h"

//
// ConfigureADC - Write ADC configurations and power up the ADC for both
//                ADC A and ADC B
//
void ConfigureADC(void) {
  /* Configure ADCA */
  EALLOW;

  //
  // write configurations
  //
  AdcaRegs.ADCCTL2.bit.PRESCALE = 6;  // set ADCCLK divider to /4
  AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

  //
  // Set pulse positions to late
  //
  AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;

  //
  // power up the ADC
  //
  AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;

  //
  // delay for 1ms to allow ADC time to power up
  //
  DELAY_US(1000);

  EDIS;

  /* Configure ADCB */
  EALLOW;

  //
  // write configurations
  //
  AdcbRegs.ADCCTL2.bit.PRESCALE = 6;  // set ADCCLK divider to /4
  AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

  //
  // Set pulse positions to late
  //
  AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;

  //
  // power up the ADC
  //
  AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;

  //
  // delay for 1ms to allow ADC time to power up
  //
  DELAY_US(1000);

  EDIS;

  /* Configure ADCC */
  EALLOW;

  //
  // write configurations
  //
  AdccRegs.ADCCTL2.bit.PRESCALE = 6;  // set ADCCLK divider to /4
  AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

  //
  // Set pulse positions to late
  //
  AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;

  //
  // power up the ADC
  //
  AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;

  //
  // delay for 1ms to allow ADC time to power up
  //
  DELAY_US(1000);

  EDIS;
}

//
// SetupADCEpwm - Setup ADC EPWM acquisition window
//
void SetupADCEpwm(void) {
  Uint16 acqps;

  //
  // Determine minimum acquisition window (in SYSCLKS) based on resolution
  //
  if (ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION) {
    acqps = 14;  // 75ns
  } else         // resolution is 16-bit
  {
    acqps = 63;  // 320ns
  }

  //
  // Select the channels to convert and end of conversion flag
  //
  EALLOW;
  AdcaRegs.ADCSOC0CTL.bit.CHSEL = ADC_CHANNEL_2;  // SOC0 will convert pin A2
  AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;          // sample window is 100 SYSCLK cycles
  AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;            // trigger on ePWM1 SOCA/C
  AdcbRegs.ADCSOC0CTL.bit.CHSEL = ADC_CHANNEL_2;
  AdcbRegs.ADCSOC0CTL.bit.ACQPS = acqps;
  AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5;
  AdccRegs.ADCSOC0CTL.bit.CHSEL = ADC_CHANNEL_3;
  AdccRegs.ADCSOC0CTL.bit.ACQPS = acqps;
  AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 5;

  AdcaRegs.ADCSOC1CTL.bit.CHSEL = ADC_CHANNEL_3;
  AdcaRegs.ADCSOC1CTL.bit.ACQPS = acqps;
  AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 5;
  AdcbRegs.ADCSOC1CTL.bit.CHSEL = ADC_CHANNEL_3;
  AdcbRegs.ADCSOC1CTL.bit.ACQPS = acqps;
  AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 5;
  AdccRegs.ADCSOC1CTL.bit.CHSEL = ADC_CHANNEL_2;
  AdccRegs.ADCSOC1CTL.bit.ACQPS = acqps;
  AdccRegs.ADCSOC1CTL.bit.TRIGSEL = 5;

  AdcaRegs.ADCSOC14CTL.bit.CHSEL = ADC_CHANNEL_14;  // SOC0 will convert pin A2
  AdcaRegs.ADCSOC14CTL.bit.ACQPS = acqps;           // sample window is 100 SYSCLK cycles
  AdcaRegs.ADCSOC14CTL.bit.TRIGSEL = 5;             // trigger on ePWM1 SOCA/C

  AdcaRegs.ADCSOC15CTL.bit.CHSEL = ADC_CHANNEL_15;  // SOC0 will convert pin A2
  AdcaRegs.ADCSOC15CTL.bit.ACQPS = acqps;           // sample window is 100 SYSCLK cycles
  AdcaRegs.ADCSOC15CTL.bit.TRIGSEL = 5;             // trigger on ePWM1 SOCA/C

  AdcaRegs.ADCSOC2CTL.bit.CHSEL = ADC_CHANNEL_0;
  AdcaRegs.ADCSOC2CTL.bit.ACQPS = acqps;
  AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 5;

  AdcaRegs.ADCSOC3CTL.bit.CHSEL = ADC_CHANNEL_2;
  AdcaRegs.ADCSOC3CTL.bit.ACQPS = acqps;
  AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 5;

  AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1;  // end of SOC1 will set ADCA's INT1 flag
  AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;    // enable INT1 flag
  AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;  // make sure INT1 flag is cleared

  AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 1;  // end of SOC1 will set ADCB's INT1 flag
  AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;    // enable INT1 flag
  AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

  AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 0;  // end of SOC0 will set ADCC's INT1 flag
  AdccRegs.ADCINTSEL1N2.bit.INT1E = 1;    // enable INT1 flag
  AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

  EDIS;
}
