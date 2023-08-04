#include "keys.h"
void configure_keys(void) {
  // configure XINT1 and XINT2
  GPIO_SetupXINT1Gpio(61);
  GPIO_SetupXINT2Gpio(123);
  GPIO_SetupXINT3Gpio(122);
  XintRegs.XINT1CR.bit.POLARITY = 1;  // Rising edge interrupt
  XintRegs.XINT2CR.bit.POLARITY = 1;  // Rising edge interrupt
  XintRegs.XINT3CR.bit.POLARITY = 1;  // Rising edge interrupt
  XintRegs.XINT1CR.bit.ENABLE = 1;    // Enable XINT1
  XintRegs.XINT2CR.bit.ENABLE = 1;    // Enable XINT2
  XintRegs.XINT3CR.bit.ENABLE = 1;    // Enable XINT2

  EALLOW;
  GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0;   // GPIO
  GpioCtrlRegs.GPBDIR.bit.GPIO61 = 0;    // input
  GpioCtrlRegs.GPBQSEL2.bit.GPIO61 = 0;  // XINT1 Synch to SYSCLKOUT only

  GpioCtrlRegs.GPDMUX2.bit.GPIO123 = 0;   // GPIO
  GpioCtrlRegs.GPDDIR.bit.GPIO123 = 0;    // input
  GpioCtrlRegs.GPDQSEL2.bit.GPIO123 = 0;  // XINT2 Synch to SYSCLKOUT only

  GpioCtrlRegs.GPDMUX2.bit.GPIO122 = 0;   // GPIO
  GpioCtrlRegs.GPDDIR.bit.GPIO122 = 0;    // input
  GpioCtrlRegs.GPDQSEL2.bit.GPIO122 = 0;  // XINT2 Synch to SYSCLKOUT only
  // GpioCtrlRegs.GPDQSEL2.bit.GPIO123 = 2;     // XINT2 Qual using 6 samples
  // GpioCtrlRegs.GPDCTRL.bit.QUALPRD0 = 0xFF;  // Each sampling window is 510*SYSCLKOUT
  EDIS;
}
