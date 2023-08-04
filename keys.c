#include "keys.h"
void configure_keys(void) {
  // configure XINT1 and XINT2
  GPIO_SetupXINT1Gpio(61);
  GPIO_SetupXINT2Gpio(123);
  GPIO_SetupXINT3Gpio(122);
  GPIO_SetupXINT4Gpio(58);
  GPIO_SetupXINT5Gpio(59);
  XintRegs.XINT1CR.bit.POLARITY = 1;  // Rising edge interrupt
  XintRegs.XINT2CR.bit.POLARITY = 1;  // Rising edge interrupt
  XintRegs.XINT3CR.bit.POLARITY = 1;  // Rising edge interrupt
  XintRegs.XINT4CR.bit.POLARITY = 1;  // Rising edge interrupt
  XintRegs.XINT5CR.bit.POLARITY = 1;  // Rising edge interrupt
  XintRegs.XINT1CR.bit.ENABLE = 1;    // Enable XINT1
  XintRegs.XINT2CR.bit.ENABLE = 1;    // Enable XINT2
  XintRegs.XINT3CR.bit.ENABLE = 1;    // Enable XINT3
  XintRegs.XINT4CR.bit.ENABLE = 1;    // Enable XINT4
  XintRegs.XINT5CR.bit.ENABLE = 1;    // Enable XINT5

  EALLOW;
  GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0;   // GPIO
  GpioCtrlRegs.GPBDIR.bit.GPIO61 = 0;    // input
  GpioCtrlRegs.GPBQSEL2.bit.GPIO61 = 2;  // XINT1 Qual using 6 samples

  GpioCtrlRegs.GPDMUX2.bit.GPIO123 = 0;   // GPIO
  GpioCtrlRegs.GPDDIR.bit.GPIO123 = 0;    // input
  GpioCtrlRegs.GPDQSEL2.bit.GPIO123 = 2;  // XINT2 Qual using 6 samples

  GpioCtrlRegs.GPDMUX2.bit.GPIO122 = 0;   // GPIO
  GpioCtrlRegs.GPDDIR.bit.GPIO122 = 0;    // input
  GpioCtrlRegs.GPDQSEL2.bit.GPIO122 = 2;  // XINT3 Qual using 6 samples

  GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 0;   // GPIO
  GpioCtrlRegs.GPBDIR.bit.GPIO58 = 0;    // input
  GpioCtrlRegs.GPBQSEL2.bit.GPIO58 = 2;  // XINT4 Qual using 6 samples

  GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 0;   // GPIO
  GpioCtrlRegs.GPBDIR.bit.GPIO59 = 0;    // input
  GpioCtrlRegs.GPBQSEL2.bit.GPIO59 = 2;  // XINT5 Qual using 6 samples

  GpioCtrlRegs.GPDCTRL.bit.QUALPRD3 = 0xFF;  // GPIO122~GPIO123: Each sampling window is 510*SYSCLKOUT
  GpioCtrlRegs.GPBCTRL.bit.QUALPRD3 = 0xFF;  // GPIO122~GPIO123: Each sampling window is 510*SYSCLKOUT
  EDIS;
}
