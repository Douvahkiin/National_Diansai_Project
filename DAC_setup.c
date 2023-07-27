#include "DAC_setup.h"

void ConfigureDAC(void) {
  EALLOW;
  DacaRegs.DACCTL.bit.DACREFSEL = 1;   // select ref voltage
  DacaRegs.DACOUTEN.bit.DACOUTEN = 1;  // enable output
  DELAY_US(500);                       // power up time
  DacaRegs.DACVALS.bit.DACVALS = 0;    // clear dacval
  EDIS;
}

void changeDACVal(Uint16 val) {
  DacaRegs.DACVALS.bit.DACVALS = val;
}
