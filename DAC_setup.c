#include "DAC_setup.h"

void ConfigureDAC(void) {
  EALLOW;
  // // DACA
  // DacaRegs.DACCTL.bit.DACREFSEL = 1;   // select ref voltage
  // DacaRegs.DACOUTEN.bit.DACOUTEN = 1;  // enable output
  // DELAY_US(500);                       // power up time
  // DacaRegs.DACVALS.bit.DACVALS = 0;    // clear dacval

  // DACB
  DacbRegs.DACCTL.bit.DACREFSEL = 1;   // select ref voltage
  DacbRegs.DACOUTEN.bit.DACOUTEN = 1;  // enable output
  DELAY_US(500);                       // power up time
  DacbRegs.DACVALS.bit.DACVALS = 0;    // clear dacval
  EDIS;
}

void changeDACAVal(Uint16 val) {
  DacaRegs.DACVALS.bit.DACVALS = val;
}

void changeDACBVal(Uint16 val) {
  DacbRegs.DACVALS.bit.DACVALS = val;
}
