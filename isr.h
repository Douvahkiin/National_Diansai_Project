#ifndef _ISR_
#define _ISR_

#include "F28x_Project.h"

//
// Interrupt service routines (CPU1).
//
// adca1_isr: 20 kHz ADC interrupt triggered at the ePWM carrier peak.
//   - waits for the three ADC modules' conversion flags,
//   - saves the raw sample words + a CPU timer snapshot,
//   - notifies the ControlTask (single-period token),
//   - raises the GPIO24 end-to-end latency probe.
//
// xint1..5_isr: key interrupts, reduced to event-group bits for KeyTask.
//
interrupt void adca1_isr(void);
interrupt void xint1_isr(void);
interrupt void xint2_isr(void);
interrupt void xint3_isr(void);
interrupt void xint4_isr(void);
interrupt void xint5_isr(void);

#endif  // _ISR_
