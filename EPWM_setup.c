#include "EPWM_setup.h"

#include "F28x_Project.h"

void ConfigureEPWM(void) {
  /* Configure EPWM1 */
  EALLOW;
  //
  // Setup TB
  //
  EPwm1Regs.TBPRD = PWM_MAX_COUNT;                // Set timer period
  EPwm1Regs.TBPHS.bit.TBPHS = 0x0000;             // Phase is 0
  EPwm1Regs.TBCTR = 0x0000;                       // Clear counter
  EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // Count updown
  EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;         // Disable phase loading
  EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;
  EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
  ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 0;  // EPWMCLK = PLLSYSCLK
  EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;     // Clock ratio to SYSCLKOUT
  EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

  //
  // Setup CC
  //
  EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
  EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
  EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
  EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
  EPwm1Regs.CMPA.bit.CMPA = PWM_MID_COUNT;  // Set compare A value to max/2 counts
  EPwm1Regs.CMPB.bit.CMPB = 0;

  //
  // Setup AQ
  //
  EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;
  EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
  EPwm1Regs.AQCTLB.bit.CBU = AQ_NO_ACTION;  // B No Action
  EPwm1Regs.AQCTLB.bit.CBD = AQ_NO_ACTION;

  //
  // Setup DB
  //
  EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;          // 只接受AQ出来的EpwmxA
  EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       // active high complementary
  EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // enable s1, s0
  EPwm1Regs.DBRED.bit.DBRED = 0;
  EPwm1Regs.DBFED.bit.DBFED = 0;

  //
  // Setup ET. Assumes ePWM clock is already enabled
  //
  EPwm1Regs.ETSEL.bit.SOCAEN = 0;   // Disable SOC on A group
  EPwm1Regs.ETSEL.bit.SOCASEL = 2;  // Select SOC on counter equal to period
  EPwm1Regs.ETPS.bit.SOCAPRD = 1;   // Generate pulse on 1st event

  EPwm1Regs.TBCTL.bit.CTRMODE = 3;  // freeze counter
  EDIS;

  /* Configure EPWM2 */
  EALLOW;
  //
  // Setup TB
  //
  EPwm2Regs.TBPRD = PWM_MAX_COUNT;                // Set timer period
  EPwm2Regs.TBPHS.bit.TBPHS = 0x0000;             // Phase is 0
  EPwm2Regs.TBCTR = 0x0000;                       // Clear counter
  EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  // Count updown
  EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;          // Disable phase loading
  EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;
  EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
  ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 0;  // EPWMCLK = PLLSYSCLK
  EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;     // Clock ratio to SYSCLKOUT
  EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

  //
  // Setup CC
  //
  EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
  EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
  EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
  EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
  EPwm2Regs.CMPA.bit.CMPA = PWM_MID_COUNT;  // Set compare A value to max/2 counts
  EPwm2Regs.CMPB.bit.CMPB = 0;

  //
  // Setup AQ
  //
  EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;
  EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;
  EPwm2Regs.AQCTLB.bit.CBU = AQ_NO_ACTION;  // B No Action
  EPwm2Regs.AQCTLB.bit.CBD = AQ_NO_ACTION;

  //
  // Setup DB
  //
  EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;          // 只接受AQ出来的EpwmxA
  EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       // active high complementary
  EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // enable s1, s0
  EPwm2Regs.DBRED.bit.DBRED = 0;
  EPwm2Regs.DBFED.bit.DBFED = 0;

  // freeze counter
  EPwm2Regs.TBCTL.bit.CTRMODE = 3;  // freeze counter
  EDIS;
}

void changeCMP_phase(float32 wt) {
  Uint16 duty1 = (Uint16)(PWM_MID_COUNT + PWM_MID_COUNT * sin(wt));
  // Uint16 duty2 = (Uint16)(PWM_MID_COUNT - PWM_MID_COUNT * sin(wt));
  Uint16 duty2 = 0;
  EPwm1Regs.CMPA.bit.CMPA = duty1;
  EPwm2Regs.CMPA.bit.CMPA = duty2;
}

/// @brief 改变PWM占空比
/// @param val 相当于sin(wt)
void changeCMP_value(float32 val) {
  Uint16 duty1 = (PWM_MID_COUNT + PWM_MID_COUNT * val);
  // Uint16 duty2 = (PWM_MID_COUNT - PWM_MID_COUNT * val);
  Uint16 duty2 = 0;
  EPwm1Regs.CMPA.bit.CMPA = duty1;
  EPwm2Regs.CMPA.bit.CMPA = duty2;
}

void disableEpwm1Gpio(void) {
  EALLOW;
  GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;
  GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;
  EDIS;
}

void disableEpwm2Gpio(void) {
  EALLOW;
  GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;
  GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;
  EDIS;
}

void enableEpwm1Gpio(void) {
  EALLOW;
  GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
  GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
  EDIS;
}

void enableEpwm2Gpio(void) {
  EALLOW;
  GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
  GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;
  EDIS;
}
