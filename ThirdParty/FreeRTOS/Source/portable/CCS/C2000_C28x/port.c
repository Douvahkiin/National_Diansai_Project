/*
 * FreeRTOS Kernel <DEVELOPMENT BRANCH>
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// Scheduler includes.
//-------------------------------------------------------------------------------------------------
#include "FreeRTOS.h"
#include "task.h"

//-------------------------------------------------------------------------------------------------
// Device register access (this project uses register-level code, no driverlib).
//-------------------------------------------------------------------------------------------------
#include "F28x_Project.h"

//-------------------------------------------------------------------------------------------------
// Implementation of functions defined in portable.h for the C28x port.
//-------------------------------------------------------------------------------------------------

// Constants required for hardware setup.
#define portINITIAL_CRITICAL_NESTING  ( ( uint16_t ) 10 )
#define portFLAGS_INT_ENABLED         ( ( StackType_t ) 0x08 )
#if defined(__TMS320C28XX_FPU64__)
# define AUX_REGISTERS_TO_SAVE        27 // XAR + FPU registers
# define XAR4_REGISTER_POSITION       6  // XAR4 position in AUX registers array
# define STF_REGISTER_POSITION        10 // STF position in AUX registers array
#elif defined(__TMS320C28XX_FPU32__)
# define AUX_REGISTERS_TO_SAVE        19 // XAR + FPU registers
# define XAR4_REGISTER_POSITION       6  // XAR4 position in AUX registers array
# define STF_REGISTER_POSITION        10 // STF position in AUX registers array
#else
# define AUX_REGISTERS_TO_SAVE        9  // XAR registers only
# define XAR4_REGISTER_POSITION       5  // XAR4 position in AUX registers array
#endif

extern uint32_t getSTF( void );

/*
 * Setup the timer to generate the tick interrupts.  The implementation in this
 * file is weak to allow application writers to change the timer used to
 * generate the tick interrupt.
 */
void vPortSetupTimerInterrupt( void );


// Each task maintains a count of the critical section nesting depth.  Each
// time a critical section is entered the count is incremented.  Each time a
// critical section is exited the count is decremented - with interrupts only
// being re-enabled if the count is zero.
//
// ulCriticalNesting will get set to zero when the scheduler starts, but must
// not be initialised to zero as this will cause problems during the startup
// sequence.
// ulCriticalNesting should be 32 bit value to keep stack alignment unchanged.
volatile uint32_t ulCriticalNesting = portINITIAL_CRITICAL_NESTING;
volatile uint16_t bYield = 0;
volatile uint16_t bPreemptive = 0;

//-------------------------------------------------------------------------------------------------
// Initialise the stack of a task to look exactly as if
// timer interrupt was executed.
//-------------------------------------------------------------------------------------------------
StackType_t *pxPortInitialiseStack( StackType_t *pxTopOfStack, TaskFunction_t pxCode, void *pvParameters )
{
  uint16_t i;
  uint16_t base = 0;

  pxTopOfStack[base++]  = 0x0080;  // ST0. PSM = 0(No shift)
  pxTopOfStack[base++]  = 0x0000;  // T
  pxTopOfStack[base++]  = 0x0000;  // AL
  pxTopOfStack[base++]  = 0x0000;  // AH
  pxTopOfStack[base++]  = 0xFFFF;  // PL
  pxTopOfStack[base++]  = 0xFFFF;  // PH
  pxTopOfStack[base++]  = 0xFFFF;  // AR0
  pxTopOfStack[base++]  = 0xFFFF;  // AR1
  pxTopOfStack[base++]  = 0x8A08;  // ST1
  pxTopOfStack[base++]  = 0x0000;  // DP
  pxTopOfStack[base++]  = 0x0000;  // IER
  pxTopOfStack[base++]  = 0x0000;  // DBGSTAT
  pxTopOfStack[base++]  = ((uint32_t)pxCode) & 0xFFFFU;       // PCL
  pxTopOfStack[base++]  = ((uint32_t)pxCode >> 16) & 0x00FFU; // PCH
  pxTopOfStack[base++]  = 0xAAAA;  // Alignment
  pxTopOfStack[base++]  = 0xBBBB;  // Alignment

  // Fill the rest of the registers with dummy values.
  for(i = 0; i < (2 * AUX_REGISTERS_TO_SAVE); i++)
  {
    uint16_t low  = 0x0000;
    uint16_t high = 0x0000;

    if(i == (2 * XAR4_REGISTER_POSITION))
    {
      low  = ((uint32_t)pvParameters) & 0xFFFFU;
      high = ((uint32_t)pvParameters >> 16) & 0xFFFFU;
    }

#if defined(__TMS320C28XX_FPU32__)
    if(i == (2 * STF_REGISTER_POSITION))
    {
      uint32_t stf = getSTF();

      low  = stf & 0xFFFFU;
      high = (stf >> 16) & 0xFFFFU;
    }
#endif

    pxTopOfStack[base + i] = low;
    i++;
    pxTopOfStack[base + i] = high;
  }

  base += i;

  // Reserve place for ST1 which will be used in context switch
  // to set correct SPA bit ASAP.
  pxTopOfStack[base++] = 0x8A18;  // ST1 with SPA bit set
  pxTopOfStack[base++] = 0x0000;  // DP
  pxTopOfStack[base++] = 0x0000;  // placeholder for 32 bit ulCriticalNesting
  pxTopOfStack[base++] = 0x0000;

  // Return a pointer to the top of the stack we have generated so this can
  // be stored in the task control block for the task.
  return pxTopOfStack + base;
}

//-------------------------------------------------------------------------------------------------
void vPortEndScheduler( void )
{
  // It is unlikely that the TMS320 port will get stopped.
  // If required simply disable the tick interrupt here.
}

//-------------------------------------------------------------------------------------------------
void vPortSetupSWInterrupt( void )
{
  EALLOW;
  PieVectTable.WAKE_INT = &portTICK_ISR;   // PIE 1.8 - RTOS yield soft interrupt
  EDIS;
  PieCtrlRegs.PIEIER1.bit.INTx8 = 1;
  IER |= M_INT1;
}

//-------------------------------------------------------------------------------------------------
// See header file for description.
//-------------------------------------------------------------------------------------------------
BaseType_t xPortStartScheduler(void)
{
  vPortSetupSWInterrupt();
  vPortSetupTimerInterrupt();

  ulCriticalNesting = 0;

#if(configUSE_PREEMPTION == 1)
  bPreemptive = 1;
#else
  bPreemptive = 0;
#endif

  portENABLE_INTERRUPTS();
  portRESTORE_FIRST_CONTEXT();

  // Should not get here!
  return pdFAIL;
}

//-------------------------------------------------------------------------------------------------
void vPortEnterCritical( void )
{
  portDISABLE_INTERRUPTS();
  ulCriticalNesting++;
}

//-------------------------------------------------------------------------------------------------
void vPortExitCritical( void )
{
  ulCriticalNesting--;
  if( ulCriticalNesting == 0 )
  {
    portENABLE_INTERRUPTS();
  }
}

/*
 * Setup CPUTIMER2 to generate the tick interrupts at the required
 * frequency.  Raw register access (no driverlib in this project).
 * NOTE: the original source marked this function #pragma WEAK - newer
 * C2000 compilers do not accept that pragma, and this project never
 * re-defines it, so the attribute is dropped.
 */
void vPortSetupTimerInterrupt( void )
{
    //
    // CPU Timer 2 clock enable (PCLKCR0).
    //
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.CPUTIMER2 = 1;
    EDIS;

    //
    // Initialize timer period:
    //
    CpuTimer2Regs.TCR.bit.TSS = 1;   // stop timer
    CpuTimer2Regs.TCR.bit.TIE = 0;   // disable interrupt while programming
    CpuTimer2Regs.PRD.all = configCPU_CLOCK_HZ / configTICK_RATE_HZ;

    //
    // Set pre-scale counter to divide by 1 (SYSCLKOUT):
    //
    CpuTimer2Regs.TPR.all = 0;
    CpuTimer2Regs.TPRH.all = 0;

    CpuTimer2Regs.TCR.bit.TRB = 1;   // reload timer with value in PRD
    CpuTimer2Regs.TCR.bit.TIF = 1;   // clear interrupt flag

    //
    // Register the tick ISR (PIE group 14, vector 2 = CPU Timer 2).
    // Groups 13-16 are not PIE-multiplexed: no PIEIER / PIEACK step needed,
    // enabling the CPU interrupt (IER |= M_INT14) is sufficient.
    //
    EALLOW;
    PieVectTable.TIMER2_INT = &portTICK_ISR;
    EDIS;
    IER |= M_INT14;

    //
    // Enable interrupt and start timer.
    //
    CpuTimer2Regs.TCR.bit.TIE = 1;
    CpuTimer2Regs.TCR.bit.TSS = 0;   // start timer
}
