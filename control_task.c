#include "control_task.h"
#include "app_tasks.h"
#include "DAC_setup.h"
#include "EPWM_setup.h"
#include "MACRO.h"
#include "filters.h"
#include "math.h"
#include "pid.h"
#include "pll.h"
#include "pr.h"
#include "utils.h"

//
// extern algorithm instances (defined in pll.c / pid.c / pr.c)
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
// Shared control state
//
bool b1 = 0;
bool b2 = 0;
bool b3 = 0;
bool b4 = 0;

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
float32 inverter_std_I_rms = 2;
float32 inverter_std_U2 = 33.941125;
float32 triggerV = 16.9705627;  // 12*sqrt(2)

float32 pid_n1_limit = 1;
float32 pid_n2_limit = 0.5;

float32 U2_d = 0;
float32 ig_q = 0;

float32 std_U2 = 0;
float32 time_elapsed = 0;
float32 openLoopRatio = 0.711;

float32 kpp = 0.5;
float32 krr = 60;

float32 ADCAResult14_mean = 0;
float32 ADCBResult3_mean = 0;

float32 ADCAResults14_converted[BUFFER_SIZE];
float32 ADCBResults3_converted[BUFFER_SIZE];

volatile Uint16 control_overrun_cnt = 0;

//
// Private
//
static float32 wt = 0;
static float32 inverter_std_I_MODE2 = 2.828427;

//
// ControlTask's own stack/TCB (static allocation - see .freertosStaticStack)
//
#define CONTROL_STACK_WORDS 512
static StackType_t uxControlTaskStack[CONTROL_STACK_WORDS];
#pragma DATA_SECTION(uxControlTaskStack, ".freertosStaticStack")
#pragma DATA_ALIGN(uxControlTaskStack, 2)
static StaticTask_t xControlTaskTCB;

static void control_trip(void);

//
// control_task_init - FreeRTOS-independent setup, identical to the original
// boot-time initialization.  Called from main() before the scheduler starts.
//
void control_task_init(void) {
  //
  // pll, pid init
  //
  pll_Init(2 * PI * 50, 2, &pll1, &sogi1, &pid_pll1);  // 50Hz
  pll_Init(2 * PI * 50, 2, &pll2, &sogi2, &pid_pll2);  // 50Hz
  pid_nx_Init(0.15, 0, 0, pid_n1_limit, -pid_n1_limit, &pid_n1);
  pid_nx_Init(0.01, 7, 0, pid_n2_limit / 7, -pid_n2_limit / 7, &pid_n2);

  //
  // pr1 init (Ts = 0.00005, p=0.05, r=5)
  //
  pr_init(1, -1.9966, 0.99686, 0.065682, -0.099831, 0.034161, &pr1);

  //
  // pr2 init (p=0.1, r=5)
  //
  pr_init(1, -1.9966, 0.99686, 0.10784, -0.19966, 0.091845, &pr2);

  //
  // pr3 init (p=0.1, r=5)
  //
  pr_init(1, -1.9966, 0.99686, 0.10784, -0.19966, 0.091845, &pr3);

  //
  // pr4 init (p=0.2, r=15)
  //
  pr_init(1, -1.9966, 0.99686, 0.22352, -0.39932, 0.17585, &pr4);

  //
  // pr origin init
  //
  pr_init(1, -1.9966, 0.99686, 0.0015682, 0, -0.0015682, &pr_origin);

  b1 = 0;
  b2 = 0;
  b3 = 0;
  b4 = 0;
  std_U2 = 0;
  time_elapsed = 0;
  control_overrun_cnt = 0;

  //
  // CpuTimer0 = free running 10 ns counter (for control period monitoring).
  //
  CpuTimer0Regs.TCR.bit.TSS = 1;
  CpuTimer0Regs.TCR.bit.TIE = 0;
  CpuTimer0Regs.PRD.all = 0xFFFFFFFF;
  CpuTimer0Regs.TPR.all = 0;
  CpuTimer0Regs.TPRH.all = 0;
  CpuTimer0Regs.TCR.bit.TRB = 1;
  CpuTimer0Regs.TCR.bit.TIF = 1;
  CpuTimer0Regs.TCR.bit.TSS = 0;  // free-run
}

void control_create_task(void) {
  xControlTaskHandle = xTaskCreateStatic(control_task, "ctrl", CONTROL_STACK_WORDS, NULL, TASK_PRIO_CONTROL, uxControlTaskStack, &xControlTaskTCB);
}

void control_task(void *pvParameters) {
  Uint16 idx = 0;
  Uint16 a14raw = 0;
  Uint16 b1raw = 0;
  Uint32 elapsed;

  (void)pvParameters;

  for (;;) {
    //
    // Wait for the ADC ISR sampling trigger (fastest wake-up primitive).
    // No timeout: strictly one sample per notification = single-period token.
    //
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    //
    // Read the frame data produced by the ADC ISR (single writer, ISR side).
    //
    a14raw = adc_isr_a14;
    b1raw = adc_isr_b1;
    idx = adc_isr_frame_index;

    control_step(a14raw, b1raw, idx);

    //
    // Period miss detection: 45 us @ 100 MHz = 4,500,000 cycles.
    // adc_isr_tick0 = CpuTimer0 count captured at ADC ISR entry.
    // CpuTimer0 counts down, so elapsed = start - now (mod 2^32).
    //
    elapsed = (Uint32)(adc_isr_tick0 - CpuTimer0Regs.TIM.all);
    if (elapsed > 4500000UL) {
      control_trip();
    }
  }
}

//
// control_step - every control calculation of the original adca1_isr.
//
void control_step(Uint16 adca14, Uint16 adcb1, Uint16 frameIndex) {
  float32 a14v = adca14 * 3.0 / 4096.0;
  float32 b1v = adcb1 * 3.0 / 4096.0;

  ADCAResults14_converted[frameIndex] = a14v;
  ADCBResults3_converted[frameIndex] = b1v;

  ADCAResult14_mean = low_pass_filter(a14v, &outputPre_A14, alpha_for_avg);
  ADCBResult3_mean = low_pass_filter(b1v, &outputPre_B3, alpha_for_avg);

  /* 这是周期为50Hz的正弦波表示 */
  wt = wt + PI / 100 / 2 * SW_FREQ;
  if (wt > PI * 2) wt -= PI * 2;

  U2_result[frameIndex] = (a14v - Uref_u2) * K_u2;
  ig_result[frameIndex] = -(b1v - Uref_i) * K_i;

  if (MMOODDEE == 1) {
    // U2 pll
    float32 pll_input1 = U2_result[frameIndex];
    // pll 的结果
    pll_result1 = pll_Run(pll_input1, &pll1, &sogi1, &pid_pll1, &U2_d);
    // 用正弦便于判断正确
    pll_result1 = cos(pll_result1);
    changeDACAVal(2048 + 2000.0 * pll_result1);

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
      // pll 的结果
      pll_result1 = pll_Run(pll_input1, &pll1, &sogi1, &pid_pll1, &U2_d);
      // 用正弦便于判断正确
      pll_result1 = cos(pll_result1);
      changeDACAVal(2048 + 2000.0 * pll_result1);

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
    // pll 的结果
    pll_result1 = pll_Run(pll_input1, &pll1, &sogi1, &pid_pll1, &U2_d);
    // 用正弦便于判断正确
    pll_result1 = cos(pll_result1);
    changeDACAVal(2048 + 2000.0 * pll_result1);

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

  //
  // Probe: control step finished.  ISR raises GPIO24 at its entry, so the
  // high-time of the pulse shows the ADC->task->PWM-write end-to-end latency.
  //
  GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;
}

static void control_trip(void) {
  //
  // 超限: 关断MOSFET驱动并使能, 停止一切输出, 通知调试。
  //
  control_overrun_cnt++;
  GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;
  GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;
}
