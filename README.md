# National_Diansai_Project
2023年国赛电源组项目

获得奖项：**全国一等奖**

## 本代码适用芯片
TI C2000系列 DSP28379\
只要符合思路中所述的标准，并且芯片运算速度足够（能够在一个采样周期内完成所有所需计算），那么无所谓用什么芯片。本项目中的代码修改后可以迁移到其他平台。
在本项目中，我将DSP28379的系统时钟频率从默认的200MHz降到了100MHz（节省功耗），PWM模块的频率为100MHz（需要设置从系统时钟频率到PWM模块频率的分频为1/1）。

## 控制策略
见Simulink文件

## 控制算法实现思路
先用Simulink搭仿真，然后照着仿真模型的控制部分进行编写。
基本思路：
1. 对现实系统的状态变量（电压，电流等）进行ADC采样
2. 基于采样结果进行控制相关的计算（**所有计算都必须在下一次采样之前全部完成**）
3. 根据计算结果，通过PWM模块输出到现实系统
4. 跳转到第1步

ADC与PWM的配置：
- PWM的载波设置为三角波（UPDOWN），载波周期为50us（20kHz），同一对PWM输出互补，两对PWM之间为单极性调制。
- 每到PWM的载波峰值时，触发ADC（SOC），接着ADC触发中断。这样，ADC的采样周期、单个开关管的开关周期和其中断函数的执行周期与PWM一致，都为Ts=50us，Ts是各个控制算法所必须的关键。Ts在代码中可调，在硬件允许的情况下尽量小。
- ADC的中断服务函数便是整个控制的关键了，**一切控制相关的计算都放在这里进行**。

关键在于PI控制器、PR控制器、SOGI以及PLL等算法如何实现：
- PI控制器：不说了，这个很简单，值得一提的是积分方式有两种可选，一是矩形积分，二是梯形积分。我选的是矩形积分，这其实完全够了。
- PR控制器：关键的传递函数G(s)不好拆分，我采取的方案是直接用MATLAB的c2d函数将连续形式G(s)暴力转成G(z)，离散化方法用Tustin(双线性变换法)。当然也可以根据G(s)手算精确的G(z)，参见自控原理的离散部分。
- SOGI：属于PLL的一部分。根据Simulink搭的模型写就行。
- PLL：这算是交流电题目都需要涉及到的关键，不仅需要根据Simulink搭的模型，还需要考虑到离散系统的特点。如果只是严格按照模型搭，会发现实际运行出来的结果居然超前所需跟踪的波两个采样周期Ts（在最终版本，采样周期为50us）。我就用了个“笨办法”（延迟队列）将SOGI的输出强行延迟了两个采样周期，这样最后输出的时候刚好抵消。后来发现超前问题其实是正常现象，也是符合理论和逻辑的（采样与PLL的输出之间有延迟），就连MATLAB自带的单相锁相环（虽然实现方式并不同）也用了同样的办法来解决这个问题。
- 以上所有涉及积分器的部分，都一定要注意积分初值和积分上限。这在实际运行中非常关键。

## 按键输入与OLED显示
采用DSP28379中的XBar模块进行外部中断的输入（按键）。
OLED屏用两个GPIO实现IIC通信。用OLED卖家提供的代码稍加修改即可使用。

## 引入FreeRTOS

原裸机架构：20kHz 控制计算全部在 `adca1_isr` 中，主循环只做 OLED 刷新和按键轮询。
改为 FreeRTOS（TI 官方 C28x 移植，`ThirdParty/FreeRTOS`，纯静态分配）：

```
PWM载波峰值 → ADC SOC → adca1_isr（仅采样+通知+探针） ──vTaskNotifyGiveFromISR──→ ControlTask（最高优先级）
                                                                                     ├─ 27us 原ISR逻辑原样执行
                                                                                     └─ 结束检测超限(45us阈值→关断GPIO0/2)
KeyTask (prio 3): 模式键轮询(GPIO124/125/29, 20ms) + XINT1~5事件组处理(原xint_isr逻辑)
UITask  (prio 1): OLED 10Hz刷新 (vTaskDelayUntil)
空闲任务 (prio 0): 空转 (vApplicationIdleHook)
```

### 关键点
- **任务优先级**：ControlTask=4 > KeyTask=3 > UITask=1；C28x 上所有 PIE 中断共走 INT1，
  ISR 期间 INTM=1，天然不会被调度器打断（27us 计算路径无 RTOS 干扰）。
- **单周期令牌**：ISR 每 50us notify 一次，ControlTask `ulTaskNotifyTake(pdTRUE,...)` 一通知一拍，
  结构上排除多拍堆积。通知值用 pdTRUE 清计数。
- **中断向量**：tick = CPU Timer2 (PIE 14.2, IER M_INT14)；
  RTOS-Yield = PIE 1.8 (WAKE_INT，组14/杂项组无 PIEIER 需直接使能 CPU 中断)；
  与 ADC(1.1)/XINT1-2(1.4/1.5)/XINT3-5(12.x) 不冲突。
  注意 `portdefines.h` 已将 `PORT_INT_YIELD` 硬编码为 PIE group1/vector8（本工程无 driverlib）。
- **内存布局**：C28x 的 SP 是 16 位，任务栈必须位于 64K 内 → `#pragma DATA_SECTION(..., ".freertosStaticStack")`
  排入 RAMGS2/RAMGS3（见 2837xD_FLASH/RAM_lnk_cpu1.cmd）；TCB/事件组等普通数据放 .bss/.ebss。
- **共享变量保护**：所有 32 位共享浮点（Uref、inverter_std_Io/K、time_elapsed 等）写入侧
  用 `taskENTER_CRITICAL()`（INTM 关闭，禁止上下文切换撕裂），ControlTask 读取侧天然安全
  （低优先级任务只能在 ControlTask 阻塞时运行）；UITask 读取侧同样加临界区。

### 调试手段
- GPIO22：原“算力探针”，仍在 ISR 入口 toggle（观察 ISR 本身负载）。
- GPIO24：RTOS 端到端探针，ISR 入口置位 / ControlTask 写完 PWM 后清零，
  高电平宽度 = 采样→任务切换→计算→PWM 写出的总延迟（预算 31~33us < 50us）。
- `control_overrun_cnt`：CpuTimer0 测量单周期执行时长，>45us 自动关断并计数，可在 CCS 观察。
- 栈溢出检测开启（configCHECK_FOR_STACK_OVERFLOW=2），溢出即关断 GPIO0/2。

### 编译
- 仅 CPU1_RAM / CPU1_FLASH 配置在当前工程中是完整可编译的（另外的 Debug/Release 为模板遗留配置）。
- FreeRTOS Config 位于 `ThirdParty/FreeRTOS/Source/portable/CCS/C2000_C28x/FreeRTOSConfig.h`。
## 心得

多读芯片的英文数据手册，多和硬件交流。
