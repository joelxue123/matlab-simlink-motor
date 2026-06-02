# AI + Simulink + MCU 控制算法闭环流程

目标：把控制算法研发从“手写代码 + 经验调参”转成“模型驱动 + 数据闭环 + 自动回归”。

核心闭环：

```text
需求/问题
  -> AI 辅助建立 Simulink 模型
  -> MIL/SIL 仿真验证
  -> Simulink Coder 生成 .c/.h
  -> 移植到 MCU 工程
  -> 上位机回读实测数据
  -> Python/MATLAB 分析指标
  -> 更新模型、参数、控制策略
  -> 回归测试
  -> 下一轮迭代
```

## 1. 需求输入

每个算法任务先写成工程问题，而不是直接写控制器。

输入内容：

```text
控制对象：电机、电流环、位置环、灵巧手关节、阻抗控制等
目标：响应速度、稳态误差、抗扰、发热、电流峰值、稳定裕度
约束：采样周期、ADC/PWM 时序、MCU 算力、限幅、传感器噪声、通信带宽
测试：阶跃、负载扰动、延时扫描、噪声、饱和、断电恢复
验收指标：rise time、settling time、overshoot、RMS error、current RMS、temperature proxy
```

AI 的作用：

```text
把模糊问题转成可仿真的模型假设
列出控制策略候选
生成 Simulink 构建脚本
生成测试脚本和指标计算脚本
检查模型和代码生成风险
```

## 2. Simulink 模型设计

建议用脚本生成模型，而不是完全手工拖模块。

推荐结构：

```text
model/
  plant.slx 或 plant subsystem
  controller subsystem
  signal conditioning subsystem
  trigger / scheduler subsystem
  logging subsystem

scripts/
  build_model.m
  run_mil_tests.m
  compute_metrics.m
  generate_code.m

results/
  mil/
  sil/
  hil/
  mcu_log/
```

控制器模块要满足：

```text
输入输出固定
参数集中管理
限幅明确
状态变量明确
采样周期明确
可生成可重用函数
```

推荐控制器接口：

```c
void controller_step(const ControllerInput *in,
                     const ControllerParam *param,
                     ControllerState *state,
                     ControllerOutput *out);
```

在 Simulink 中对应：

```text
Bus input  -> Controller subsystem -> Bus output
参数 Bus   -> Controller subsystem
状态由模块内部 Unit Delay / Discrete Integrator / MATLAB Function persistent 保存
```

## 3. 仿真验证

先做 MIL，再做 SIL。

MIL：Model-in-the-loop。

```text
Simulink 原模型直接跑
验证控制逻辑和物理响应
```

SIL：Software-in-the-loop。

```text
生成 C 代码后由 Simulink 调用
验证生成代码和模型行为一致
```

当前最容易漏掉的地方：

```text
Simulink 生成了 .c/.h
但是没有再拿生成出来的 .c/.h 跑同一组输入
也没有和原 Simulink 模型输出逐点比较
```

这时证据链是不完整的。正确闭环应增加一个 SIL 关口：

```text
Simulink model
  -> run MIL, save input/output golden data
  -> generate C code
  -> compile generated C on host
  -> run generated C with same input sequence
  -> compare generated-C output against MIL golden output
  -> max error / mismatch count / pass-fail report
```

SIL 可以分两层：

```text
Level 1: Simulink 官方 SIL
  Simulink Coder 自动构建 SIL target
  在 Simulink 中用 generated code 替代原模型/子系统运行

Level 2: 外部 generated-C-in-loop
  把最终要移植的 .c/.h 编译成 MEX 或 host executable
  用 MATLAB/Python 喂同一组输入
  验证 MCU wrapper 级接口是否正确
```

两层都重要：

```text
官方 SIL 证明 Simulink 生成代码等价
外部 C-in-loop 证明你真正拿去移植的 .c/.h 接口可调用、状态复位正确、缩放正确
```

基础测试集：

```text
位置阶跃
电流阶跃
负载阶跃
外力/外力矩突加
采样延时扫描
参数摄动
饱和测试
噪声测试
长时间热负载测试
```

每个测试输出指标：

```text
rise_time
settling_time
overshoot
steady_state_error
rms_error
peak_current
rms_current
peak_torque
rms_torque
control_saturation_ratio
estimated_heat_power
phase_margin 或等效延时裕度
```

通过标准示例：

```text
稳态误差 < 目标阈值
超调 < 目标阈值
电流 RMS 降低
控制输出不长时间饱和
延时增加到指定值仍稳定
仿真结果和理论趋势一致
```

## 4. 代码生成

Simulink 配置重点：

```text
Solver: Fixed-step
采样周期: 与 MCU 中断周期一致
Controller subsystem: Reusable function
Function name: 用户指定，例如 usr_current_loop
接口: Bus / struct
数据类型: 明确 double / single / fixed-point
限幅和 anti-windup: 模型内显式表达
```

生成物：

```text
controller.c
controller.h
controller_types.h
rtwtypes.h
```

建议不要直接改生成代码。

正确做法：

```text
Simulink 生成算法核心
手写很薄的 adapter/wrapper
MCU 工程只调用 wrapper
```

示例：

```c
void CurrentLoop_Run(const AdcSample *adc,
                     const ControlParam *param,
                     PwmCommand *pwm)
{
    ControllerInput in;
    ControllerOutput out;

    in.i_ref = param->i_ref;
    in.i_meas = adc->phase_current;

    usr_current_loop(&in, param, &g_controller_state, &out);

    pwm->duty = voltage_to_pwm(out.v_cmd);
}
```

## 5. MCU 集成

MCU 工程中建议分层：

```text
drivers/
  adc.c
  pwm.c
  timer.c
  uart.c

generated/
  controller.c
  controller.h
  controller_types.h

app/
  current_loop_adapter.c
  control_scheduler.c
  telemetry.c
```

中断触发关系要和 Simulink 一致：

```text
PWM event / Timer event
  -> ADC sample
  -> ADC EOC or fixed control interrupt
  -> controller_step()
  -> PWM shadow update
  -> next PWM period effective
```

必须记录：

```text
实际中断周期
ADC 采样时刻
控制计算耗时
PWM 更新时刻
最大 jitter
```

这些会决定真实相位裕度。

## 6. 上位机回读数据

MCU 需要回传用于闭环分析的数据，不只是看波形。

推荐 telemetry 数据：

```text
timestamp
loop_counter
i_ref
i_meas
position_ref
position
velocity
torque_cmd
voltage_cmd
pwm_duty
load_estimate
controller_mode
saturation_flag
fault_flag
cpu_time_us
```

数据格式建议：

```text
CSV：简单调试
二进制帧：高速回读
MAT 文件：离线分析
```

上位机脚本职责：

```text
串口/CAN/USB 采集
保存原始数据
转换成标准表格
计算指标
和 Simulink 仿真结果对齐
生成报告图
```

## 7. 实测数据回灌模型

实测后不要只调代码，要回到模型。

回灌内容：

```text
真实电阻、电感、惯量、摩擦
真实采样延迟
真实 PWM 延迟
传感器噪声
量化误差
死区
饱和
温升导致的参数漂移
```

更新模型后重新跑：

```text
MIL
SIL
回归测试
```

目标是让模型越来越像真实系统，而不是让代码越来越复杂。

## 8. 控制策略更新

每次策略变化都必须说明：

```text
为什么改
改了什么
预期改善哪个指标
可能牺牲哪个指标
仿真是否支持
实测是否支持
```

常见策略池：

```text
PI + 饱和 + anti-windup
PD + DOB
阻抗控制
刚度调度
负载观测器
摩擦补偿
轨迹规划器
延时补偿
MPC
自适应控制
学习型参数整定
```

推荐落地顺序：

```text
1. PI/PD + 限幅 + anti-windup
2. DOB / 负载观测器
3. 刚度/阻尼调度
4. 非线性补偿
5. MPC / 强化学习等高阶策略
```

## 9. 回归测试

回归测试用于防止“这次调好了，别的工况坏了”。

每次修改后自动跑：

```text
baseline tests
delay scan
load scan
noise scan
parameter sweep
saturation tests
code generation test
SIL consistency test
```

回归结果保存：

```text
results/regression/YYYYMMDD_HHMMSS/
  metrics.csv
  plots/
  config.json
  model_version.txt
  generated_code_hash.txt
  sil_compare.mat
  sil_compare.csv
```

通过条件：

```text
关键指标不能劣化超过阈值
稳定性测试必须全部通过
代码生成必须通过
SIL 与 MIL 偏差必须在阈值内
生成 C 输出和模型输出必须逐点一致，或误差不超过定义的 LSB 阈值
```

## 10. AI 的最佳使用方式

不要只让 AI 写代码。更有价值的是让 AI 做这些事：

```text
把问题形式化
建立可检验假设
生成模型和测试
找出模型缺失项
分析仿真和实测差异
提出下一轮实验
维护回归测试
审查生成代码接口
把控制策略转成可证明的指标
```

人与 AI 的分工：

```text
人：定义真实问题、判断物理合理性、做实验、确认风险
AI：快速建模、生成测试、分析数据、整理证据链、维护工程闭环
Simulink：算法表达和代码生成
MCU：实时执行
上位机：数据采集和验证
```

## 11. 一个完整迭代例子

```text
问题：
  灵巧手抓包裹发热。

假设：
  目标位置和实际位置长期偏差导致 PD 输出持续电流。

模型：
  单关节电机 + 负载 + 位置 PD + 电流限制 + 热功率估计。

策略：
  PD -> PD + 死区/力矩限制 -> DOB+PD -> 阻抗/刚度调度。

仿真：
  抓取保持工况、外力阶跃、目标位置偏差、延时扫描。

生成代码：
  Controller subsystem 生成 reusable function。

MCU：
  ISR 中调用 controller_step。

回读：
  q_ref, q, qdot, tau_cmd, current, saturation_flag, temperature_proxy。

分析：
  对比电流 RMS、位置误差、抗扰恢复时间。

更新：
  修正摩擦、负载、延时模型，更新控制参数。

回归：
  确认发热降低，同时抓取稳定性没有明显下降。
```

## 12. 最小落地版本

第一阶段不要做太复杂。

最小可运行闭环：

```text
1. 一个 Simulink 单关节/电流环模型
2. 一个 PI 或 PD 控制器
3. 三个测试：阶跃、负载扰动、延时扫描
4. 一个 compute_metrics.m
5. 一个 generate_code.m
6. MCU wrapper 调用生成函数
7. 上位机回读 CSV
8. MATLAB/Python 分析 CSV
9. 回归脚本比较 metrics
```

等这个闭环跑通后，再加入：

```text
Bus/struct 接口
固定点
SIL/PIL
DOB
阻抗控制
自动参数扫描
更多 MCU 实测工况
```
