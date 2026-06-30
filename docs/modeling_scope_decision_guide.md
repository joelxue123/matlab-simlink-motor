# Modeling Scope Decision Guide

本文件回答两个长期问题：

```text
1. 什么时候用平均电压模型，什么时候用开关型模型？
2. 什么模块按 MBD/codegen 标准做，什么模块保持原来的仿真/研究形态？
```

核心原则：

```text
控制算法用 MBD 沉淀。
电机 plant 按验证目标选择模型。
物理细节和探索脚本服务验证，不强行 codegen。
```

## 一句话决策

```text
如果问题和平均电压、控制环路、算法接口有关 -> 平均电压模型优先。
如果问题和 PWM 边沿、死区、采样窗口、电流纹波、开关器件有关 -> 开关型模型优先。
如果模块未来会交付给固件或同事复用 -> MBD。
如果模块只用于观察物理现象或做一次性研究 -> 保持仿真/脚本形态。
```

## 平均电压模型

平均电压模型把逆变器看成理想受控电压源或 Average-Value Inverter。它不解析每个 PWM 边沿，适合做控制算法主线。

适合使用平均电压模型的场景：

| 场景 | 原因 |
| --- | --- |
| 电流环 PI 初始整定 | 仿真快，能先看带宽、超调、饱和和 anti-windup |
| 速度环、电流环、电机集成 | 关注闭环动态，不需要每个 PWM 边沿 |
| Clarke/Park、SVPWM/duty 主算法验证 | 关注平均电压矢量和接口正确性 |
| MTPA、弱磁、速度阶跃、负载阶跃 | 时间尺度较长，开关级模型太慢 |
| 参数辨识算法初版 | 先验证估计算法和数据流，不被 PWM 纹波淹没 |
| 长时间仿真或批量扫参数 | 速度比开关级更重要 |
| MBD/codegen 模块集成 | 适合固定接口、采样时间和可重入 C 结构 |

平均电压模型不适合回答：

```text
死区导致的真实电流纹波有多大。
高占空比低边采样窗口是否足够。
中心采样和周期平均值差多少。
MOSFET/二极管导通路径如何切换。
单电阻/双电阻电流重构是否可行。
PWM 边沿、ADC 触发、采样保持延迟是否安全。
```

当前仓库代表：

```text
motor_float_open_loop_mbd/
motor_current_loop_mbd/
motor_speed_current_loop_mbd/
Motor Control Blockset Average-Value Inverter 相关模型
```

## 开关型模型

开关型模型显式描述 PWM 比较、死区、上下桥臂门极、MOSFET/二极管和电机绕组动态。它慢，但能回答平均模型无法回答的问题。

适合使用开关型模型的场景：

| 场景 | 原因 |
| --- | --- |
| 死区影响分析 | 死区是边沿级现象，平均模型无法自然产生 |
| 死区补偿极性和增益校准 | 需要观察补偿前后真实相电流/端电压趋势 |
| 高占空比采样窗口 | 低边导通窗口和 ADC settle time 是 PWM 时序问题 |
| PWM 中心采样误差 | 一个 PWM 周期内电流斜率很大时，采样点不等于周期平均 |
| 单电阻/双电阻采样重构 | 依赖具体开关状态和可采样窗口 |
| 共模电压、零相量分配、DPWM 对窗口的影响 | duty 绝对位置变化影响采样和共模 |
| 开关损耗、二极管续流、bootstrap 约束 | 必须看到器件导通状态 |
| 硬件异常复现 | 如窄脉冲、过调制、边沿抖动、采样错位 |

开关型模型不适合作为日常控制开发主模型：

```text
仿真慢。
参数更多，调试成本高。
结论容易被器件模型、步长、powergui、采样设置影响。
不适合长时间速度环/MTPA/弱磁大规模扫参。
不适合直接 codegen 成嵌入式控制算法。
```

当前仓库代表：

```text
average-inverter/switching_sampling_study/
run_switching_deadtime_motor_smoke_test.m
Universal Bridge MOSFET/Diodes + SPS PMSM
```

## 两类模型如何配合

推荐流程：

```text
1. 平均电压模型先跑通控制算法。
2. MBD 化稳定算法 core，并生成 C 检查接口。
3. 对 PWM/ADC/死区/采样窗口问题，建立开关级专项验证 harness。
4. 把开关级发现转成可交付 adapter 或补偿算法。
5. adapter/补偿算法再回到 MBD/codegen 模块。
```

例子：

```text
电流 PI:
  平均电压模型整定 Kp/Ki/Kaw
  -> motor_current_pi_mbd/ 生成 C
  -> 开关型模型只验证 PWM 纹波和采样影响

死区补偿:
  开关型模型观察死区造成的电流/电压偏差
  -> pwm_deadtime_compensation_mbd/ 做成可交付算法
  -> 开关型模型复测补偿效果

采样窗口 valid:
  开关型模型发现高占空比窗口不足
  -> pwm_deadtime_sampling_mbd/ 做成 ADC/current adapter 判定模块
```

## MBD 化的对象

必须优先 MBD 化：

| 模块 | 原因 |
| --- | --- |
| Clarke/Park | 稳定算法接口，固件必用 |
| PI 电流环/速度环 | 需要可重入 C 和饱和/anti-windup 可审查 |
| SVPWM/dq to duty | 输出 duty 合同必须稳定 |
| DeadtimeCompensationStep | 用户级补偿算法，可交付 |
| DeadtimeSamplingWindowStep | ADC/current adapter 判定，可交付 |
| observer / sensorless core | 后续要进固件和硬件验证 |
| 在线参数辨识 core | 输入输出、状态和数据类型必须稳定 |
| 传感器校正/滤波/健康监测 | 平台无关，适合复用 |

MBD 模块标准：

```text
.sldd 管接口合同。
Simulink.Bus 管结构体。
Simulink.AliasType/NumericType 管类型。
Simulink.Parameter 管参数。
固定采样时间。
可重复 build/test/codegen。
生成 ERT reusable/reentrant C。
不包含 MCU 寄存器、HAL、芯片头文件。
```

## 保持原来形态的对象

不强行 MBD/codegen 的对象：

| 模块 | 推荐形态 |
| --- | --- |
| Universal Bridge、MOSFET、Diodes | Simscape/SPS plant |
| PMSM、RL winding、DC bus 物理 plant | Simulink/Simscape/SPS plant |
| PWM 边沿级时序观察 | Simulink harness 或 MATLAB 脚本 |
| powergui、Scope、To Workspace、plot | 验证/可视化工具 |
| 参数扫描、论文复现、一次性对比 | MATLAB script |
| 真实 CSV 数据分析 | MATLAB script/report |
| 平台寄存器配置、HAL、驱动初始化 | 固件平台适配层，不放进算法 MBD core |

判断理由：

```text
这些对象的价值是复现物理现象、观察波形、发现边界条件。
它们不是要交付给固件的算法接口。
强行 MBD/codegen 会增加成本，不会提高工程沉淀质量。
```

## 模块归属表

| 对象 | 平均模型 | 开关模型 | MBD/codegen | 备注 |
| --- | --- | --- | --- | --- |
| Current PI | Yes | Optional verify | Yes | 主开发在 MBD，开关模型看纹波/采样影响 |
| Speed PI | Yes | No | Yes | 时间尺度长，不需要开关级 |
| Clarke/Park | Yes | No | Yes | 算法 core |
| SVPWM/duty | Yes | Optional verify | Yes | `[0,1]` duty 合同稳定 |
| Zero-vector allocation | Yes | Yes | Adapter candidate | 平均模型看电压，开关模型看窗口/共模 |
| Deadtime compensation | Optional | Yes verify | Yes | 策略交付 MBD，效果验证用开关型 |
| Current valid/window | Optional | Yes source | Yes | 判定逻辑可 MBD 化 |
| PMSM plant | Yes | Yes | No | 根据验证目标选平均或开关 |
| MOSFET/Diode bridge | No | Yes | No | 物理验证 harness |
| ADC/PWM register write | No | No | No | 平台适配层 |

## 采样时间基线

当前电机控制学习基线：

```text
25us PWM tick / plant boundary
50us current loop / deadtime compensation / current valid
100us speed loop
```

规则：

```text
平均模型中，多速率边界用 Rate Transition。
开关模型中，powergui 和 fixed-step 要足够小，服务 PWM 边沿解析。
不要用 Zero-Order Hold 伪装跨任务同步。
不要把开关级模型的微小步长带回控制算法 codegen 模块。
```

## 开发检查清单

新任务开始时先问：

```text
1. 我现在要回答的是控制平均效果，还是 PWM/器件/采样细节？
2. 这个模块未来会不会交付给固件或同事复用？
3. 接口是否稳定到值得建 Bus/.sldd？
4. 是否需要生成 C？
5. 需要跑多久？是否会因为开关级模型太慢影响迭代？
6. 结论是否需要硬件实测闭环验证？
```

对应选择：

```text
控制平均效果 + 长时间仿真 -> 平均电压模型。
PWM/器件/采样细节 -> 开关型模型。
可交付算法接口 -> MBD/codegen。
验证物理现象或临时扫参 -> 原有仿真/脚本形态。
```

