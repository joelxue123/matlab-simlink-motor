# Green Joint Digital Twin Architecture Plan

本文记录当前目录中已经存在的 MBD 模块、电机模型和研究模型，并给出
`green-joint` 数字孪生的长期搭建路线。后续 AI 接手数字孪生任务时，先读本文件，
不要重新从零扫描和重建一套模型。

最终主线纪律入口：

```text
../green_joint_digital_twin/FINAL_MAINLINE_ARCHITECTURE.md
```

该文件优先级高于本文中的历史阶段性描述；如果发现冲突，以最终主线架构为准，
并同步修正本文。

## 目标

数字孪生不是单个 Simulink 模型，而是一条可逐步闭环的工程链：

```text
真实 green-joint 固件/硬件
  -> 日志与参数辨识
  -> MBD 控制器 core
  -> 平均电压 plant 快速闭环
  -> 开关级专项 plant 验证 PWM/ADC/死区
  -> 误差对齐与参数更新
  -> 生成代码回到固件 adapter
```

第一阶段目标不要做“大一统巨型模型”，但也不要继续制造测试孤岛。优先建立：

```text
可复用 controller module
  + 可复用 plant wrapper
  + 统一 test harness
  + scenario catalog
  + 日志对齐脚本
```

也就是先让电流环 PI 的 MBD 结果能在虚拟 plant 中复现，并能和硬件日志比较；
随后速度环、位置环只作为新 scenario 和 controller module 接入同一个 harness，不再复制新模型。

## 主线纪律

`green-joint` 数字孪生必须是一条从底层逐层衍生的主线，而不是多个互相平行的模型集合：

```text
PlantWrapper
  电机 + 驱动器 + 减速器 + 传感器/延时

CurrentController
  通过 Model Reference 复用 green_joint_current_loop_model

SpeedController
  只新增速度环逻辑，验证时必须接 CurrentController + PlantWrapper

PositionController
  只新增位置环逻辑，验证时必须接 SpeedController + CurrentController + PlantWrapper

TestSupervisor
  管理电流、速度、位置、饱和、故障等测试状态
```

因此：

- 电流环必须以 `green_joint_current_loop_mbd/green_joint_current_loop_model.slx`
  为唯一源头；digital twin 使用 Model Reference 和 dictionary reference，不复制
  `GreenJointCurrentLoopStep` 子系统。
- 速度环必须以 `motor_speed_pi_mbd/speed_pi_model.slx` 为可复用 core，并接入
  `green_joint_digital_twin/green_joint_average_motor_twin_model.slx` 做主线验证。
  独立速度 PI harness 只能做 core smoke，不作为物理结论。
- 位置环模型不应绕过速度环、电流环和同一个 plant wrapper。
- 新测试不应复制一个完整 `.slx` 再局部改名；应进入 scenario catalog 和统一 test harness。
- 可以为了简单验证临时新建小模型，但开始前要向用户说明，验证后必须回归本主线。
- 任何临时模型如果保留，必须标记为 `prototype` / `temporary` / `reference only`，并写清楚不能作为生产代码生成源。

这条纪律优先级高于“快速做出一个能跑的模型”。能跑但不能回归主线的模型，会增加后续维护成本。

## 速度环减速比规则

green-joint 当前按减速器输出端速度做速度环接口：

```text
SpeedPiStep.wm_ref  = joint-side speed, rad/s
SpeedPiStep.wm_meas = joint-side speed, rad/s
SpeedPiStep.iq_ref  = motor-side q-axis current, A
gear_ratio          = motor_speed / joint_speed = 183.35
```

因此速度环设计中从输出端速度误差到电机端 Iq 的等效惯量为：

```text
J_speed_loop = J_output_equivalent / gear_ratio
B_speed_loop = B_output_equivalent / gear_ratio
```

禁止两种常见错误：

```text
1. 直接用 J_motor 设计输出端速度环。
2. 把输出端等效惯量 J_output 直接代入电机端 Iq PI。
```

当前 1615 主线参数：

```text
J_motor = 0.034 kg*mm^2 = 3.4e-8 kg*m^2
gear_ratio = 183.35
J_output_equivalent = 0.00132792306138 kg*m^2
B_output_equivalent = 0.0109757550501 N*m*s/rad
Tc_output = 0.028677381561 N*m
Tbias_output = -0.0020638267429 N*m
J_speed_loop = 7.24255828e-6 kg*m^2
B_speed_loop = 5.98623128e-5 N*m*s/rad
Kt = 2.56e-3 / 0.4949 = 0.00517276217 N*m/A
Ts_speed = 100us
20Hz bring-up: Kp = 0.340319362, Ki = 22.1100241, Kaw = 125.663706
```

当前 1620 参数已记录但不是默认主线：

```text
J_motor = 0.058 kg*mm^2 = 5.8e-8 kg*m^2
Rll = 2.0 ohm
Lll = 55uH
Kt = 0.00517276217 N*m/A
```

速度环参数同步入口：

```bash
matlab -batch "run('matlab-practice/green_joint_digital_twin/sync_green_joint_speed_loop_twin_parameters.m')"
```

主线速度环仿真入口：

```bash
matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_average_motor_speed_step_test.m')"
```

物理 plant 阻尼规则：

```text
1615 motor.B 来自输出端辨识阻尼折算：motor.B = B_output / gear_ratio^2。
旧 average-inverter 示例中的 motor.B = 1e-4 N*m/(rad/s) 不适合 1615/1620 默认仿真。
该值会在 104.7rad/s 电机端速度产生约 0.0105N*m 阻尼转矩，
等价吃掉约 2A q 轴电流，导致速度环被假阻尼钉死。
```

## 已存在资产总览

### [KEEP] 可交付 MBD 控制模块

这些模块适合作为数字孪生中的控制器 core，后续也适合生成 C 接固件：

| 目录 | 角色 | 数字孪生用途 |
| --- | --- | --- |
| `green_joint_current_loop_mbd/` | green-joint 电流 PI-only 替换模块 | 当前主线，作为 Model Reference 接入 digital twin |
| `green_joint_mit_impedance_mbd/` | green-joint MIT 阻抗控制模块 | 当前主线，输出电机端 `iq_ref_a`，固件 adapter 已接入 |
| `motor_control_modules/` | 团队共享控制模块库 | 后续复用 Speed/Current PI、Clarke/Park、DqToDuty、DeadtimeComp |
| `motor_clarke_park_struct/` | Clarke/Park + Bus 类型示例 | 接入真实 `ia/ib/ic/theta_e` 反馈 |
| `motor_current_pi_mbd/` | 通用 dq Current PI | 可作为 green-joint PI 方案对照 |
| `motor_speed_pi_mbd/` | 通用 Speed PI | twin 扩展到速度环时使用 |
| `motor_float_open_loop_mbd/` | float-first 开环 motor skeleton | 平均 plant 骨架来源 |
| `motor_current_loop_mbd/` | 电流环平均闭环集成 | twin v0 的结构参考 |
| `motor_speed_current_loop_mbd/` | 速度 + 电流平均闭环集成 | twin v1/v2 的结构参考 |
| `motor_current_filter_mbd/` | green-joint dq 电流滤波 | 接近固件反馈路径时接入 |
| `pwm_deadtime_compensation_mbd/` | 死区补偿算法 core | 开关级验证成熟后作为 PWM adapter |
| `pwm_deadtime_sampling_mbd/` | 采样窗口有效性判定 | 建模 ADC/PWM 约束时使用 |

这些模块共同特点：

```text
.sldd / Bus / AliasType / Parameter
fixed-step discrete
smoke test
部分模块已有 ERT reusable codegen
平台无关，不直接依赖 MCU 寄存器
```

### [RESEARCH] 电机与逆变器 plant / 验证模型

这些模型适合做数字孪生的 plant 或专项验证 harness，但不要当作新的交付模块模板：

| 目录/模型 | 角色 | 数字孪生用途 |
| --- | --- | --- |
| `average-inverter/average_inverter_foc.slx` | 平均电压 FOC 研究主模型 | 理解旧 FOC 链路和 average inverter 语义 |
| `average-inverter/openloop_vf_test.slx` | 开环 VF 测试 | plant sanity check |
| `average-inverter/speedloop_kf_test.slx` | 速度环 + 估计器测试 | 速度环 twin 参考 |
| `average-inverter/vibration_comp_test.slx` | 周期扰动/齿槽补偿研究 | 负载扰动、齿槽转矩 twin 扩展 |
| `average-inverter/switching_sampling_study/*.slx` | 开关级 PWM/死区/采样研究 | 验证死区、低边采样窗口、PWM 纹波 |
| `adc_interrupt_current_loop_test/` | ADC/PWM 中断时序研究 | 建模控制延迟和 shadow update |
| `identification/pmsm_electrical_parameter_identification/` | PMSM 电气参数辨识 | 提供 Rs/Ld/Lq/flux/encoder offset |
| `motor_performance_characterization/` | 电流传感器/转矩曲线分析 | 用真实测试数据校准 twin |
| `hpm_ethercat_*` | 真实执行器数据分析 | 机械惯量、频响、台架日志对齐参考 |

这些模型的价值是“验证物理现象”和“对齐硬件数据”，不是生成固件 C。

### [TRANSITION] 可参考但不作为新主线

| 目录 | 角色 | 使用方式 |
| --- | --- | --- |
| `fixed_point_pi_q14_simulink/` | 定点 PI 学习与代码生成 | 后续定点化时参考，不作为 float twin v0 主线 |
| `average-inverter/algorithms/*.m` | 旧研究脚本和 MATLAB Function 函数 | 可读算法意图，成熟算法要迁移到独立 `*_mbd/` |
| `average-inverter/build_modules/*.m` | 旧式模型生成脚本 | 可参考连接关系，不作为新模块模板 |

### [LEGACY] 不进入当前数字孪生主线

| 目录 | 原因 |
| --- | --- |
| `dexterous_hand_impedance_plan/` | 机械臂/灵巧手旧路径，与 green-joint 电机 twin 主线不同 |
| `主动阻尼控制/` | 旧编码和旧模型路径，先保留历史价值 |

## 推荐数字孪生分层

### Layer 1：控制器 MBD Core

当前优先使用：

```text
green_joint_current_loop_mbd/GreenJointCurrentLoopStep
```

边界：

```text
输入：id_ref, iq_ref, id_fbk, iq_fbk, vbus
输出：vd_cmd, vq_cmd, vd_norm, vq_norm, voltage_mag
```

注意：

```text
只替换电流 PI 和 Vd 优先限幅。
不包含 ADC、Clarke/Park、电流滤波、inverse Park、SVPWM、PWM 写寄存器。
```

### Layer 2：平均电压 Plant

优先从这些模型借结构：

```text
motor_current_loop_mbd/
motor_speed_current_loop_mbd/
motor_float_open_loop_mbd/
average-inverter/
```

用途：

```text
快速闭环仿真。
整定 Kp/Ki/Kaw、VLimitRatio、滤波参数。
对比 50us current loop、100us speed loop、25us plant boundary。
```

不要在 v0 阶段引入开关器件细节。先让平均模型和硬件日志在主要动态上对齐。

### Layer 3：平台 Adapter

数字孪生中的 adapter 用于模拟固件边界：

```text
ADC/电流传感器缩放
Clarke/Park
dq current filter
inverse Park / dq-to-duty
SVPWM / duty [0, 1]
deadtime compensation
PWM shadow update / current-loop delay
```

已有模块：

```text
motor_clarke_park_struct/
motor_current_filter_mbd/
motor_control_modules/DqToAbcDutyStep
pwm_deadtime_compensation_mbd/
pwm_deadtime_sampling_mbd/
adc_interrupt_current_loop_test/
```

v0 不要一次全接。建议顺序：

```text
1. GreenJointCurrentLoopStep + ideal dq feedback
2. 加 DqToAbcDutyStep + Average Inverter + PMSM
3. 加 Clarke/Park feedback
4. 加 current filter
5. 加 PWM/ADC 延迟
6. 加 deadtime compensation 和 sampling window 判定
```

### Layer 4：参数与日志

数字孪生最终要靠真实数据收敛，而不是靠手感调参。

参数来源：

```text
identification/pmsm_electrical_parameter_identification/
motor_performance_characterization/
hpm_ethercat_* 数据分析目录
green-joint 固件参数和台架日志
```

必须统一物理量纲：

```text
current: A
voltage: V
speed: rad/s
angle: rad
resistance: ohm
inductance: H
flux linkage: Wb
sample time: s
duty: [0, 1]
```

不要继续用无量纲 PI 参数作为长期接口。无量纲旧参数只能作为反推和对照。

### Layer 5：测试 Harness 与状态管理

数字孪生不应靠大量孤立 `.slx` 管理测试。长期应建立统一 test harness：

```text
TestSupervisor
  -> ScenarioSelector
  -> ReferenceGenerator
  -> ControllerModel
  -> PlantWrapper
  -> Logger / Assessment
```

规则：

- 产品控制器只保留真实产品需要的状态和模式。
- 电流方波、阶跃、饱和退出、速度阶跃、位置阶跃等测试状态放在 harness。
- 简单测试用 Test Sequence 或 Signal Editor。
- 复杂切换、故障恢复、互锁流程用 Stateflow TestSupervisor。
- 大批量场景和 pass/fail 管理后续用 Simulink Test Manager。
- TestSupervisor 默认不生成到嵌入式代码，避免污染可交付 controller core。

详细长期规范见：

```text
docs/mbd_test_state_management_architecture.md
```

green-joint 的统一测试顶层目标：

```text
green_joint_control_test_harness.slx
  -> TestSupervisor / Test Sequence
  -> GreenJointControllerWrapper
  -> PlantWrapper
  -> LoggerAssessment
```

新增速度环测试时，不新建 `green_joint_speed_step_harness.slx` 这种孤岛模型。
正确动作是：

```text
1. 在 scenario catalog 增加 speed_step_*
2. 在 ControllerWrapper 接入 SpeedPiStep -> GreenJointCurrentLoopStep
3. 在 TestSupervisor 增加 SpeedStepTest 状态或 Test Sequence step
4. 在 LoggerAssessment 增加 wm_ref/wm_meas/iq_ref/pass-fail 指标
5. 复用同一个 PlantWrapper
```

## 推荐目录

新建数字孪生主目录：

```text
green_joint_digital_twin/
  README.md
  data/                 optional, ignored or small samples only
  results/              optional
  reports/              optional
  build_*_model.m       future
  run_*_smoke_test.m    future
  compare_*_log.m       future
```

当前已经建立：

```text
green_joint_digital_twin/README.md
green_joint_digital_twin/setup_green_joint_current_loop_twin.m
green_joint_digital_twin/sync_green_joint_current_loop_twin_parameters.m
green_joint_digital_twin/build_green_joint_current_loop_twin_model.m
green_joint_digital_twin/run_green_joint_current_loop_twin_smoke_test.m
green_joint_digital_twin/green_joint_current_loop_twin_model.slx
green_joint_digital_twin/build_green_joint_average_motor_twin_model.m
green_joint_digital_twin/run_green_joint_average_motor_twin_smoke_test.m
green_joint_digital_twin/green_joint_average_motor_twin_model.slx
green_joint_digital_twin/green_joint_average_motor_twin_interface.sldd
```

接下来应建立的长期目录：

```text
green_joint_digital_twin/
  scenarios/
    catalog.yaml 或 README.md
  test_harness/
    build_green_joint_control_test_harness.m
    open_green_joint_control_test_harness.m
    run_green_joint_control_scenario.m
  controller_wrapper/
    current_only
    speed_current
    mit_current
    position_speed_current
  plant_wrapper/
    average_v1
    switching_study
    log_replay
  results/
  reports/
```

已有 `green_joint_average_motor_square_wave_harness.slx` 是电流方波可视化验证资产，
后续应迁移进统一 `green_joint_control_test_harness`，不作为新增速度环/位置环测试模板。

## v0 最小闭环建议

第一版只追求可跑、可比较、可解释：

```text
iq_ref step / id_ref = 0
  -> GreenJointCurrentLoopStep
  -> vd_cmd/vq_cmd
  -> ideal dq voltage to average plant
  -> id/iq feedback
  -> compare with expected current-loop bandwidth
```

v0 输出：

```text
id/iq response
vd/vq command
voltage saturation flag or voltage_mag_norm
settling time / overshoot / steady-state error
```

当前 v0 状态：

```text
已完成 GreenJointCurrentLoopStep + 离散 dq 平均电压 plant 的最小闭环。
smoke test 已通过。
```

运行命令：

```bash
matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_current_loop_twin_smoke_test.m')"
```

当前结果：

```text
iq_ref final          = 3 A
iq final              = 3 A
iq tracking error     = 2.38419e-07 A
id final              = 0 A
vd final              = 0 V
vq final              = 3 V
voltage_mag_norm max  = 0.433276
```

v0 不做：

```text
完整 firmware 替换
开关 MOSFET 级仿真
死区补偿
ADC 中断延迟
速度环
齿槽补偿
真实 C adapter
```

## v1 平均电压电机模型

v1 已经接入当前仓库推荐的平均电压电机模型路线：

```text
GreenJointCurrentLoopStep
  -> DqToAbcDutyStep
  -> Average-Value Inverter
  -> Surface Mount PMSM
  -> MotorClarkeParkStep feedback
```

运行命令：

```bash
matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_average_motor_twin_smoke_test.m')"
```

当前结果：

```text
iq_ref final          = 3 A
id final              = 0.000295401 A
iq final              = 0.557888 A
vd final              = -0.249145 V
vq final              = 6.91952 V
wm final              = 47.9616 rad/s
voltage_mag_norm max  = 1
```

当前解释：

```text
v1 是自由转子 Average-Value Inverter + PMSM plant。
q 轴电流阶跃会产生转矩并加速转子，因此它验证的是平均电压电机模型链路可运行，
不是严格锁轴电流环带宽测试。
默认母线电压按 green-joint 12V 系统设置，不使用 motor_current_loop_mbd 的 68V 示例值。
当前电机参数按用户提供的线间 R=2ohm、L=55uH 设置；相参数为 Rs=1ohm、Ld=Lq=27.5uH。
v1 已触发电压上限，说明该工况主要用于链路验证，不应直接解读为稳定电流带宽结果。
```

## v1 硬件日志对齐

输入真实日志：

```text
time
id_ref / iq_ref
id_fbk / iq_fbk
vbus
vd/vq 或 duty
theta_e / speed, if available
```

比较：

```text
MBD PI 输出 vs 固件当前输出
仿真 id/iq vs 硬件 id/iq
滤波前后 id/iq
电压限幅触发次数
跟踪延迟和带宽估算
```

目标：

```text
把“人工调参数”变成“用日志反推参数，再用 twin 验证参数”。
```

## v2 平台细节增强

接入：

```text
current filter
Clarke/Park
DqToAbcDutyStep
ADC/PWM delay
deadtime compensation
sampling window valid
```

这个阶段再使用：

```text
average-inverter/switching_sampling_study/
adc_interrupt_current_loop_test/
pwm_deadtime_compensation_mbd/
pwm_deadtime_sampling_mbd/
```

## 后续 AI 工作规则

1. 不要从 `average-inverter/algorithms/*.m` 直接复制成新交付模块。
2. 不要在数字孪生 v0 中一次接入全部 FOC、SVPWM、死区和 ADC 时序。
3. 控制器 core 优先使用 `green_joint_current_loop_mbd/`。
4. plant 优先复用已有平均电压模型结构，不重新发明 PMSM plant。
5. 硬件相关内容留在 adapter 层，不污染 MBD controller core。
6. 新参数必须写单位，优先来自辨识或硬件日志。
7. 如果要重建 `.slx/.sldd`，先读 `docs/simulink_hang_troubleshooting.md`。
8. 每完成一个 twin milestone，更新 `docs/progress.md` 和本文件。

## 当前结论

当前目录已经具备建立数字孪生的主要积木：

```text
MBD controller core: 已有
平均电压 plant: 已有
开关级专项验证: 已有
参数辨识框架: 已有
真实日志分析入口: 部分已有
统一 twin harness: 待建立
硬件日志对齐流程: 待建立
green-joint 固件 adapter 闭环: 待建立
```

下一步最合理的工程动作：

```text
建立 green_joint_digital_twin 统一测试顶层：
先把已验证的 CurrentSquareTest / SaturationExitTest 收编为 scenarios，
再把 SpeedPiStep 作为外环模块接入同一 ControllerWrapper，
用同一个 Average-Value PMSM PlantWrapper 做速度环测试。
```
