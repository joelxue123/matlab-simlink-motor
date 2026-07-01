# Green Joint Final Mainline Digital Twin Architecture

本文是 `green-joint` 数字孪生的最终主线约定。后续人工或 AI 修改 MBD、生成代码、固件
adapter、参数辨识或测试场景前，必须先读本文。

目标不是再造一个更大的孤岛 `.slx`，而是建立一条从第一原理到固件代码的可追踪工程链：

```text
variant contract
  -> physical plant parameters
  -> reusable MBD controller cores
  -> green_joint_digital_twin unified harness
  -> generated C
  -> green-joint firmware adapter
  -> hardware log / identification feedback
```

## 第一原理边界

数字孪生必须显式表达这些物理层，而不是只调一组无量纲增益：

```text
DC bus / inverter:
  Vbus, modulation limit, SVPWM duty [0, 1], update delay

PMSM electrical plant:
  vd/vq -> id/iq
  R_phase = R_line_to_line / 2
  L_phase = L_line_to_line / 2
  Ts_current = 50 us

Mechanical plant:
  motor_speed = joint_speed * gear_ratio
  motor-side PMSM plant uses J_output / gear_ratio^2 and B_output / gear_ratio^2
  speed PI design uses J_output / gear_ratio and B_output / gear_ratio

Sensor / estimator:
  encoder motor mechanical angle rad
  PLL internal theta_hat [0, 2*pi)
  PLL angle error [-pi, pi)
  speed output joint-side rad/s for control

Controller:
  current loop output vd/vq in volt
  speed loop input joint-side rad/s, output motor-side iq_ref A
  MIT protocol torque is output-side Nm
  MIT current command uses Kt_output = Kt_motor * gear_ratio
```

禁止把以下内容混在一起：

```text
line-to-line R/L and phase R/L
motor-side speed and joint-side speed
motor-side torque/current and output-side torque
physical-domain gains and current-domain gains
temporary study model and production MBD source
```

## 唯一权威参数源

长期权威参数入口是 machine-readable variant contract：

```text
green-joint/Module/Config/green_joint_1615_config.json
green-joint/Module/Config/green_joint_1620_config.json
```

这些文件派生到：

```text
green-joint/Module/Config/*_config.h
green-joint/Module/Inc/green_joint_module_config.h
Simulink Data Dictionary .sldd defaults
green-joint/Module/MBD/* generated parameter defaults
green-joint/Module/Src/*_mbd_adapter.c runtime defaults
green-joint/docs/* parameter tables
matlab-practice/green_joint_digital_twin setup/sync scripts
```

不允许：

```text
在 MATLAB 脚本里手抄一套 J/R/L/Kt/gear_ratio 后长期使用。
在 C adapter 里手抄一套和 JSON 不一致的默认值。
只修改 .sldd 或 generated C，而不回写 variant contract 和同步脚本。
```

当前 1615 主线机械参数：

```text
gear_ratio = 183.35
Kt_motor = 0.00517276217 N*m/A
J_output = 0.00132792306138 kg*m^2
B_output = 0.0109757550501 N*m*s/rad
Tc_output = 0.028677381561 N*m
Tbias_output = -0.0020638267429 N*m
```

## 主线模型分层

最终主线不是单个巨型模型，而是强链接的可复用层：

```text
green_joint_control_test_harness
  TestSupervisor / scenario catalog
  GreenJointControllerWrapper
    SpeedEstimatorPllStep
    PositionLoopStep, future
    SpeedPiStep
    GreenJointCurrentLoopStep
    DqToAbcDutyStep
  PlantWrapper
    Average-Value Inverter
    Surface Mount PMSM
    gearbox / load / friction
    sensor / estimator boundary
  LoggerAssessment
```

原则：

```text
基础 MBD 模块只在自己的目录实现和 codegen。
digital twin 通过 Model Reference、Library Link 或等价强链接复用模块。
digital twin 不复制 PI 子系统、不复制 PLL 子系统、不复制 Bus/Parameter 定义。
新增测试先成为 scenario，再接入统一 harness。
临时 scratch/prototype 可以存在，但必须标记，结论必须回归主线。
```

## 当前模块权威映射

| 功能 | MBD 源头 | 固件 generated C | 固件 adapter / 调用点 | 状态 |
| --- | --- | --- | --- | --- |
| Current PI + voltage limit | `matlab-practice/green_joint_current_loop_mbd` | `green-joint/Module/MBD/green_joint_current_loop` | `green_joint_current_loop_mbd_adapter.c`, `foc.c` | 主线 |
| Speed PI | `matlab-practice/motor_speed_pi_mbd` | `green-joint/Module/MBD/green_joint_speed_loop` | `green_joint_speed_loop_mbd_adapter.c`, `speed_control()` | 主线 |
| Speed PLL | `matlab-practice/motor_speed_estimator_mbd` | `green-joint/Module/MBD/green_joint_speed_estimator` | `green_joint_speed_estimator_mbd_adapter.c`, ADC ISR | 主线 |
| Duty conversion | `matlab-practice/motor_float_open_loop_mbd` / shared motor modules | firmware FOC path | current digital twin uses same contract | 需继续收敛 |
| MIT impedance | `matlab-practice/green_joint_mit_impedance_mbd` | `green-joint/Module/MBD/green_joint_mit_impedance` | `green_joint_mit_impedance_mbd_adapter.c`, `foc.c::mit_control()` | 主线，兼容旧电流域协议 |
| Position loop | none | none | firmware mode logic | 待 MBD 化 |

MIT 和位置环不得另起炉灶。MIT 已建立独立 MBD core，并已通过
`green_joint_mit_impedance_mbd_adapter.c` 替换 `foc.c::mit_control()` 的手写公式。
当前 adapter 保持旧协议 `MIT_kp/MIT_kd = A/rad, A/(rad/s)`，内部乘
`Kt_output = Kt_motor * gear_ratio` 转成物理阻抗后送入 MBD core。位置环仍需先建立
独立 MBD core。

## 控制周期合同

当前主线周期：

```text
FOC/current loop: 50 us, 20 kHz
speed estimator PLL: 50 us unless explicitly changed by variant contract
speed PI target: 100 us
position loop: future, must state sample time explicitly
plant solver step: 5 us / 25 us depending on fidelity
```

跨速率边界必须显式表达：

```text
plant -> estimator
estimator -> speed PI
speed PI -> current PI
current PI -> inverter/plant
```

不允许通过“仿真能跑”隐式依赖 base workspace 或 inherited sample time。

## 代码生成纪律

MBD 交付模块必须满足：

```text
.sldd owns AliasType / Bus / Parameter
fixed-step discrete
ERT reusable or reentrant-friendly interface
no sample main
generated code kept under green-joint/Module/MBD/<module>
firmware adapter owns MCU/runtime integration, not algorithm rewrite
```

生成代码不是人工长期编辑区。若不得不修 generated C，应同时修改 MATLAB build/codegen
脚本并记录原因，否则下次 codegen 会丢失修复。

## 场景和验收

场景不是完整模型副本。每个 scenario 至少记录：

```text
scenario_name
control_mode
reference profile
sample times
controller modules used
plant variant
physical units
pass/fail metrics
result file naming
```

当前主线优先场景：

```text
current_square_1khz_0p3A_average_motor_v1
current_saturation_exit_4A_to_1p5A
speed_estimator_pll_interval_sweep
speed_estimator_pll_noise_sweep
speed_step_0_to_4radps_joint_average_motor_v1
speed_high_speed_voltage_limit_average_motor_v1
mit_impedance_1615_step
position_step, planned
```

当前 `mit_impedance_1615_step` 已进入统一 V1：

```text
green_joint_digital_twin/green_joint_average_motor_twin_model.slx
GJDT_ControlMode = GJDT_ControlModeMit
run_green_joint_average_motor_mit_step_test.m
```

MIT 输出已经接回：

```text
GreenJointMitImpedanceStep
  -> GreenJointCurrentLoopStep
  -> DqToAbcDutyStep
  -> Average-Value Inverter
  -> Surface Mount PMSM
```

旧的 `green_joint_mit_mode_1615_harness.slx` 只保留为过渡可视化模型，不再作为主线入口。

硬件日志必须最终参与闭环：

```text
simulation result
  -> same signal names / units as firmware log
  -> compare rise time, overshoot, settling, saturation, voltage limit, speed noise
  -> update identification or controller parameters
  -> sync back to variant contract and MBD dictionaries
```

## 新功能进入主线的流程

1. 先判断是否已有 MBD core 可复用。
2. 若只是新测试，新增 scenario，不新建完整 `.slx`。
3. 若是新算法，先建独立 `*_mbd` core，定义 `.sldd` 接口。
4. 在 `green_joint_digital_twin` 中通过 wrapper 强链接接入。
5. 用 average_v1 plant 验证物理动态。
6. 生成 C 到 `green-joint/Module/MBD/<module>`。
7. 写 firmware adapter，替换手写算法边界。
8. 跑 variant contract、MATLAB scenario、firmware build 验证。
9. 更新本文件、README、green-joint/docs 对应专题。

## 禁止事项

```text
禁止为了速度环、位置环、MIT 单独复制一整套电机 plant。
禁止在 digital twin 中绕过 PLL 直接用理想速度作为默认速度反馈。
禁止继续引入 legacy diff + IIR 速度滤波作为默认主线。
禁止把 1625 调出来的 Kp=1/Ki=20000 当成 1620 默认电流环参数。
禁止把 MIT_kp/MIT_kd 当成 Nm/rad 和 Nm*s/rad，当前固件仍是 A/rad 和 A/(rad/s)。
禁止用无单位“看起来像”的参数替代物理量纲设计。
```

这份文档的核心判断是：最终数字孪生必须由物理合同驱动，由 MBD core 实现，由
green-joint adapter 落地，由硬件日志校正。任何不能沿这条链路回溯的模型，都只能是临时研究资产。
