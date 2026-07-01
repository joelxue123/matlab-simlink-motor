# Green Joint Digital Twin

本目录是 `green-joint` 数字孪生的主入口。

## 必读入口

先读最终主线约定：

```text
FINAL_MAINLINE_ARCHITECTURE.md
```

该文件定义第一原理物理边界、variant 合同、MBD 模块权威源、生成代码落点、
统一 test harness 和禁止事项。后续不要绕过这份主线约定新建孤岛模型。

## 主线纪律

本目录后续按统一主线推进，不再默认新建孤岛模型：

```text
电机/驱动器/减速器 plant
  -> 电流环
  -> 速度环
  -> 位置环
  -> TestSupervisor / scenarios
```

重新造模型前必须先说明原因、范围和回归方式。允许为了简单验证建立临时
`prototype/temporary/scratch` 模型，但验证结束后必须把结论回归到统一
ControllerWrapper、PlantWrapper、TestSupervisor 和 scenario catalog。

当前已经有两层模型：

```text
v0: green_joint_current_loop_model Model Reference + 简化 dq 平均电压 plant
v1: green_joint_current_loop_model Model Reference + DqToAbcDutyStep + Average-Value Inverter + Surface Mount PMSM + MotorClarkeParkStep
```

v1 使用的是 `matlab-practice` 已有平均电压电机模型路线，不再停留在手写 dq plant。
电流环通过 Model Reference 引用 `../green_joint_current_loop_mbd/green_joint_current_loop_model.slx`，
并通过 dictionary reference 使用 `green_joint_current_loop_interface.sldd`。digital twin 不再复制
`GreenJointCurrentLoopStep` 子系统，也不再复制电流环的 GJ 类型、Bus 和 Parameter 定义。

当前主线 V1 顶层入口是：

```text
green_joint_average_motor_twin_model.slx
```

该模型用 `GJDT_ControlMode` 统一管理控制源：

```text
0 = direct current / current-loop test
1 = SpeedPiStep -> current loop
2 = GreenJointMitImpedanceStep -> current loop
```

无论哪种模式，最终都进入同一条物理链：

```text
selected iq_ref
  -> GreenJointCurrentLoopStep
  -> DqToAbcDutyStep
  -> Average-Value Inverter
  -> Surface Mount PMSM
  -> SpeedEstimatorPllStep / ClarkePark feedback
```

旧的 `green_joint_mit_mode_1615_harness.slx` 只是过渡可视化模型，不再作为 MIT 主线入口。

## 目标

建立一个能逐步对齐真实 `green-joint` 固件和硬件数据的数字孪生：

```text
green-joint 固件/硬件日志
  -> 参数辨识与单位统一
  -> MBD 控制器 core
  -> Simulink plant
  -> 仿真/日志对比
  -> 参数更新
  -> 生成 C + 固件 adapter
```

长期开发原则：

```text
基础模块复用优先。
新增测试先成为 scenario，再进入统一 test harness。
不为速度环、位置环等单个测试复制一套完整 .slx。
当前已有独立 harness 只作为迁移来源，不作为新测试模板。
```

目标顶层：

```text
green_joint_control_test_harness
  TestSupervisor / Test Sequence
  GreenJointControllerWrapper
  PlantWrapper
  LoggerAssessment
```

## 当前推荐 v0

第一版只做电流环 PI 的数字孪生，不替换完整 FOC：

```text
id_ref / iq_ref / id_fbk / iq_fbk / vbus
  -> green_joint_current_loop_mbd/green_joint_current_loop_model Model Reference
  -> vd_cmd / vq_cmd
  -> 简化 dq 平均 plant
  -> id/iq feedback
```

v0 关注：

```text
电流环跟踪速度
PI 参数物理量纲
Vd 优先限幅
anti-windup 行为
voltage_mag_norm 是否接近饱和
```

v0 不做：

```text
ADC/PWM 寄存器
完整 Clarke/Park + inverse Park + SVPWM 替换
开关 MOSFET 级仿真
死区补偿
速度环
齿槽转矩补偿
```

## v0 文件

```text
setup_green_joint_current_loop_twin.m
sync_green_joint_current_loop_twin_parameters.m
build_green_joint_current_loop_twin_model.m
run_green_joint_current_loop_twin_smoke_test.m
green_joint_current_loop_twin_model.slx
build_green_joint_average_motor_twin_model.m
run_green_joint_average_motor_twin_smoke_test.m
sync_green_joint_speed_loop_twin_parameters.m
run_green_joint_average_motor_speed_step_test.m
run_green_joint_average_motor_mit_step_test.m
green_joint_average_motor_twin_model.slx
green_joint_average_motor_twin_interface.sldd
```

初始化参数：

```matlab
run('matlab-practice/green_joint_digital_twin/setup_green_joint_current_loop_twin.m')
```

构建模型：

```bash
matlab -batch "run('matlab-practice/green_joint_digital_twin/build_green_joint_current_loop_twin_model.m')"
```

在 MATLAB Desktop 里查看已生成模型：

```matlab
run('matlab-practice/green_joint_digital_twin/build_green_joint_current_loop_twin_model.m')
```

如果 `green_joint_current_loop_twin_model.slx` 已存在，脚本会先恢复 `GJDT_*`
参数，再只打开模型，不会删除重建。

如果你手动清空了 base workspace，模型提示找不到 `GJDT_StopTime`、`GJDT_Ts` 等变量，
运行：

```matlab
run('matlab-practice/green_joint_digital_twin/setup_green_joint_current_loop_twin.m')
```

运行 smoke test：

```bash
matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_current_loop_twin_smoke_test.m')"
```

构建 v1 平均电压电机模型：

```bash
matlab -batch "run('matlab-practice/green_joint_digital_twin/build_green_joint_average_motor_twin_model.m')"
```

运行 v1 smoke test：

```bash
matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_average_motor_twin_smoke_test.m')"
```

同步 green-joint 速度环设计参数到可复用 SpeedPiStep 字典：

```bash
matlab -batch "run('matlab-practice/green_joint_digital_twin/sync_green_joint_speed_loop_twin_parameters.m')"
```

运行 v1 物理模型速度环阶跃测试：

```bash
matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_average_motor_speed_step_test.m')"
```

运行 v1 物理模型 MIT 位置阶跃测试：

```bash
matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_average_motor_mit_step_test.m')"
```

评估固件当前速度滤波器对速度环相位余量的影响：

```bash
matlab -batch "run('matlab-practice/green_joint_digital_twin/analyze_green_joint_speed_filter_phase_margin.m')"
```

验证 ODrive-style PLL 速度估算器候选：

```bash
matlab -batch "run('matlab-practice/motor_speed_estimator_mbd/run_speed_estimator_pll_smoke_test.m')"
matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_speed_estimator_pll_interval_sweep.m')"
matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_speed_estimator_wrap_contract_test.m')"
```

主线连接：

```text
motor_speed_estimator_mbd/SpeedEstimatorPllStep
  -> green_joint_digital_twin ControllerWrapper/TestHarness
  -> green-joint/Module/MBD/green_joint_speed_estimator
  -> green_joint_speed_estimator_mbd_adapter.c
  -> Core/Src/main.c
```

不要在速度环或位置环测试里复制 PLL 估算器。需要新测试场景时，只能引用
`SpeedEstimatorPllStep` 或其 adapter 等价接口。

运行 v1 物理模型 1kHz 电流方波测试：

```bash
matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_average_motor_square_wave_test.m')"
```

构建可打开的 v1 物理模型 1kHz 电流方波 `.slx` harness：

```bash
matlab -batch "run('matlab-practice/green_joint_digital_twin/build_green_joint_average_motor_square_wave_harness.m')"
```

运行可视化 harness 的自动验证：

```bash
matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_average_motor_square_wave_harness_test.m')"
```

在 MATLAB Desktop 中手动运行：

```matlab
run('/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/open_green_joint_average_motor_square_wave_harness.m')
sim_result = sim('green_joint_average_motor_square_wave_harness', 'ReturnWorkspaceOutputs', 'on');
open_system('green_joint_average_motor_square_wave_harness/IqRef_Iq_Scope')
```

注意：

```text
green_joint_average_motor_twin_model.slx 是基础 V1 物理模型，默认还是 step 输入。
green_joint_average_motor_square_wave_harness.slx 才是可打开查看 TestSupervisor 的 1kHz 方波测试模型。
如果看不到 Stateflow/TestSupervisor，通常是打开了基础 twin，而不是 square_wave_harness。
这个 square_wave_harness 是过渡期可视化验证资产；后续新增速度环、位置环测试不要复制它，
而是接入统一 green_joint_control_test_harness 架构。
```

## 中长期统一测试 Harness

后续新增测试按这个方向实现：

```text
scenarios/
  current_square_1khz_0p3A
  current_saturation_exit_4A_to_1p5A
  speed_step_0_to_4radps_joint
  speed_sweep_low_frequency

GreenJointControllerWrapper
  SpeedEstimatorPllStep
  SpeedPiStep
  GreenJointCurrentLoopStep
  DqToAbcDutyStep

PlantWrapper
  Average-Value Inverter + Surface Mount PMSM

LoggerAssessment
  iq_ref / iq
  wm_ref / wm_meas
  vd / vq / voltage_mag_norm
  duty_a / duty_b / duty_c
```

速度环测试接入方式：

```text
TestSupervisor.joint_speed_ref_rad_s
  + PlantWrapper.motor_angle_rad
  -> SpeedEstimatorPllStep
  -> joint_speed_est_rad_s
  -> SpeedPiStep
  -> motor-side iq_ref
  -> Rate Transition, TsSpeed=100us to TsCurrent=50us
  -> GreenJointCurrentLoopStep
  -> DqToAbcDutyStep
  -> PlantWrapper
  -> motor_angle_rad feedback to SpeedEstimatorPllStep
```

当前 `green_joint_average_motor_twin_model.slx` 已按上述主线连接：

```text
Surface Mount PMSM MtrPos
  -> MotorAngle_RateTransition_50us
  -> SpeedEstimatorPllStep Model Reference
  -> joint_speed_est_rad_s
  -> JointSpeed_RateTransition_TsSpeed
  -> SpeedPiStep Model Reference
```

`joint_speed_ideal_rad_s = plant motor speed / gear_ratio` 只作为对照日志，
不能再作为速度环反馈。

最近一次完整 V1 速度阶跃验证：

```text
script: matlab-practice/green_joint_digital_twin/run_green_joint_average_motor_speed_step_test.m
motor: 1615
ref: 0 -> 4 rad/s joint-side
feedback: SpeedEstimatorPllStep joint_speed_est_rad_s
Ts_speed: 100 us
PLL design bandwidth: 360 Hz
final estimator error: about 5.5e-6 rad/s
max transient estimator error: about 0.777 rad/s
overshoot: about 17.73 %
settling time: about 31.2 ms
voltage_mag_norm max: about 0.679
result csv: results/speed_step_0_to_4radps_joint_average_motor_v1.csv
```

这个 transient estimator error 是完整闭环下真实暴露出来的 PLL 动态误差，
后续如果调 PLL 带宽或速度环带宽，必须一起观察该指标和速度超调。
`600Hz` 无噪声仿真超调更低，但实机反馈速度噪声偏大；当前硬件 bring-up 默认改为
`360Hz`。

当前速度环单位约定：

```text
SpeedPiStep 输入 wm_ref / wm_meas 使用减速器输出端 joint-side rad/s。
SpeedPiStep 输出 iq_ref 使用电机端 q 轴电流 A。
gear_ratio = motor_speed / joint_speed = 183.35。

速度环设计等效惯量：
  J_speed_loop = J_output_equivalent / gear_ratio
  B_speed_loop = B_output_equivalent / gear_ratio

不要直接用 raw J_motor，也不要把输出端等效惯量 J_output 直接配电机端 Iq。
```

速度环测试必须记录：

```text
scenario name
Ts_speed / Ts_current / Ts_plant
wm_ref, joint-side rad/s
wm_meas, joint-side rad/s
iq_limit, unit A
speed Kp/Ki/Kaw
current Kp/Ki/Kaw
pass/fail metrics
```

当前基于用户提供的转子惯量 `0.034 kg*mm^2` 的速度环设计入口：

```bash
matlab -batch "run('matlab-practice/green_joint_digital_twin/design_green_joint_speed_loop.m')"
```

推荐先用 20Hz 作为硬件 bring-up 起点：

```text
J_motor = 3.4e-8 kg*m^2
gear_ratio = 183.35
J_output_equivalent = 0.00132792306138 kg*m^2
B_output_equivalent = 0.0109757550501 N*m*s/rad
Tc_output = 0.028677381561 N*m
Tbias_output = -0.0020638267429 N*m
J_speed_loop = 7.24255828e-6 kg*m^2
B_speed_loop = 5.98623128e-5 N*m*s/rad
Kt_default = 2.56e-3 / 0.4949 = 0.00517276217 N*m/A
Kp_speed = 0.340319362 A/(rad/s)
Ki_speed = 22.1100241 A/rad
Kaw_speed = 125.663706 1/s
iq_limit_initial = 0.1 A
```

40Hz 可作为硬件验证后的候选值：

```text
Kp_speed = 0.692211324 A/(rad/s)
Ki_speed = 88.4400964 A/rad
Kaw_speed = 251.327412 1/s
```

注意：

```text
速度环设计使用 joint-side speed -> motor-side Iq 的等效惯量。
如果后续识别出输出端负载惯量：
  J_speed_loop = J_output_equivalent / gear_ratio
  B_speed_loop = B_output_equivalent / gear_ratio
Kp_speed/Ki_speed 按 J_speed_loop 线性放大，Kp 同时扣除 B_speed_loop/Kt。
1615 motor.B 来自输出端辨识阻尼折算；不要继承旧 average-inverter 示例的 1e-4 阻尼。
```

当前 V1 主线速度阶跃结果：

```text
scenario = speed_step_0_to_4radps_joint_average_motor_v1
joint_speed_ref = 0 -> 4 rad/s
iq_limit = 2 A for 1615 current variant contract; historical 4 A result should be rerun after limit-contract changes
final joint speed = 4 rad/s
final speed error = 0 rad/s
rise time to 90% = 6 ms
settling time = 30.3 ms
overshoot = 15.5438 %
|iq_ref| max = 1.39905 A
|iq| max = 1.45208 A
voltage_mag_norm max = 0.743815
```

速度滤波器相位余量评估结果：

```text
script: analyze_green_joint_speed_filter_phase_margin.m
legacy firmware estimator: 400 us angle-difference window + alpha=0.1 IIR
speed PI sample time: 100 us

no speed filter:
  gain crossover ~= 40.57 Hz
  phase margin   ~= 69.66 deg

legacy alpha=0.1:
  gain crossover ~= 32.56 Hz
  phase margin   ~= 29.75 deg
  phase margin loss ~= 39.91 deg
```

结论：

```text
旧速度滤波器是速度环相位余量的主要消耗项，已从固件主线移除。
`speed_estimator_alpha` 只作为旧 diff + IIR 链路的离线 baseline 参数保留。
继续提高速度环带宽前，应优先验证 ODrive-style `SpeedEstimatorPllStep` 的噪声、相位和低速抖动。
```

默认场景：

```text
scenario: current_square_1khz_0p3A_average_motor_v1
iq_ref:   +/-0.3 A
period:   1 ms full period, 0.5 ms half-period
Ts:       50 us current loop, 5 us plant
Kp/Ki:    default comes from green-joint Module/Config variant JSON
plant:    V1 Average-Value Inverter + Surface Mount PMSM
```

输出：

```text
results/current_square_1khz_0p3A_average_motor_v1.png
results/current_square_1khz_0p3A_average_motor_v1.csv
```

当前结果：

```text
iq positive peak       = 0.333138 A
iq negative peak       = -0.333407 A
iq peak-to-peak        = 0.666545 A
iq/ref p-p gain        = 1.11091
gain@1kHz fundamental  = 0.997516
lag@1kHz fundamental   = 54.7103 deg / 151.973 us
voltage_mag_norm max   = 0.164503
```

### 1620 电流环带延时 PI 候选

主线设计脚本：

```bash
matlab -batch "run('matlab-practice/green_joint_digital_twin/sweep_green_joint_1620_current_loop_delay_pm_candidates.m')"
GJDT_CURRENT_TARGET_BW_HZ=1200 matlab -batch "run('matlab-practice/green_joint_digital_twin/design_green_joint_current_loop_with_delay.m')"
GJDT_CURRENT_TARGET_BW_HZ=1500 matlab -batch "run('matlab-practice/green_joint_digital_twin/design_green_joint_current_loop_with_delay.m')"
GJDT_CURRENT_TARGET_BW_HZ=2000 matlab -batch "run('matlab-practice/green_joint_digital_twin/design_green_joint_current_loop_with_delay.m')"
```

设计口径：

```text
formula: bandwidth + phase margin + explicit delay
PM = 60/65/70 deg sweep
Td = 75 us default, with 50/75/100 us sensitivity
R/L = phase values from green-joint Module/Config JSON
```

1620 候选结果：

```text
1200 Hz, PM70, Td75us:
  Kp = 0.417244 V/A
  Ki = 7028.23 V/(A*s)
  V1 1kHz square: peak ~= +0.3211 / -0.3211 A
  fundamental gain ~= 0.972, lag ~= 145.5 us
  voltage_mag_norm max ~= 0.057

1500 Hz, PM70, Td75us:
  Kp = 0.592975 V/A
  Ki = 7972.47 V/(A*s)
  V1 1kHz square: peak ~= +0.3367 / -0.3369 A
  fundamental gain ~= 0.961, lag ~= 123.9 us
  voltage_mag_norm max ~= 0.060

1500 Hz, PM65, Td75us:
  Kp = 0.516993 V/A
  Ki = 8429.21 V/(A*s)
  V1 1kHz square: peak ~= +0.3720 / -0.3714 A
  fundamental gain ~= 1.023, lag ~= 123.6 us
  voltage_mag_norm max ~= 0.065

1500 Hz, PM60, Td75us:
  Kp = 0.437076903 V/A
  Ki = 8821.80753 V/(A*s)
  V1 1kHz square: peak ~= +0.4197 / -0.4177 A
  fundamental gain ~= 1.084, lag ~= 122.6 us
  voltage_mag_norm max ~= 0.073

2000 Hz, PM60, Td75us:
  Kp = 0.72243529 V/A
  Ki = 9713.6457 V/(A*s)
  V1 1kHz square: peak ~= +0.4307 / -0.4336 A
  fundamental gain ~= 1.008, lag ~= 104.1 us
  voltage_mag_norm max ~= 0.075

1620 current variant default, 800Hz:
  Kp = 0.138230076758 V/A
  Ki = 5026.54824574 V/(A*s)
  V1 1kHz square: peak ~= +0.3229 / -0.3234 A
  fundamental gain ~= 0.943, lag ~= 210.9 us
  voltage_mag_norm max ~= 0.058

1625-derived Kp=1/Ki=20000:
  V1 1kHz square: peak ~= +/-4.14 A
  forbidden for 1620 default
```

当前判断：

```text
PM70 明显压低 1kHz 方波过冲，优于 PM60/PM65。
1200Hz PM70 的峰值接近 800Hz default，但滞后从约 210.9us 降到约 145.5us。
1500Hz PM70 更快，滞后约 123.9us，峰值约 +/-0.337A。
推荐下一轮硬件候选优先级：
  1. 1200Hz PM70 Td75us: 稳健、峰值低，适合作为首个硬件试验值。
  2. 1500Hz PM70 Td75us: 更快，作为第二候选。
  3. 1500Hz PM65/PM60: 暂不推荐，峰值明显更高。
硬件评审前仍不直接替换 1620 variant default。
```

### 1615 电流环带延时 PI 候选

1615 复用同一套扫描脚本，但必须显式指定 motor variant：

```bash
GJDT_MOTOR_TYPE=1615 matlab -batch "run('matlab-practice/green_joint_digital_twin/sweep_green_joint_1620_current_loop_delay_pm_candidates.m')"
GJDT_MOTOR_TYPE=1615 GJDT_CURRENT_TUNING_CASE=1615_1200hz_pm70_td075us matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_average_motor_square_wave_test.m')"
GJDT_MOTOR_TYPE=1615 GJDT_CURRENT_TUNING_CASE=1615_1500hz_pm70_td075us matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_average_motor_square_wave_test.m')"
```

1615 候选结果：

```text
1615 current variant default, current tuned value:
  Kp = 1.0 V/A
  Ki = 20000 V/(A*s)
  V1 1kHz square: peak ~= +0.3331 / -0.3334 A
  fundamental gain ~= 0.998, lag ~= 152.0 us
  voltage_mag_norm max ~= 0.165

1200 Hz, PM70, Td75us:
  Kp = 1.04616 V/A
  Ki = 20653.5 V/(A*s)
  V1 1kHz square: peak ~= +0.3392 / -0.3393 A
  fundamental gain ~= 1.007, lag ~= 147.6 us
  voltage_mag_norm max ~= 0.167

1500 Hz, PM70, Td75us:
  Kp = 1.52321 V/A
  Ki = 23812.3 V/(A*s)
  V1 1kHz square: peak ~= +0.3434 / -0.3434 A
  fundamental gain ~= 0.999, lag ~= 124.7 us
  voltage_mag_norm max ~= 0.169
```

当前判断：

```text
1615 和 1620 不同：现有 Kp=1/Ki=20000 在 1615 V1 模型里是可工作的。
1200Hz PM70 相比 default 改善很小，不值得优先替换。
1500Hz PM70 能把滞后从约 152us 降到约 125us，峰值只从约 +/-0.333A 增到 +/-0.343A。
因此 1615 的合理候选是：
  1. 保留现有 default: 最稳妥，不改变现有行为。
  2. 1500Hz PM70 Td75us: 作为评审后的性能升级候选。
不要把 1620 的禁用结论直接套到 1615；Kp=1/Ki=20000 对 1620 危险，但对 1615 当前模型可用。
```

注意：

```text
会写同一个 .sldd 的 MATLAB/Simulink 验证不要并行跑。
本目录 V1 方波脚本会同步 green_joint_average_motor_twin_interface.sldd，
并行跑不同 tuning case 会互相污染结果。
```

## PiCorrectionGain 扫描验证

验证脚本：

```bash
matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_pi_correction_gain_sweep.m')"
```

验证目标：

```text
确认 gj_mbd_pi_correction_gain = 400.0f 作为 back-calculation anti-windup 增益是否合理。
```

公式：

```text
integrator += Ts * (Ki * error + Kaw * (u_sat - u_pre_sat))
Kaw = PiCorrectionGain
```

当前控制周期 `Ts=50us`，所以：

```text
Kaw = 400 1/s
Kaw * Ts = 0.02
anti-windup time constant ~= 1 / 400 = 2.5 ms
```

扫描工况：

```text
Kp/Ki = 1.0 / 20000.0
iq_ref = 4.0 A -> 1.5 A at 8 ms
Vbus = 12 V
voltage_limit = 12 * 0.577 * 0.9 = 6.2316 V
```

关键结果：

```text
Kaw=0:    不退出饱和，积分器 release 时约 299 V
Kaw=100:  退出饱和约 9.355 ms，settling 约 9.555 ms
Kaw=200:  退出饱和约 5.905 ms，settling 约 6.060 ms
Kaw=400:  退出饱和约 3.255 ms，settling 约 3.425 ms
Kaw=800:  退出饱和约 1.655 ms，settling 约 1.815 ms
Kaw=1200: 退出饱和约 1.055 ms，settling 约 1.260 ms
Kaw=2000: 退出饱和约 0.605 ms，settling 约 0.810 ms
```

当前判断：

```text
Kaw=400 是合理的安全默认值，不是危险的大值。
它能明显防止积分饱和，但退出饱和速度偏保守。
如果硬件测试显示饱和退出太慢，下一步优先试 800 或 1200，再考虑 2000。
不要直接跳到 5000 以上作为默认值，除非 V1 和硬件波形都证明无抖动、无过度修正。
```

输出：

```text
results/pi_correction_gain_sweep_kp1_ki20000.csv
results/pi_correction_gain_sweep_kp1_ki20000.png
```

## v0 模型结构

```text
id_ref = 0 A
iq_ref = 0 -> 3 A step
vbus = 12 V
  -> green_joint_current_loop_input_t
  -> GreenJointCurrentLoopStep
  -> vd_cmd / vq_cmd
  -> dq average plant
  -> id_fbk / iq_fbk
```

平均 plant 目前使用锁轴/低速 dq 电气方程：

```text
did/dt = (vd - Rs * id) / Ld
diq/dt = (vq - Rs * iq) / Lq
```

这个 v0 暂时不加入交叉耦合、反电动势、机械速度和三相逆变器。它的目的不是替代全部
FOC，而是先把 `green-joint` 电流 PI 在物理量纲下闭环跑起来。

当前默认参数：

```text
Ts = 50 us
PlantTs = 5 us
StopTime = 30 ms
Vbus = 12 V
line-to-line R = 5.8 ohm
line-to-line L = 110 uH
Rs = Rll / 2 = 2.9 ohm
Ld = Lq = Lll / 2 = 55 uH
current-loop bandwidth = 800 Hz
CurDKp = CurQKp ~= 0.27646 V/A
CurDKi = CurQKi ~= 14576.99 V/(A*s)
iq step = 0 -> 1.5 A at 1 ms
```

当前验证结果：

```text
iq_ref final          = 1.5 A
iq final              = 1.5 A
iq tracking error     = 0 A
id final              = 0 A
vd final              = 0 V
vq final              = 4.35 V
voltage_mag_norm max  = 0.918469
Green-joint current-loop digital twin smoke test passed.
```

饱和退出测试：

```bash
matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_current_loop_saturation_exit_test.m')"
```

测试工况：

```text
iq_ref = 4 A -> 1.5 A at 8 ms
Vbus = 12 V
Rs = 2.9 ohm
Ld = Lq = 55 uH
```

4A 在 2.9ohm 相电阻下稳态需要约 11.6V q 轴电压，超过当前
`VoltageLimitRatio=0.577` 对应的圆限幅，因此会先进入电压饱和，再观察降回
1.5A 后的退出速度。

当前结果：

```text
pre-release iq             = 2.14883 A
pre-release vnorm max      = 1
saturation exit threshold  = 0.98
saturation exit time       = 3.3 ms after release
settling band              = +/-0.15 A
iq settling time           = 3.6 ms after release
iq peak after release      = 2.14883 A
iq min after release       = 1.37102 A
iq final                   = 1.5 A
vq final                   = 4.35 V
```

注意：当前相电气时间常数约为 `55uH / 2.9ohm = 18.97us`，已经小于
50us 控制周期。v0 平均 dq plant 因此使用 `PlantTs = 5us` 子步长仿真电机电气
动态，控制器仍保持 `Ts = 50us`。

## v1 平均电压电机模型

从 1kHz current-test 方波调试开始，数字孪生判断优先级调整为：

```text
v1 平均电压电机模型 = 主判据
v0 dq average plant = 辅助 sanity check
```

原因是 v0 只验证 PI、圆限幅和基础 R/L 电气方程，默认控制输出更直接地作用到 plant，
会低估 FOC 50us 调度、PWM/平均逆变器、采样反馈、角度反馈和电流重构带来的等效延时。
实机波形、带宽和参数整定以后应优先用 v1 对齐。

v1 使用已有平均电压电机模型主线：

```text
id_ref / iq_ref / id_fbk / iq_fbk / vbus
  -> GreenJointCurrentLoopStep
  -> vd_cmd / vq_cmd
  -> DqToAbcDutyStep
  -> phase_duty_t [0, 1]
  -> Average-Value Inverter
  -> Surface Mount PMSM
  -> MotorClarkeParkStep
  -> id_fbk / iq_fbk
```

复用来源：

```text
GreenJointCurrentLoopStep: ../green_joint_current_loop_mbd/
DqToAbcDutyStep:          ../motor_float_open_loop_mbd/
MotorClarkeParkStep:      ../motor_clarke_park_struct/
Average-Value Inverter:   Motor Control Blockset mcbplantlib
Surface Mount PMSM:       Motor Control Blockset mcblib
```

v1 有一个合并字典：

```text
green_joint_average_motor_twin_interface.sldd
```

这个字典把 green-joint 控制器接口和已有平均模型接口放到同一个模型合同里，避免多个
`.sldd` 在顶层模型里互相找不到。

当前 v1 验证结果：

```text
smoke scenario        = 0.3A short current-only sanity
iq_ref final          = 0.3 A
id final              = 0.00407041 A
iq final              = 0.292832 A
vd final              = -0.229439 V
vq final              = 1.44195 V
wm final              = 173.423 rad/s
voltage_mag_norm max  = 0.234304
Green-joint average motor twin smoke test passed.
```

注意：v1 是自由转子平均电压模型。当前 `motor.B=0` 等待辨识，所以长时间正向
`Iq` 阶跃会让电机快速加速并进入反电势/电压受限区。smoke test 只做 5ms、0.3A
低能量 sanity check；如果要严格测电流环跟踪，应使用 1kHz 方波、锁轴/负载/速度约束，
或建立专用 current-loop test harness。

### 1kHz 电流方波场景

当前已经建立一个 V1 物理模型方波场景：

```text
current_square_1khz_0p3A_average_motor_v1
```

它不是新的孤岛模型，而是在内存中把 `iq_ref_step` 替换为 `From Workspace` 方波源。
这样可以复用同一个 V1 平均电压电机模型，同时把测试场景独立成脚本和结果文件。

当前脚本在仿真结束前会恢复原 `Step` 参考源并关闭内存模型，不保存 `.slx`。

同时也建立了可视化 `.slx` harness：

```text
green_joint_average_motor_square_wave_harness.slx
```

这个模型中包含 `TestSupervisor` Stateflow Chart：

```text
CurrentSquareLow  -> after(10,tick) -> CurrentSquareHigh
CurrentSquareHigh -> after(10,tick) -> CurrentSquareLow
```

由于当前控制周期是 `50us`，`10 tick = 0.5ms`，完整周期就是 `1ms = 1kHz`。
该 harness 是测试模型，不进入生成代码。

### 延时纪律

V1 调参与实机对比时必须显式考虑延时：

```text
iq_ref[k]
  -> 50us FOC current PI
  -> dq voltage command
  -> dq-to-duty + average inverter
  -> PMSM electrical dynamics
  -> sampled phase currents
  -> Clarke/Park
  -> optional dq current filter
  -> iq_fbk[k+1]
```

因此 V1 比 V0 更接近真实闭环。当前脚本已经包含 controller 50us 与 plant 5us 的
Rate Transition；后续还需要继续补齐固件里的 dq 动态反馈滤波、scope 采样顺序和
current-test 方波输入。

## 可复用模块

优先复用：

```text
../green_joint_current_loop_mbd/
../motor_current_loop_mbd/
../motor_speed_current_loop_mbd/
../motor_float_open_loop_mbd/
../motor_control_modules/
../motor_clarke_park_struct/
../motor_current_filter_mbd/
../pwm_deadtime_compensation_mbd/
../pwm_deadtime_sampling_mbd/
```

plant 和研究模型参考：

```text
../average-inverter/
../average-inverter/switching_sampling_study/
../adc_interrupt_current_loop_test/
../identification/pmsm_electrical_parameter_identification/
../motor_performance_characterization/
```

总路线见：

```text
../docs/digital_twin_architecture_plan.md
```

## 建议开发顺序

1. 已完成：建立 `GreenJointCurrentLoopStep + dq average plant` 的最小闭环，作为 V0 辅助 sanity check。
2. 加入真实或合成 `id_ref/iq_ref/vbus` 序列输入。
3. 输出 `id/iq/vd/vq/voltage_mag_norm`，形成可比较的结果文件。
4. 接入 `motor_current_filter_mbd/`，对齐固件反馈滤波。
5. 已完成：接入 `DqToAbcDutyStep + Average Inverter + PMSM`，从 dq plant 过渡到三相 plant。
6. 已完成：接入 `MotorClarkeParkStep`，让反馈路径接近固件结构。
7. 将 V1 作为波形主判据，加入 current-test 方波、dq 动态反馈滤波、ADC/PWM 延迟、deadtime compensation、sampling window valid。
8. 最后再考虑速度环、负载扰动、齿槽补偿和硬件 adapter。

## 参数纪律

数字孪生参数必须使用物理量纲：

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

电流 PI 主线推荐从物理参数、目标带宽、相位裕度和等效延时联合计算：

```text
wc = 2 * pi * bandwidth_hz
phi_i = pi - PM - atan(wc * L / R) - wc * Td
Kp = sqrt(R^2 + (wc * L)^2) * cos(phi_i)
Ki = Kp * wc * tan(phi_i)
```

无延时 pole-cancel 公式只作为 sanity/legacy 对照：

```text
Kp = L * wc
Ki = R * wc
```

如果实测参数来自电机两根相线端子之间，按线间参数记录，并在模型入口统一换算：

```text
R_phase = R_line_to_line / 2
L_phase = L_line_to_line / 2
```

旧固件中的无量纲参数只能作为对照和反推依据，不作为数字孪生长期接口。

当前固件侧 MBD adapter 默认参数来自当前编译 variant：

```text
green-joint/Module/Config/green_joint_1615_config.json
green-joint/Module/Config/green_joint_1620_config.json
PiCorrectionGain = 400.0
VoltageLimitRatio = 0.577
VoltageModulationRatio = 0.9
```

`PiCorrectionGain=400` 已通过 `run_green_joint_pi_correction_gain_sweep.m`
做过饱和退出扫描，结论是安全但偏保守。

## 安全规则

不要在 MATLAB Desktop 打开模型时重建 `.slx/.sldd`。需要重建或 codegen 时优先用：

```bash
matlab -batch "run('<script>.m')"
```

本目录的 build 脚本已经做了保护：

```text
MATLAB Desktop + 已存在 .slx:
  只打开模型，不重建。

matlab -batch:
  允许安全重建。
```

如果你明确知道模型已关闭、并且仍想在 Desktop 中强制重建，可以先设置：

```matlab
setenv('GJ_DT_ALLOW_UNSAFE_REBUILD', '1')
run('matlab-practice/green_joint_digital_twin/build_green_joint_current_loop_twin_model.m')
```

一般不推荐这样做；优先使用 `matlab -batch`。

如果 Simulink 卡在“模型更新”，先读：

```text
../docs/simulink_hang_troubleshooting.md
```

## MIT 模式验证

当前固件 MIT 路径在 `INPUT_MODE_MIT` 下每个 50us FOC ISR 执行一次，输出直接作为电机端
`Iq_ref A` 进入 MBD 电流环。生产算法已经由 MBD 接管：

```text
foc.c::mit_control()
  -> green_joint_mit_impedance_mbd_adapter.c
  -> GreenJointMitImpedanceStep()
  -> iq_ref_a
```

对当前协议保持兼容后的等效公式仍是：

```text
Iq_ref = MIT_kp * position_error
       + MIT_kd * speed_error
       + ff_torque / (Kt * gear_ratio)
```

因此当前 `MIT_kp/MIT_kd` 的实际语义是：

```text
MIT_kp: A/rad
MIT_kd: A/(rad/s)
ff_torque: output-side Nm
```

不要把当前固件的 MIT `kp/kd` 直接理解为输出端 `Nm/rad`、`Nm*s/rad`。
长期物理阻抗接口已经建立在：

```text
../green_joint_mit_impedance_mbd/
```

该 MBD core 使用 `kp_nm_per_rad`、`kd_nm_s_per_rad`、`ff_torque_nm` 和
`torque_to_iq_gain_a_per_nm`，输出最小生产接口 `iq_ref_a`。固件 adapter 为了兼容旧协议，
会把 `MIT_kp/MIT_kd` 乘 `Kt_output = Kt_motor * gear_ratio` 后送入 MBD core。
不要在 digital twin 或 `foc.c` 中复制第二份 MIT 公式。

### MIT 二阶带宽设计公式

按输出端二阶系统设计：

```text
J * theta_ddot + D * theta_dot + K * theta = 0
wn = 2*pi*bandwidth_hz

K_phys = J_output * wn^2
D_phys = 2*zeta*J_output*wn - B_output
```

如果保持当前固件 MIT 电流语义，需要把输出端物理刚度/阻尼除以输出端力矩常数：

```text
Kt_output = Kt_motor * gear_ratio

MIT_kp = K_phys / Kt_output
MIT_kd = D_phys / Kt_output
```

这套公式比“给定 Kp 再补 Kd”更适合作为长期主线。给定 `Kp=12` 反推临界 `Kd` 只作为兼容当前默认值的解释，不作为正式设计入口。

MIT 主线 `.slx` 入口是 `green_joint_average_motor_twin_model.slx`：

```matlab
run('/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/setup_green_joint_current_loop_twin.m')
GJDT_ControlMode = GJDT_ControlModeMit;
open_system('/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/green_joint_average_motor_twin_model.slx')
sim('green_joint_average_motor_twin_model')
```

批量验证入口：

```bash
matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_average_motor_mit_step_test.m')"
```

旧的 `green_joint_mit_mode_1615_harness.slx` 使用一阶电流环近似 + 输出端机械模型，只作为迁移/教学参考。
不要在该 harness 上继续扩展 MIT 主线功能。

1615 脚本级批量验证入口：

```bash
matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_mit_mode_1615_test.m')"
```

当前 1615 参数：

```text
J_output = 0.00132792306138 kg*m^2
B_output = 0.0109757550501 N*m*s/rad
Tc_output = 0.028677381561 N*m
Tbias_output = -0.0020638267429 N*m
Kt_output = Kt_motor * gear_ratio = 0.948425944 N*m/A
iq_limit = variant contract default: 2 A for 1615, 4 A for 1620
```

仿真结果：

```text
MIT default Kp=12, Kd=0.1, step 0.05rad:
  overshoot ~= 16.84%
  settling ~= 65.45 ms
  max |Iq_cmd| ~= 0.6 A

MIT default Kp=12, Kd=0.1, step 0.2rad:
  overshoot ~= 21.14%
  settling ~= 101.35 ms
  max |Iq_cmd| ~= 2.4 A

MIT Kp=12, Kd~=0.24767, step 0.2rad:
  estimated damping ratio ~= 1.0
  overshoot ~= 0%
  settling ~= 122.7 ms
  max |Iq_cmd| ~= 2.4 A

MIT bandwidth=15Hz, zeta=1, step 0.2rad:
  Kp ~= 12.4369 A/rad
  Kd ~= 0.25235 A/(rad/s)
  overshoot ~= 0%
  settling ~= 116.8 ms
  max |Iq_cmd| ~= 2.49 A

MIT bandwidth=20Hz, zeta=1, step 0.2rad:
  Kp ~= 22.1100 A/rad
  Kd ~= 0.34032 A/(rad/s)
  overshoot ~= 0.04%
  settling ~= 62.4 ms
  max |Iq_cmd| ~= 4.42 A
  触发峰值电流饱和；1615 当前合同为 2A，1620 当前合同为 4A
```

结论：当前默认 `Kd=0.1` 对 1615 新辨识惯量偏欠阻尼。历史 4A 仿真下没有触发饱和，
但按当前 1615 2A 峰值电流合同，0.2rad 阶跃中的 `max |Iq_cmd| ~= 2.4A` 会触发限幅。
按 1615 当前 2A 峰值电流限制，`15Hz/zeta=1` 是更稳的主线候选；`20Hz/zeta=1`
在 0.2rad 阶跃已经会超过 1615 限幅，更适合小角度或 1620/更高限流场景的评估。
如果实机希望减少 MIT 位置阶跃超调，可以优先 A/B 测试 `15Hz/zeta=1`
对应的 `Kp≈12.44, Kd≈0.252`；但在没有真实编码器噪声和摩擦死区验证前，不直接改固件默认值。

## 当前状态

```text
资产盘点完成。
数字孪生总路线已写入 docs/digital_twin_architecture_plan.md。
本目录作为 green-joint digital twin 入口建立。
v0 平均电压电流环模型已创建并通过 smoke test。
v1 平均电压电机模型已创建并通过 smoke test；在 12V、3A 自由转子场景会触发电压饱和，
用于验证链路跑通，不作为锁轴电流环带宽结论。
```
