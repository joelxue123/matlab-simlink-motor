# Green Joint Scenario Catalog

本目录登记 `green-joint` 数字孪生的测试场景。

场景不是一个完整 `.slx`，而是一组可被统一 test harness 读取和执行的测试定义。
新增速度环、位置环、电流环测试时，先登记到这里，再接入
`../test_harness/`。

## 场景规则

每个场景必须记录：

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

不允许在场景里重新实现控制器或 plant。

## 已有场景

### speed_estimator_pll_interval_sweep

用途：

```text
验证 ODrive-style SpeedEstimatorPllStep 在 50/100/200/400us 更新周期下的收敛速度、稳态误差和速度测量相位。
为已经移除的 firmware 400us diff + alpha=0.1 IIR 链路保留历史 A/B 基准。
```

当前入口：

```text
../run_green_joint_speed_estimator_pll_interval_sweep.m
../../motor_speed_estimator_mbd/run_speed_estimator_pll_smoke_test.m
```

长期归属：

```text
TestSupervisor.SpeedEstimatorValidation
GreenJointControllerWrapper.speed_current measurement path
PlantWrapper.average_v1 angle feedback
```

参数：

```text
motor speed step = 0 -> 900 rad/s
sample_time = 50 / 100 / 200 / 400 us
pll_bandwidth = 120 / 240 / 360 / 480 / 600 Hz
gear_ratio = 183.35
encoder_counts = 65536
```

必须记录：

```text
rise_time
settling_time
overshoot
steady_speed_noise
phase_at_speed_loop_bandwidth
final_motor_speed_error
```

### speed_estimator_pll_noise_sweep

用途：

```text
验证 ODrive-style SpeedEstimatorPllStep 对编码器 count 级角度噪声的速度噪声放大。
硬件反馈显示 600 Hz PLL 噪声偏大，因此默认值下调到 360 Hz。
```

当前入口：

```text
../run_green_joint_speed_estimator_pll_noise_sweep.m
```

必须记录：

```text
pll_bandwidth_hz
noise_std_count
motor_speed_noise_std_rad_s
joint_speed_noise_std_rad_s
noise_ratio_vs_600hz
```

### speed_estimator_wrap_contract

用途：

```text
单元级验证 SpeedEstimator 的角度约定：
theta_meas/theta_pred/theta_hat 使用 [0, 2*pi)
theta_err 使用 [-pi, pi)
跨 0/2pi 边界时速度估算不产生尖峰。
```

注意：

```text
这不是完整 digital twin 闭环验证。
完整验证必须运行 speed_step_0_to_4radps_joint_average_motor_v1，
让 SpeedEstimatorPllStep 接入 SpeedPiStep、GreenJointCurrentLoopStep
和 Average-Value Inverter + PMSM plant。
```

当前入口：

```text
../run_green_joint_speed_estimator_wrap_contract_test.m
../../motor_speed_estimator_mbd/run_speed_estimator_pll_smoke_test.m
```

长期归属：

```text
TestSupervisor.SpeedEstimatorValidation
GreenJointControllerWrapper.speed_current measurement path
PlantWrapper.average_v1 angle feedback
```

必须记录：

```text
static wrap boundary cases
positive 0 -> 2pi crossing
negative 2pi -> 0 crossing
theta_hat range
theta_err range
max speed-estimator error at wrap
```

### current_square_1khz_0p3A_average_motor_v1

用途：

```text
电流环小信号跟踪测试。
对齐硬件 1 kHz iq 方波波形。
```

当前入口：

```text
../run_green_joint_average_motor_square_wave_test.m
../run_green_joint_average_motor_square_wave_harness_test.m
```

长期归属：

```text
TestSupervisor.CurrentSquareTest
GreenJointControllerWrapper.current_only
PlantWrapper.average_v1
```

参数：

```text
iq_ref = +/-0.3 A
period = 1 ms
Ts_current = 50 us
Ts_plant = 5 us
Kp/Ki = 1.0 / 20000.0
Vbus = 12 V
```

### mit_impedance_1615_step

用途：

```text
验证 MIT MBD core 在 1615 输出端机械参数下，通过完整 V1 平均电压物理链的位置阶跃响应。
MIT 输出必须进入 GreenJointCurrentLoopStep，不再使用一阶电流环近似作为主线结论。
```

当前入口：

```text
../green_joint_average_motor_twin_model.slx
../run_green_joint_average_motor_mit_step_test.m
```

长期归属：

```text
TestSupervisor.MitImpedanceStep
GreenJointControllerWrapper.mit_current
PlantWrapper.average_v1
```

参数：

```text
pos_target = 0.2 rad
Ts_mit = 50 us
MIT design = 15 Hz, zeta = 1, converted to current-domain protocol gains
iq_limit = variant contract default: 2 A for 1615, 4 A for 1620
plant = DqToAbcDutyStep + Average-Value Inverter + Surface Mount PMSM + PLL feedback
```

必须记录：

```text
overshoot
settling_time
iq_ref_abs_max
iq_saturation
final_position_error
```

## 计划场景

### current_saturation_exit_4A_to_1p5A

用途：

```text
验证电流环电压饱和和 back-calculation anti-windup 退出速度。
```

长期归属：

```text
TestSupervisor.CurrentSaturationExitTest
GreenJointControllerWrapper.current_only
PlantWrapper.average_v1
```

必须记录：

```text
iq_ref_high = 4 A
iq_ref_low = 1.5 A
transition_time
voltage_mag_norm
saturation_exit_time
settling_time
PiCorrectionGain
```

### speed_step_0_to_4radps_joint_average_motor_v1

用途：

```text
验证速度环 step 响应、iq_ref 限幅、电流内环配合和机械速度跟踪。
```

长期归属：

```text
TestSupervisor.SpeedStepTest
GreenJointControllerWrapper.speed_current
PlantWrapper.average_v1
```

推荐连接：

```text
joint_speed_ref_rad_s
  -> SpeedPiStep
  -> motor-side iq_ref
  -> Rate Transition, Ts_speed to Ts_current
  -> GreenJointCurrentLoopStep
  -> DqToAbcDutyStep
  -> Average-Value Inverter + PMSM
  -> motor_speed / gear_ratio
  -> joint_speed feedback
```

必须记录：

```text
joint_speed_ref = 0 -> 4 rad/s
Ts_speed = 100 us
Ts_current = 50 us
Ts_plant = 5 us or chosen plant step
iq_limit = A
Kp_speed / Ki_speed / Kaw_speed
Kp_current / Ki_current / Kaw_current
rise_time
overshoot
settling_time
final_speed_error
iq_ref_max
voltage_mag_norm_max
```

基于当前 1615 输出端机械辨识的主线设计：

```text
gear_ratio = 183.35
J_output_equivalent = 0.00132792306138 kg*m^2
B_output_equivalent = 0.0109757550501 N*m*s/rad
J_speed_loop = J_output_equivalent / gear_ratio = 7.24255828e-6 kg*m^2
B_speed_loop = B_output_equivalent / gear_ratio = 5.98623128e-5 N*m*s/rad
Kt_rated = 2.56e-3 / 0.4949 = 0.00517276217 N*m/A
Kt_peak = 7.33e-3 / 1.4847 = 0.00493702431 N*m/A
Kt_default = Kt_rated
selected bring-up speed bandwidth = 20 Hz
production candidate bandwidth = 40 Hz, after hardware validation
```

20 Hz 安全启动参数：

```text
Kp_speed = 0.340319362 A/(rad/s)
Ki_speed = 22.1100241 A/rad
Kaw_speed = 125.663706 1/s
iq_limit_initial = 0.1 A
```

40 Hz 候选参数：

```text
Kp_speed = 0.692211324 A/(rad/s)
Ki_speed = 88.4400964 A/rad
Kaw_speed = 251.327412 1/s
```

设计脚本：

```text
../design_green_joint_speed_loop.m
```

结果表：

```text
../results/green_joint_speed_loop_design_j0p034kgmm2.csv
```

同步脚本：

```text
../sync_green_joint_speed_loop_twin_parameters.m
```

主线验证脚本：

```text
../run_green_joint_average_motor_speed_step_test.m
```

当前 V1 结果：

```text
final joint speed      = 4 rad/s
final speed error      = 0 rad/s
rise time to 90%       = 6 ms
settling time          = 32 ms
overshoot              = 16.235 %
|iq_ref| max           = 1.2496 A
|iq| max               = 1.22198 A
voltage_mag_norm max   = 0.668392
```

注意：

```text
SpeedPiStep 输入速度是输出端 joint-side rad/s，输出是电机端 Iq A。
速度环等效惯量使用 J_output_equivalent / gear_ratio。
motor.B 来自输出端辨识阻尼折算；不要继承旧 average-inverter 示例的 1e-4 阻尼。
```

### speed_high_speed_voltage_limit_average_motor_v1

用途：

```text
复现最高速/保护区域中 SPEED_IQ_REF_A 很高但 MOTOR_IQ 很小的现象。
验证速度环积分退饱和慢是否来自电压/反电势限制，而不是单纯 iq_limit clamp。
```

当前入口：

```text
../run_green_joint_speed_loop_v1_high_speed_voltage_limit_test.m
```

长期归属：

```text
TestSupervisor.SpeedHighSpeedVoltageLimitTest
GreenJointControllerWrapper.speed_current
PlantWrapper.average_v1
```

参数：

```text
Vbus = 12 V
voltage_limit = 12 * 0.577 * 0.9 = 6.2316 V
Ts_speed = 100 us
Ts_current = 50 us
Ts_plant = 5 us
iq_limit = 4 A
initial speed integrator = 4 A
speed feedback = 0.98 * no-load speed at voltage_limit
speed error = -3 rad/s
```

当前 V1 诊断结论：

```text
baseline_no_iq_tracking_aw:
  time_to_iq_ref_below_0p5_s = 10.712 s
  iq_actual_mean_a = 0.04292 A
  voltage_norm_mean = 1.0

naive iq tracking anti-windup:
  time_to_iq_ref_below_0p5_s = 0.017 s
  iq_ref_final_a = -3.6239 A
```

注意：

```text
1. 该场景证明 V1 平均电压模型可以复现实测“实际电流很小”的高速保护现象。
2. 直接用 iq_meas - iq_ref 做 tracking anti-windup 过于激进，只能作为反例。
3. 生产方案应在速度环 MBD v2 中增加带电压饱和判据的 tracking anti-windup。
```

结果文件：

```text
../results/green_joint_speed_loop_v1_high_speed_voltage_limit.csv
../results/green_joint_speed_loop_v1_high_speed_voltage_limit_summary.csv
../results/green_joint_speed_loop_v1_high_speed_voltage_limit.png
```

### speed_sweep_low_frequency_average_motor_v1

用途：

```text
估算速度闭环低频带宽和相位滞后。
```

长期归属：

```text
TestSupervisor.SpeedSweepTest
GreenJointControllerWrapper.speed_current
PlantWrapper.average_v1
```

注意：

```text
速度环扫频频率应远低于电流环 1 kHz 测试频率。
先从 0.5 Hz、1 Hz、2 Hz、5 Hz 等低频开始。
```
