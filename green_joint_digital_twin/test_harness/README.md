# Green Joint Unified Test Harness

本目录用于承载 `green-joint` 数字孪生的统一测试顶层。

目标不是为每个测试新建一个完整模型，而是用一个 harness 组合：

```text
Scenario
  -> TestSupervisor / Test Sequence
  -> ControllerWrapper
  -> PlantWrapper
  -> LoggerAssessment
```

## 目标模型

长期目标：

```text
green_joint_control_test_harness.slx
```

配套脚本：

```text
build_green_joint_control_test_harness.m
open_green_joint_control_test_harness.m
run_green_joint_control_scenario.m
run_green_joint_control_smoke_tests.m
```

## TestSupervisor 职责

TestSupervisor 只负责测试状态和参考输入：

```text
Idle
CurrentSquareTest
CurrentStepTest
CurrentSaturationExitTest
SpeedStepTest
SpeedSweepTest
PositionStepTest
FaultStop
```

TestSupervisor 不允许实现控制算法。

## 推荐信号边界

TestSupervisor 输出：

```text
scenario_id
control_mode
id_ref
iq_ref_direct
wm_ref
theta_ref
iq_limit
vbus_cmd
load_torque_cmd
fault_injection_cmd
```

ControllerWrapper 输出：

```text
vd_cmd
vq_cmd
voltage_mag_norm
duty_a
duty_b
duty_c
controller_state
fault_status
```

PlantWrapper 输出：

```text
id_fbk
iq_fbk
ia
ib
ic
theta_e
wm_meas
vbus_meas
```

LoggerAssessment 输出：

```text
pass
metric_rise_time
metric_overshoot
metric_settling_time
metric_saturation_exit_time
metric_voltage_mag_norm_max
```

## 速度环测试接入纪律

速度环测试必须作为 scenario 接入，不创建独立完整 `.slx`。

推荐链路：

```text
SpeedStepTest
  -> SpeedPiStep
  -> iq_ref
  -> GreenJointCurrentLoopStep
  -> DqToAbcDutyStep
  -> PlantWrapper.average_v1
```

跨速率边界必须显式：

```text
Ts_speed -> Ts_current
Ts_current -> Ts_plant
Ts_plant -> Ts_current
Ts_plant -> Ts_speed
```

## 当前迁移来源

以下文件是迁移来源，不是新增测试模板：

```text
../green_joint_average_motor_square_wave_harness.slx
../build_green_joint_average_motor_square_wave_harness.m
../run_green_joint_average_motor_square_wave_harness_test.m
```
