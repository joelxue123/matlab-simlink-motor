# Green Joint Controller Wrapper

本目录用于定义 `green-joint` 数字孪生里的控制器组合边界。

ControllerWrapper 只组合基础 MBD 模块，不重新实现算法。

## 目标组合

### current_only

```text
id_ref / iq_ref / id_fbk / iq_fbk / vbus
  -> GreenJointCurrentLoopStep
  -> vd_cmd / vq_cmd
  -> DqToAbcDutyStep
```

复用模块：

```text
../../green_joint_current_loop_mbd/
../../motor_float_open_loop_mbd/
```

### speed_current

```text
motor_angle_rad
  -> SpeedEstimatorPllStep
  -> joint_speed_meas

joint_speed_ref / joint_speed_meas / iq_limit
  -> SpeedPiStep
  -> motor-side iq_ref
  -> GreenJointCurrentLoopStep
  -> DqToAbcDutyStep
```

复用模块：

```text
../../motor_speed_estimator_mbd/
../../motor_speed_pi_mbd/
../../green_joint_current_loop_mbd/
../../motor_float_open_loop_mbd/
```

速度环单位约定：

```text
SpeedEstimatorPllStep 的角度输入使用电机端机械角 rad。
SpeedEstimatorPllStep 输出 joint_speed_est_rad_s 供 SpeedPiStep 使用。
SpeedPiStep 的速度输入使用减速器输出端 rad/s。
SpeedPiStep 的电流输出使用电机端 q 轴电流 A。
gear_ratio = motor_speed / joint_speed = 183.35。
J_speed_loop = J_motor * gear_ratio + J_load_output / gear_ratio。
```

基于当前 1615 + 减速比的速度环初始参数：

```text
J_motor = 0.034 kg*mm^2 = 3.4e-8 kg*m^2
J_speed_loop = 6.2339e-6 kg*m^2
Kt_default = 2.56e-3 / 0.4949 = 0.00517276217 N*m/A
Ts_speed = 100 us
bandwidth_start = 20 Hz
Kp_speed = 0.302884591 A/(rad/s)
Ki_speed = 19.0308001 A/rad
Kaw_speed = 125.663706 1/s
iq_limit_initial = 0.1 A
```

如果后续识别出输出端负载惯量，速度环 `Kp_speed` 和 `Ki_speed` 按
`J_speed_loop` 线性缩放：

```text
J_speed_loop_new = J_motor * gear_ratio + J_load_output / gear_ratio
scale = J_speed_loop_new / 6.2339e-6
Kp_speed_new = Kp_speed * scale
Ki_speed_new = Ki_speed * scale
```

参数同步规则：

```text
先运行 ../design_green_joint_speed_loop.m 计算物理参数。
再运行 ../sync_green_joint_speed_loop_twin_parameters.m 写入 speed_pi_interface.sldd。
不要只改 base workspace，也不要在 digital twin 里复制 SpeedPiStep 参数。
```

速度估算器接入规则：

```text
不要在速度环 harness 里用 motor_speed / gear_ratio 直接绕过 estimator。
先用 SpeedEstimatorPllStep 表达测量链路，再把 joint_speed_est_rad_s 送入 SpeedPiStep。
旧 firmware diff + IIR 已从固件主线移除，只能作为离线历史 baseline，不作为新测试默认路径。
```

### position_speed_current

```text
theta_ref / theta_meas
  -> PositionLoop
  -> wm_ref
  -> SpeedPiStep
  -> iq_ref
  -> GreenJointCurrentLoopStep
  -> DqToAbcDutyStep
```

复用模块待定，必须先建立独立 `position_*_mbd/` 模块后再接入。

## 规则

```text
控制算法修改回到基础模块目录。
wrapper 不持有重复 PI 状态。
wrapper 不直接依赖 MCU HAL、ADC、PWM 寄存器。
wrapper 所有接口使用物理单位。
wrapper 所有跨速率边界显式 Rate Transition。
```

## 采样时间建议

```text
current loop: 50 us
speed loop: 100 us current mainline target
position loop: 100 us internal speed command update or按系统需求
plant: 5 us / 25 us, depending on model fidelity
```
