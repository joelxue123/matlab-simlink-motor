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
J_speed_loop = J_output_equivalent / gear_ratio。
B_speed_loop = B_output_equivalent / gear_ratio。
```

基于当前 1615 输出端机械辨识的速度环主线参数：

```text
J_output_equivalent = 0.00132792306138 kg*m^2
B_output_equivalent = 0.0109757550501 N*m*s/rad
J_speed_loop = 7.24255828e-6 kg*m^2
B_speed_loop = 5.98623128e-5 N*m*s/rad
Kt_default = 2.56e-3 / 0.4949 = 0.00517276217 N*m/A
Ts_speed = 100 us
bandwidth_start = 20 Hz
Kp_speed = 0.340319362 A/(rad/s)
Ki_speed = 22.1100241 A/rad
Kaw_speed = 125.663706 1/s
iq_limit_initial = 0.1 A
```

如果后续重新辨识输出端机械参数，必须先更新 variant contract，再由同步脚本更新
SpeedPiStep 和 digital twin：

```text
J_speed_loop = J_output_equivalent / gear_ratio
B_speed_loop = B_output_equivalent / gear_ratio
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

### mit_current

```text
theta_ref / theta_meas / vel_ref / vel_meas / ff_torque
  -> GreenJointMitImpedanceStep
  -> motor-side iq_ref
  -> GreenJointCurrentLoopStep
  -> DqToAbcDutyStep
```

复用模块：

```text
../../green_joint_mit_impedance_mbd/
../../green_joint_current_loop_mbd/
../../motor_float_open_loop_mbd/
```

MIT 接口纪律：

```text
GreenJointMitImpedanceStep 使用输出端物理阻抗：
  kp_nm_per_rad, kd_nm_s_per_rad, ff_torque_nm
输出是电机端 iq_ref_a。
torque_to_iq_gain_a_per_nm = 1 / (Kt_motor * gear_ratio)，由 variant/adapter 预计算后传入。
当前固件协议 MIT_kp/MIT_kd 仍是 A/rad 和 A/(rad/s)，adapter 已通过
Kt_output = Kt_motor * gear_ratio 把旧电流域增益转换成物理域 MBD 输入。
不要在 wrapper、scenario 或 firmware 中复制第二份 MIT 公式。
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
