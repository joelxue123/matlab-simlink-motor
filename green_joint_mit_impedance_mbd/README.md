# Green Joint MIT Impedance MBD

本目录是 `green-joint` MIT 阻抗控制的 MBD core。它只实现 MIT 模式里从
输出端位置/速度误差和前馈力矩生成电机端 `iq_ref` 的算法，不包含电流环、速度环、
PWM、ADC 或状态机。

## 主线定位

```text
protocol MIT setpoint
  -> GreenJointMitImpedanceStep
  -> motor-side iq_ref A
  -> GreenJointCurrentLoopStep
  -> PlantWrapper.average_v1 / firmware FOC
```

本模块是 `green_joint_digital_twin/FINAL_MAINLINE_ARCHITECTURE.md` 里的
`MIT impedance` 主线模块。固件已经通过
`green-joint/Module/Src/green_joint_mit_impedance_mbd_adapter.c` 接入本模块。不要在
digital twin 或 `foc.c` 里复制一份 MIT 公式。

固件 `INPUT_MODE_MIT` 没有 `ISR_Counter % 2` 分频，MIT 与 FOC/current loop 同频运行：

```text
Ts_mit = Ts_current = 50 us
frequency = 20 kHz
```

`build_green_joint_mit_impedance_model.m` 的根模型固定步长和 smoke harness 均应保持 50us。
不要把速度环的 100us 周期套到 MIT core 上。

## 物理接口

输入 bus：`green_joint_mit_input_t`

```text
pos_target_rad                 output-side position target, rad
pos_feedback_rad               output-side position feedback, rad
vel_target_rad_s               output-side velocity target, rad/s
vel_feedback_rad_s             output-side velocity feedback, rad/s
ff_torque_nm                   output-side feedforward torque, N*m
kp_nm_per_rad                  physical stiffness, N*m/rad
kd_nm_s_per_rad                physical damping, N*m*s/rad
torque_to_iq_gain_a_per_nm     1 / (Kt_motor * gear_ratio), A/N*m
iq_limit_a                     motor-side current limit, A
```

输出 bus：`green_joint_mit_output_t`

```text
iq_ref_a                       motor-side q-axis current command, A
```

第一版生产接口刻意只输出 `iq_ref_a`，与 `SpeedPiStep` 的最小 bus 风格保持一致。
`position_error`、`torque_cmd`、`saturated` 等诊断量先留在 smoke test / digital twin
里计算对照，后续确认固件和上位机确实需要后再扩展 ABI。

## 公式

```text
position_error = wrap_pi(pos_target - pos_feedback)
speed_error    = vel_target - vel_feedback
torque_cmd     = kp * position_error + kd * speed_error + ff_torque
iq_unsat       = torque_cmd * torque_to_iq_gain
iq_ref         = clamp(iq_unsat, -abs(iq_limit), abs(iq_limit))
```

角度 wrap 不使用 `fmod`。由于协议位置在 `[-pi, pi]`，原始误差最多在
`[-2*pi, 2*pi]`，所以一次 `> pi` 减 `2*pi`、一次 `< -pi` 加 `2*pi` 足够。

## 1615 当前候选

基于输出端二阶系统：

```text
wn = 2*pi*bandwidth_hz
K_phys = J_output * wn^2
D_phys = 2*zeta*J_output*wn - B_output
```

1615 当前辨识参数：

```text
J_output = 0.00132792306138 kg*m^2
B_output = 0.0109757550501 N*m*s/rad
Kt_output = 0.00517276217 * 183.35 = 0.9484259438695 N*m/A
torque_to_iq_gain = 1.05437858007 A/N*m
```

推荐 bring-up 候选：

```text
15 Hz, zeta = 1:
  kp_nm_per_rad   = 11.7954678
  kd_nm_s_per_rad = 0.239331845
```

当前固件旧协议 `MIT_kp/MIT_kd` 是电流域增益 `A/rad` 和 `A/(rad/s)`。
长期主线使用本模块的物理域 `N*m/rad` 和 `N*m*s/rad`。如果短期仍要兼容旧协议，
应在 adapter 层明确转换，不要污染 MBD core 的物理合同。

## 使用

构建模型：

```bash
matlab -batch "run('matlab-practice/green_joint_mit_impedance_mbd/build_green_joint_mit_impedance_model.m')"
```

运行 smoke test：

```bash
matlab -batch "run('matlab-practice/green_joint_mit_impedance_mbd/run_green_joint_mit_impedance_smoke_test.m')"
```

生成 C 代码：

```bash
matlab -batch "run('matlab-practice/green_joint_mit_impedance_mbd/generate_green_joint_mit_impedance_code.m')"
```

当前固件接入位置：

```text
green-joint/Module/MBD/green_joint_mit_impedance
green_joint_mit_impedance_mbd_adapter.c/.h
foc.c::mit_control()
```

同步到固件后必须比对 `.c/.h`：

```bash
for f in matlab-practice/green_joint_mit_impedance_mbd/green_joint_mit_impedance_model_ert_rtw/*.[ch]; do \
  base=$(basename "$f"); \
  cmp -s "$f" "green-joint/Module/MBD/green_joint_mit_impedance/$base" || echo "$base differs"; \
done
```

当前固件协议 `MIT_kp/MIT_kd` 暂时保持旧电流域语义。adapter 会乘
`Kt_output = Kt_motor * gear_ratio` 转成 `kp_nm_per_rad/kd_nm_s_per_rad`，所以不要把
旧协议字段直接改成物理域，除非同步升级上位机协议和文档。

## 实现注意

本模块的 build 脚本应持续对齐 `motor_speed_pi_mbd/build_speed_pi_model.m` 的骨架：

```text
add_mit_subsystem / add_root_interface 分层
Subsystem 清空逻辑必须删除默认 In1/Out1 和默认连线
Bus Creator 使用 UseBusObject + BusObject + NonVirtualBus
Outport 使用 OutDataTypeStr = Bus: <bus_name>
```

不要用临时的 `delete_block_if_exists()` 或只删除默认线的写法。那会残留默认 `Out1`，
导致外部看到 `GreenJointMitImpedanceStep/1` 是 nonbus，而不是 `mit_output_bus_creator`
输出的非虚拟 bus。这个坑已经在本模块 bring-up 时踩过一次。
