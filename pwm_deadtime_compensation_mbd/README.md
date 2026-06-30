# PWM Deadtime Compensation MBD

这个模块把 50us 周期的死区补偿策略做成用户级 MBD 算法模块，可生成嵌入式 C。

## 算法

输入：

```text
pwm_deadtime_comp_input_t {
  da
  db
  dc
  id
  iq
  sin_theta_e
  cos_theta_e
}
```

输出：

```text
pwm_deadtime_comp_output_t {
  da
  db
  dc
  comp_a
  comp_b
  comp_c
  active_a
  active_b
  active_c
}
```

公式：

```text
ia_synth = id*cos(theta_e) - iq*sin(theta_e)
ib_synth = -0.5*ia_synth + sqrt(3)/2*(id*sin(theta_e) + iq*cos(theta_e))
ic_synth = -0.5*ia_synth - sqrt(3)/2*(id*sin(theta_e) + iq*cos(theta_e))

gain_x   = clamp((abs(i_x) - current_zero) / (current_full - current_zero), 0, 1)
active_x = enable && gain_x > 0
sign_x   = +1 when i_x > 0, else -1
comp_x   = polarity * sign_x * comp_duty * gain_x when active_x else 0
d_x_out  = clamp(d_x + comp_x, 0, 1)
```

其中 `i_x` 使用模块内部由 `id/iq/sin(theta_e)/cos(theta_e)` 合成出来的三相电流，
不直接使用 ADC 相电流极性。这样在小电流和 ADC 零漂/噪声附近更稳定。

含义：

```text
abs(i) <= current_zero:
  小电流区，不硬判极性，不补偿。

current_zero < abs(i) < current_full:
  过渡区，按电流幅值线性放大补偿。

abs(i) >= current_full:
  大电流区，使用电流极性满幅补偿。
```

当前默认：

```text
sample time = 50 us
comp_duty = 0.01000
current_zero = 0.02 A
current_full = 0.10 A
current_inv_range = 12.5 1/A
polarity = -1
enable = true
```

`polarity=-1` 是根据当前 SPS `Universal Bridge + PMSM` 电流正方向校准的结果。

## 修改入口

客户或同事要修改接口类型、Bus 名、默认补偿参数时，优先修改：

```text
build_pwm_deadtime_compensation_model.m
customer_interface_config()
default_deadtime_compensation_params()
```

然后重新生成 `.sldd/.slx/C`。不要手工修改生成的 `.c/.h`。

当前 `DeadtimeCompDuty`、`DeadtimeCompCurrentZero_A`、`DeadtimeCompCurrentFull_A`、
`DeadtimeCompCurrentInvRange_1perA`、`DeadtimeCompPolarity`
是编译期默认参数。后续如果要台架在线标定，应把这些 `Simulink.Parameter`
升级为 `ExportedGlobal` 或参数结构。

为了避免生成代码里出现运行时除法，当前模型使用：

```text
DeadtimeCompCurrentInvRange_1perA = 1 / (DeadtimeCompCurrentFull_A - DeadtimeCompCurrentZero_A)
```

如果手工修改 `current_zero/current_full`，需要同步更新这个倒数参数。

## 构建

```bash
matlab -batch "run('pwm_deadtime_compensation_mbd/build_pwm_deadtime_compensation_model.m')"
```

## 测试

```bash
matlab -batch "run('pwm_deadtime_compensation_mbd/run_pwm_deadtime_compensation_test.m')"
```

当前测试点：

```text
input duty = [0.05 0.95 0.50]
input dq = [id=0.0 iq=0.2] A, theta_e = 0 rad
synth current = [0.0 0.1732 -0.1732] A
output duty = [0.05 0.94 0.51]
comp = [0.00 -0.01 0.01]
active = [0 1 1]
```

## 生成 C

```bash
matlab -batch "run('pwm_deadtime_compensation_mbd/generate_pwm_deadtime_compensation_code.m')"
```

期望接口：

```c
extern void DeadtimeCompensationStep(const pwm_deadtime_comp_input_t *rtu_comp_in,
  pwm_deadtime_comp_output_t *rty_comp_out);
```

## 和开关级仿真的关系

这个模块是用户级补偿算法 core，符合 MBD/codegen 规范。

开关级验证仍在：

```text
average-inverter/switching_sampling_study/run_switching_deadtime_motor_smoke_test.m
```

验证 harness 使用开关 MOS + PMSM plant 观察补偿效果；本目录负责沉淀可交付算法接口。
当前可交付接口使用 `id/iq/sin_theta_e/cos_theta_e` 合成相电流极性。

当前开关级 harness 也已经按同一原则更新：它插入团队总库里的
`motor_control_lib.slx/DeadtimeCompensationStep` 复用模块。
开关级模型只负责把 duty、`id/iq/theta_e` 转成 `pwm_deadtime_comp_input_t`
输入 bus，并把输出 bus 转回 plant/PWM 需要的 double 信号。plant 的真实
`ia/ib/ic` 只用于物理波形观察、误差分析和极性校准，不作为最终固件接口。

本目录仍保留 `pwm_deadtime_compensation_lib.slx` 作为模块包内的独立库产物；
团队对外复用入口统一走：

```text
motor_control_modules/motor_control_lib.slx/DeadtimeCompensationStep
```

最新开关级验证：

```text
deadtime_comp_current_source = dq_synthesized
deadtime_comp_id_A = 0
deadtime_comp_iq_A = 0.2
ia/ib/ic pk-pk = [0.2731 1.1862 1.1045] A
deadtime_comp_range = a[-0 9.04e-06], b[-0.01 -0.01], c[0.01 0.01]
Result: PASS
```

边界总结：

```text
DeadtimeCompensationStep = 可交付死区补偿算法
DeadtimeSamplingWindowStep = 采样窗口 valid 判定
Universal Bridge + PMSM = 开关级物理验证台
```
