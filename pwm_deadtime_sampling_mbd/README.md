# PWM Deadtime Sampling MBD

这个模块把 `average-inverter/switching_sampling_study/` 里关于死区和采样窗口的经验，整理成一个可复用、可生成 C 代码的 MBD 模块。

注意：本模块不是开关级死区物理仿真。它只计算采样窗口是否有效。真正观察死区对三相电流的影响，需要使用 `average-inverter/switching_sampling_study/run_switching_deadtime_motor_smoke_test.m` 这类开关 MOS + 电机 plant 仿真。

## 目标

输入三相 duty：

```text
pwm_phase_duty_t {
  da
  db
  dc
}
```

输出每相低边采样窗口和有效标志：

```text
pwm_sampling_status_t {
  ideal_low_a_s
  ideal_low_b_s
  ideal_low_c_s
  usable_low_a_s
  usable_low_b_s
  usable_low_c_s
  sample_valid_a
  sample_valid_b
  sample_valid_c
  all_samples_valid
  min_usable_low_s
}
```

核心逻辑：

```text
T_low_ideal = max((1 - duty) * T_pwm, 0)
T_usable    = max(T_low_ideal - 2 * dead_time - adc_settle_time, 0)
valid       = T_usable >= min_valid_window
```

## 当前默认测试条件

```text
PWM frequency       = 20 kHz
T_pwm               = 50 us
dead_time           = 500 ns
dead_time_total     = 1 us
adc_settle_time     = 1 us
min_valid_window    = 2.5 us
duty                = [0.05, 0.95, 0.50]
```

对应结论：

```text
A phase usable window = 45.5 us -> valid
B phase usable window = 0.5 us  -> invalid
C phase usable window = 23.0 us -> valid
all_samples_valid     = false
```

这正好对应高占空比下某一相低边采样窗口被死区和采样建立时间压缩的问题。

## 构建模型

```bash
matlab -batch "run('pwm_deadtime_sampling_mbd/build_pwm_deadtime_sampling_model.m')"
```

生成：

```text
pwm_deadtime_sampling_mbd/pwm_deadtime_sampling_model.slx
pwm_deadtime_sampling_mbd/pwm_deadtime_sampling_interface.sldd
```

## 功能测试

```bash
matlab -batch "run('pwm_deadtime_sampling_mbd/run_pwm_deadtime_sampling_window_test.m')"
```

测试会检查：

```text
usable_low_a_s ~= 45.5 us
usable_low_b_s ~= 0.5 us
usable_low_c_s ~= 23.0 us
sample_valid_a = true
sample_valid_b = false
sample_valid_c = true
all_samples_valid = false
```

## 生成 C 代码

```bash
matlab -batch "run('pwm_deadtime_sampling_mbd/generate_pwm_deadtime_sampling_code.m')"
```

期望生成的可复用函数位于：

```text
pwm_deadtime_sampling_mbd/pwm_deadtime_sampling_model_ert_rtw/
```

重点检查：

```text
DeadtimeSamplingWindowStep.h
pwm_deadtime_sampling_types.h
```

当前已验证生成接口：

```c
extern void DeadtimeSamplingWindowStep(const pwm_phase_duty_t *rtu_duty_in,
  pwm_sampling_status_t *rty_status_out);
```

类型头文件中生成：

```c
typedef struct {
  T_PwmDuty da;
  T_PwmDuty db;
  T_PwmDuty dc;
} pwm_phase_duty_t;

typedef struct {
  T_PwmTime ideal_low_a_s;
  T_PwmTime ideal_low_b_s;
  T_PwmTime ideal_low_c_s;
  T_PwmTime usable_low_a_s;
  T_PwmTime usable_low_b_s;
  T_PwmTime usable_low_c_s;
  T_PwmSampleValid sample_valid_a;
  T_PwmSampleValid sample_valid_b;
  T_PwmSampleValid sample_valid_c;
  T_PwmSampleValid all_samples_valid;
  T_PwmTime min_usable_low_s;
} pwm_sampling_status_t;
```

## 复现脚本基线

复现命令使用了旧研究目录：

```bash
matlab -batch "cd('average-inverter/switching_sampling_study'); set(0,'DefaultFigureVisible','off'); rdt = plot_phase_a_deadtime_comparison('deadTime',500e-9,'modulationRatio',0.9); rmcu = run_mcu_sampling_window_study('modulationRatio',0.9,'dutySampleLimit',0.95); rrl = run_rl_sampling_impact_study('modulationRatio',0.9,'Rohm',4,'Lh',100e-6,'periods',20,'samplesPerPwm',2000);"
```

结果摘要：

```text
deadtime_us = 0.500
duty_a      = 0.050000
mcu_min_window_base_us = 25.006252
mcu_min_window_shifted_us = 25.006252
rl_sample_error_rms_base_A = 0.302540
rl_sample_error_rms_shifted_A = 0.302540
rl_ripple_pkpk_base_A = 3.348031
rl_ripple_pkpk_shifted_A = 3.348031
```

这里的 RL 复现脚本主要用于观察 PWM 纹波和采样点相对周期平均值的误差；新 MBD 模块聚焦嵌入式可交付的窗口 valid 判断。

## 工程边界

这个模块只负责采样窗口判定，不负责：

- 生成 SVPWM duty。
- 做 ADC 原始值到电流的转换。
- 做电流重构。
- 做 deadtime 电压补偿。
- 判断实际运放是否饱和。

后续更完整的采样链路应该是：

```text
phase_duty_t
  -> DeadtimeSamplingWindowStep
  -> current_valid flags
  -> ADC sampling adapter / current reconstruction
  -> Clarke/Park / current PI / observer
```
