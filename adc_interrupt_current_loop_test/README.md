# ADC Interrupt Current Loop Test

标签：`[RESEARCH]`。这是电流环触发时序的专项验证目录，不是新的共享控制模块模板；其中 `grt` 和触发式实现只服务本专题分析。

这个小测试用于比较两种电流环触发方式：

```text
without ADC interrupt:
  固定周期运行电流环，采样后立即更新电压输出

with ADC interrupt:
  PWM 周期中点触发虚拟 ADC
  ADC 转换完成后触发电流环
  电流环结果写入 PWM shadow
  下一个 PWM 周期开始时生效
```

测试对象是最小 RL 电流模型：

```text
L di/dt + R i = v
```

电流控制器是 PI：

```text
v_cmd = Kp(i_ref - i_meas) + integral
```

## Run

From repository root:

```matlab
run('adc_interrupt_current_loop_test/matlab/run_with_without_adc_interrupt.m')
```

Outputs are saved to:

```text
adc_interrupt_current_loop_test/results/
```

## Simulink Trigger Model

Generate the visual Simulink model:

```matlab
run('adc_interrupt_current_loop_test/simulink/build_adc_interrupt_current_loop_triggered_model.m')
open_system('adc_interrupt_current_loop_triggered')
```

Run the Simulink test and save data/plots:

```matlab
run('adc_interrupt_current_loop_test/simulink/run_triggered_simulink_test.m')
```

Generated model:

```text
adc_interrupt_current_loop_test/simulink/adc_interrupt_current_loop_triggered.slx
```

The trigger source is fixed-period:

```text
Function-Call Generator sample_time = 50e-6
```

The model contains two paths:

```text
Fixed_50us_Timer_INT -> CurrentLoop_Timer_ISR -> Plant_without_ADC_delay

Fixed_50us_HW_INT -> CurrentLoop_ADC_ISR -> PWM_shadow_load
                 ^                             |
                 |                             v
          ADC_sample_hold <- Plant_with_ADC_delay
```

This mirrors the MCU structure where a fixed hardware interrupt triggers the
current-loop module. In this simplified version the interrupt is exactly
periodic at 50 us; it does not use a `[period offset]` sample-time vector.

## Expected Difference

MATLAB script 版本仍然用于比较真实 ADC/PWM 时序延迟：

```text
PWM event -> ADC sample -> ADC EOC -> current loop -> next PWM update
```

Simulink trigger model 版本现在使用固定 50 us 中断：

```text
Fixed 50 us HW_INT -> current loop
```

因此它主要用于验证“固定周期中断触发控制模块”的结构。因为没有再加入
`[period offset]` 和额外 PWM shadow 延迟，两条 Simulink 路径的响应会非常接近。
