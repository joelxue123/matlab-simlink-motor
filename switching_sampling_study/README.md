# Switching Sampling Study

这个目录用于研究开关级 PWM、零矢量分配、低边导通窗口，以及 MCU 电流采样窗口的变化规律。

它和上层 average-inverter 控制模型的关注点不同。这里不重点研究平均电压和闭环控制，而是重点回答下面几类问题：

- 同样的电压参考下，不同 `V0/V7` 分配会怎样改变三相占空比
- 中心对齐三角载波下，每相低边导通窗口到底有多宽
- 死区、采样建立时间、MCU 额外共模偏置，会怎样压缩可采样窗口
- 如果把 PWM 结果作用到简化 RL 负载，采样点上的电流和真实电流会差多少
- 如果电角度连续旋转，哪些扇区、哪些角度最容易出现采样误差

## 目录作用

这个目录大致分成 3 类内容：

- 配置与模型脚手架
- 单 PWM 周期的时序/窗口可视化研究
- 加入 RL 负载或整周旋转后的采样误差研究

## 主要文件

### 配置和模型

- `switching_sampling_study_config.m`
	统一配置文件。定义母线电压、PWM 频率、死区、调制度、电角度、RL 参数、采样窗口假设等。

- `build_switching_sampling_study_model.m`
	自动生成 `switching_sampling_study_model.slx`。该模型是一个用于时序观察的脚手架，包含占空比生成、三角载波、PWM 比较、低边门极、窗口度量和一套 SPS 功率级验证通道。

- `run_switching_sampling_study.m`
	加载配置并调用 `build_switching_sampling_study_model.m`，然后打开模型。

### 单周期时序与零矢量分配

- `run_symmetric_v0v7_study.m`
	最简单的入口。固定 `V0/V7 = 50%/50%`，用于先建立中心对齐 PWM 和采样窗口的直观认识。

- `run_triangle_carrier_study.m`
	比较不同 `V0/V7` 分配下的占空比、上下桥臂开关波形、低边窗口长度，以及采样点的理论位置。

- `run_saddle_common_mode_study.m`
	沿一个完整电角度周期扫描，观察 saddle-wave 调制、零序分量、平均共模电压 `v_cm` 如何随 `V0/V7` 分配变化。

- `plot_phase_a_deadtime_comparison.m`
	单独观察 A 相在加入死区前后的高低桥臂门极波形差异，适合理解死区如何侵蚀有效采样窗口。

- `run_mcu_sampling_window_study.m`
	复现 MCU 侧“扇区相关的额外共模偏置/占空比钳位”逻辑，对比处理前后的占空比、低边窗口和平均共模电压。这个脚本最接近 MCU 实际采样窗口策略。

### RL 负载与采样误差

- `run_rl_sampling_impact_study.m`
	在固定电角度下，把 PWM 作用到简化三相浮地 RL 负载上，比较真实相电流、采样点电流、一个 PWM 周期内的平均电流，并评估采样误差。

- `run_rotating_rl_sampling_study.m`
	让电角度跨整个电周期连续旋转。该脚本会逐 PWM 周期更新占空比和扇区，观察不同角度位置上的采样误差分布，是研究“哪个角度最危险”的核心脚本。

## 建议的使用顺序

如果你第一次看这个目录，建议按下面顺序走，不要一开始就跑最复杂的脚本。

1. `run_symmetric_v0v7_study`
	 先理解中心对齐 PWM 和对称零矢量分配。

2. `run_triangle_carrier_study`
	 再比较 `All V0 / Symmetric / All V7` 三种分配下窗口怎么变。

3. `run_saddle_common_mode_study`
	 再从“单周期局部时序”切换到“整电角度周期的共模变化”。

4. `run_mcu_sampling_window_study`
	 然后引入 MCU 实际 duty clamp/额外共模偏置逻辑，看窗口如何被重排。

5. `run_rl_sampling_impact_study`
	 再把窗口变化映射到 RL 电流采样误差。

6. `run_rotating_rl_sampling_study`
	 最后做整电角度扫描，找到整个周期上的误差热点。

如果你想先看 Simulink 模型结构而不是脚本图像，可以先执行：

```matlab
cd switching_sampling_study
run_switching_sampling_study
```

## 快速开始

在 MATLAB 中进入目录后，可以直接运行下面这些入口。

### 1. 打开研究模型

```matlab
cd switching_sampling_study
run_switching_sampling_study
```

### 2. 仅看对称 V0/V7

```matlab
run_symmetric_v0v7_study
```

### 3. 比较不同 V0/V7 分配

```matlab
run_triangle_carrier_study
```

### 4. 看 saddle-wave 和共模电压

```matlab
run_saddle_common_mode_study
```

### 5. 看 MCU 采样窗口修正逻辑

```matlab
run_mcu_sampling_window_study
```

### 6. 看 RL 负载下的采样误差

```matlab
run_rl_sampling_impact_study
```

### 7. 看整个电角度周期的采样误差轨迹

```matlab
run_rotating_rl_sampling_study
```

## 常用可调参数

各入口脚本都支持 name/value 参数覆盖，最常用的是下面这些。

### `run_triangle_carrier_study`

- `thetaEDeg`
	电角度，单位度。
- `modulationRatio`
	调制度。
- `samplesPerPwm`
	每个 PWM 周期的离散点数。
- `splitCases`
	零矢量分配比例数组，例如 `[0 0.5 1]`。
- `caseNames`
	与 `splitCases` 对应的名称。

示例：

```matlab
run_triangle_carrier_study('thetaEDeg', 20, 'modulationRatio', 0.85)
```

### `run_saddle_common_mode_study`

- `modulationRatio`
- `numPoints`
- `splitCases`
- `caseNames`

### `run_mcu_sampling_window_study`

- `thetaEDeg`
- `modulationRatio`
- `dutySampleLimit`
- `samplesPerPwm`

### `run_rl_sampling_impact_study`

- `thetaEDeg`
- `modulationRatio`
- `dutySampleLimit`
- `samplesPerPwm`
- `periods`
- `ROhm`
- `LH`

### `run_rotating_rl_sampling_study`

- `modulationRatio`
- `dutySampleLimit`
- `samplesPerPwm`
- `ROhm`
- `LH`
- `electricalCycles`
- `electricalFreqHz`

## 输出结果怎么看

### `run_triangle_carrier_study` 输出

返回 `result` 结构体，核心字段包括：

- `result.carrier`
	当前 PWM 周期的三角载波。
- `result.case_data(i).duty`
	第 `i` 个零矢量分配方案对应的三相占空比。
- `result.case_data(i).upper_gate / lower_gate`
	PWM 比较得到的上下桥臂逻辑波形。
- `result.case_data(i).low_side_windows_s`
	每相最长低边导通窗口。
- `result.case_data(i).sample_point_s`
	每相理论采样点位置。

### `run_mcu_sampling_window_study` 输出

- `result.base`
	原始对称 SVPWM 情况。
- `result.shifted`
	应用 MCU duty clamp 后的情况。
- `result.shift_info`
	包括扇区、被采样相、额外共模偏置量、是否触发下限钳位等信息。
- `result.v_cm_base / result.v_cm_shifted`
	平均共模电压变化。

这个脚本最适合回答：

- 某个扇区下 MCU 额外偏置到底把哪两相变成“可采样相”
- 最窄采样窗口是否被拉宽
- 代价是不是引入了额外共模电压

### `run_rl_sampling_impact_study` 输出

- `result.base.sampled_phase_ripple_pkpk_A`
- `result.shifted.sampled_phase_ripple_pkpk_A`
- `result.base.sample_error_rms_A`
- `result.shifted.sample_error_rms_A`

这个脚本回答的是：窗口变化有没有真的改善采样电流误差，而不是只改善几何窗口。

### `run_rotating_rl_sampling_study` 输出

- `result.theta_period_deg`
	每个 PWM 周期对应的电角度。
- `result.base.sample_error_A / result.shifted.sample_error_A`
	三相采样误差随电角度变化。
- `result.base.period_average_A / result.shifted.period_average_A`
	每个 PWM 周期内的平均相电流。
- `result.base.sample_value_A / result.shifted.sample_value_A`
	实际采样点取到的电流值。

如果你要判断“哪些角度采样最差”，看这个脚本最直接。

## 配置脚本说明

`switching_sampling_study_config.m` 里几个最关键的参数如下：

- `ss_cfg.Vdc`
	直流母线电压。
- `ss_cfg.f_pwm`
	PWM 频率。
- `ss_cfg.dead_time_s`
	死区时间。
- `ss_cfg.modulation_ratio`
	调制度。
- `ss_cfg.theta_e_deg`
	默认电角度。
- `ss_cfg.adc_settle_time_s`
	ADC 前端建立时间，决定有效采样窗口需要再扣掉多少时间。
- `ss_cfg.duty_sample_limit`
	MCU 允许被采样相的最大占空比阈值。
- `ss_cfg.rl_load_R_ohm / rl_load_L_h`
	RL 简化负载参数。
- `ss_cfg.v0v7_splits`
	零矢量分配比例，通常 `[0.0, 0.5, 1.0]` 分别对应 `All V0 / Symmetric / All V7`。

## 当前模型包含什么

`build_switching_sampling_study_model.m` 生成的模型当前包含：

- 对称 `V0/V7 = 50/50` 占空比生成
- 中心对齐三角载波
- 三相比较器
- 互补低边门极重构
- 低边窗口度量
- `Universal Bridge + PMSM` 的一条 SPS 功率级验证通道
- `To Workspace` 日志，方便后续继续加 ADC 触发和采样判据

这个模型的意义不是替代所有脚本，而是提供一个可视化的实验台，方便你把“脚本里的理论窗口”和“Simulink 里的时序波形”对起来看。

## 窗口指标定义

当前研究中的低边窗口近似可以理解为：

```matlab
T_low_ideal = (1 - duty) * T_pwm
T_low_dead  = max(T_low_ideal - 2 * dead_time, 0)
T_valid     = max(T_low_dead - settle_time, 0)
```

这里：

- `T_low_ideal` 是不考虑死区时的低边导通时间
- `T_low_dead` 是扣掉上下沿死区后的剩余低边窗口
- `T_valid` 是再扣掉采样建立时间后的可用采样窗口

这还是时序级近似。再往下走，就应该显式加入 ADC 触发时刻、采样保持延时和电流重构逻辑。

## 适用边界

这个目录里的大多数脚本不是完整电机控制仿真，而是“为理解采样窗口和采样误差而做的控制外简化研究”。因此：

- 适合比较 PWM 与采样几何关系
- 适合评估某类 duty clamp 是否会改善采样条件
- 适合做 RL 级别的误差趋势分析
- 不适合直接替代完整 PMSM 闭环模型做控制器整定结论

## 推荐扩展方向

如果后续继续扩展，建议按下面顺序推进：

1. 显式加入 ADC 触发时间，标记每个 PWM 周期的候选采样点。
2. 把“窗口宽度”进一步转成“采样可信度”或“采样误差上界”。
3. 结合双电阻/单电阻采样重构约束，分析哪些扇区需要特别处理。
4. 在 SPS 功率级或完整电机模型里验证 duty clamp 逻辑对实际电流观测误差的影响。