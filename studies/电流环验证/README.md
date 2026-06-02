# 电流环验证

这个实验目录用于做电流环 PI 参数算法验证。

模型特点：

- 使用平均电压逆变器模型，不走开关级细节
- 继续复用项目里已经在用的 `abc to dq`、`Current PI`、`dq to abc` 和 PMSM 平均模型
- 保留现有 PMSM 平均模型和调制限幅逻辑
- 默认把机械惯量按比例放大，用来压低转速漂移，更聚焦电流环本身
- 默认改为 `id` 方波 `2 A`，优先做不带明显转矩扰动的纯电流环验证

## 目录内容

- `run_study.m`：主入口，负责参数覆盖、建模、仿真、评价、导出结果
- `build_current_loop_test_model.m`：构建电流环专用平均模型
- `outputs/`：模型、图和结果文件默认写到这里

## 推荐用法

```matlab
cd studies/电流环验证
result = run_study;
```

如果你想回到原来的 `iq` 阶跃：

```matlab
cfg = struct('ref_axis', 'iq', 'ref_waveform', 'step', 'step_amplitude_a', 2.0);
result = run_study(cfg);
```

如果你想扫“带宽 × 相位裕度”二维区域：

```matlab
cfg = struct( ...
    'bandwidth_hz_list', [400 600 800 1000 1200], ...
    'phase_margin_deg_list', [45 50 55 60 65 70]);
report = run_bandwidth_pm_sweep(cfg);
```

按指定带宽验证单组参数：

```matlab
cfg = struct('current_bandwidth_hz', 1200, 'ref_axis', 'id', ...
    'ref_waveform', 'square', 'step_amplitude_a', 2.0);
result = run_study(cfg);
```

扫一组候选带宽：

```matlab
cfg = struct('bandwidth_hz_list', [400 800 1200 1600], ...
    'step_amplitude_a', 2.0, ...
    'plot_results', true);
result = run_study(cfg);
```

如果你想故意偏离理论 PI 参数，也可以单独缩放：

```matlab
cfg = struct('current_bandwidth_hz', 800, 'kp_scale', 0.8, 'ki_scale', 1.2);
result = run_study(cfg);
```

## 常用参数

- `current_bandwidth_hz`：单次验证的目标电流环带宽
- `bandwidth_hz_list`：带宽扫频列表；一旦给出，会逐项跑多组 case
- `tuning_method`：`'bandwidth'`、`'delay_aware'` 或 `'bandwidth_pm'`
- `phase_margin_deg`：`bandwidth_pm` 模式下的目标相位裕度
- `current_delay_s`：电流环等效延时；默认按 `1.5 * Ts_ctrl`
- `delay_safety_factor`：延时限带宽安全系数，默认 `3`
- `ref_axis`：`'id'` 或 `'iq'`
- `ref_waveform`：`'step'` 或 `'square'`
- `kp_scale` / `ki_scale`：在带宽法计算值基础上再做缩放
- `step_amplitude_a`：参考电流幅值
- `step_time_s`：参考开始时刻
- `square_frequency_hz`：方波频率，仅在 `square` 模式下使用
- `stop_time_s`：总仿真时长
- `inertia_scale`：机械惯量放大倍数，默认用于降低速度漂移
- `plot_results`：是否出图
- `save_outputs`：是否把结果写到 `outputs/`

## 默认输出

默认会在 `outputs/` 下生成：

- `currentloop_pi_test.slx`
- `current_loop_validation_summary.csv`
- `current_loop_validation_result.mat`
- `waveform_*.csv`
- `result_*.mat`
- `response_*.png`
- 多 case 扫描时额外输出 `current_loop_validation_summary.png`

## 适合放什么

- 电流环带宽法参数验证
- `Kp/Ki` 缩放敏感性比较
- `id` 方波、`iq` 阶跃两类工况的快速切换
- 主轴电流跟踪、交叉轴串扰、电压利用率的快速对比

## 关于延时

当前项目原始推荐值是理想 RL 带宽法：

- `Kp = L * wc`
- `Ki = R * wc`

这默认没有把电流环的数字延时单独折进推荐带宽里。

如果你想保守一些，可以在 study 里改成：

```matlab
cfg = struct('tuning_method', 'delay_aware', ...
    'current_bandwidth_hz', 800, ...
    'ref_axis', 'id', ...
    'ref_waveform', 'square');
result = run_study(cfg);
```

这个模式会按下面的限带宽规则取有效带宽：

- `bw_eff = min(bw_req, 1 / (2*pi*delay_safety_factor*current_delay_s))`

默认 `current_delay_s = 1.5 * Ts_ctrl`，和项目里速度环处理延时的口径保持一致。

如果你想保留“目标带宽”这个概念，但又把延时正式带进 PI 参数公式，可以用：

```matlab
cfg = struct('tuning_method', 'bandwidth_pm', ...
    'current_bandwidth_hz', 800, ...
    'phase_margin_deg', 60, ...
    'ref_axis', 'id', ...
    'ref_waveform', 'square');
result = run_study(cfg);
```

`bandwidth_pm` 模式使用的是“目标带宽 + 目标相位裕度 + 延时”联合设计：

- 指定目标交越频率 `wc = 2*pi*bw`
- 指定目标相位裕度 `PM`
- 考虑对象 `1 / (Ls + R)` 和延时 `exp(-sTd)`
- 直接反算 PI 零点和 `Kp`、`Ki`

它比简单的 `Kp = L*wc, Ki = R*wc` 更适合回答“我想保留带宽概念，但又不想忽略延时”这个问题。

如果你不确定该选哪组 `带宽 + 相位裕度`，可以直接跑：

- `run_bandwidth_pm_sweep.m`

它会输出：

- 每个组合的 summary 表
- overshoot / settling / rmse 热力图
- 最小超调、最短调节时间、最小 RMSE 的组合

## 不适合放什么

- 速度环参数整定
- 位置环算法验证
- 开关纹波、采样点时序、死区效应这类开关级问题

这套实验默认更偏“平均模型上的控制参数验证”，不是硬件最终闭环表现的完整替代。