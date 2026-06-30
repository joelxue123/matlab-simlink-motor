# Dexterous Hand Impedance Control Plan

标签：`[LEGACY]` + `[RESEARCH]`。该目录保留历史建模和验证价值，但其 `grt`/`MATLAB Function` 路线不是当前新的嵌入式交付模板。

这个文件夹用于整理灵巧手抓取包裹时的阻抗控制、降电流、降发热算法计划，后期用于 MATLAB、Simulink 和 Python 联合仿真。

## 核心问题

灵巧手抓住物体后，目标关节位置 `q_d` 仍然在继续闭合，实际关节位置 `q` 被物体阻挡，导致长期位置误差：

```text
e = q_d - q
```

PD 控制输出近似为：

```text
tau = Kp e - Kd q_dot
```

电流与力矩近似相关：

```text
I = tau / Kt
```

发热主要与电流平方相关：

```text
P_heat = I^2 R
```

因此发热根源不是单独的 `Kp` 或位置误差，而是长期存在的：

```text
Kp * e
```

## 总体路线

第一阶段做可落地算法：

```text
状态机 + 低刚度阻抗 + 误差限幅 + 力矩/电流饱和 + PI 调握紧变量
```

第二阶段做研究算法：

```text
扰动观测器 + 自适应阻抗 + CBF/QP 安全约束 + 低维 MPC 优化
```

## 建议目录

```text
dexterous_hand_impedance_plan/
  README.md
  docs/
    simulation_plan.md
    algorithms.md
    stability_and_bandwidth.md
  matlab/
    后续放 MATLAB 脚本
  simulink/
    后续放 Simulink 模型
  python/
    后续放 Python 分析脚本
  results/
    后续放仿真结果、图表、csv、mat 文件
```

## 第一版最小闭环

建议先仿真单关节，再扩展到多关节和多指。

单关节模型：

```text
J q_ddot + b q_dot = tau + tau_ext
```

阻抗/PD 控制：

```text
tau = sat(K(q_d - q) - D q_dot, tau_max)
```

接触后目标位置：

```text
q_d = q_contact + S alpha
```

其中 `alpha` 是整体握紧量，不是每个关节独立调位置。

PI 外环：

```text
alpha_dot = sat(ki(F_ref - F_hat), alpha_dot_max)
alpha = sat(alpha, alpha_min, alpha_max)
```

或者调力矩上限：

```text
tau_max_dot = sat(ki(F_ref - F_hat), tau_dot_max)
tau_max = sat(tau_max, tau_min, tau_limit)
```

## 关键验证指标

需要对比原始 PD 和改进算法：

```text
I_mean
I_rms
I_peak
tau_mean
tau_peak
position_error_rms
contact_force_peak
slip_count
grasp_success_rate
temperature_rise
```

最重要的结论指标：

```text
I_rms 降低
温升降低
抓取成功率不下降
滑移次数可接受
接触冲击峰值降低
```

## 下一步

1. 先实现单关节 MATLAB 仿真。
2. 对比原始 PD、限幅 PD、阻抗保持、PI 调握紧量四组。
3. 在 Simulink 里搭建状态机和饱和模块。
4. 用 Python 读取结果，自动生成对比图和指标表。
