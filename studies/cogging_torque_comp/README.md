# Cogging Torque Compensation Study

这个目录收口齿槽转矩补偿相关实验。

## 入口

```matlab
cd studies/cogging_torque_comp
init_project_paths
result = run_study;
```

## 常用参数

`run_study(cfg)` 支持传入结构体覆盖配置，常用项包括：

- `table_points`：离线学习表点数
- `phase_advance_deg`：补偿相位超前角
- `ff_output_limit`：前馈输出限幅
- `ff_enable_time`：前馈使能时间
- `learn_start_time`：离线学习开始时间
- `test_stop_time`：测试停止时间
- `speed_ref_scale`：速度参考缩放
- `load_base_torque`：基线负载转矩
- `amp1`、`harmonic1`、`phase1_deg`：主扰动项
- `amp2`、`harmonic2`、`phase2_deg`：第二扰动项

## 示例命令

### 1. 默认运行

```matlab
cd studies/cogging_torque_comp
init_project_paths
result = run_study;
```

### 2. 扫不同相位超前

```matlab
cfg = struct('phase_advance_deg', 12, 'table_points', 180);
result = run_study(cfg);
```

### 3. 改成更强的单周期扰动

```matlab
cfg = struct('amp1', 0.10, 'harmonic1', 1, 'amp2', 0, 'speed_ref_scale', 1.0);
result = run_study(cfg);
```

### 4. 测试更高速度下的补偿效果

```matlab
cfg = struct('speed_ref_scale', 1.5, 'ff_output_limit', 0.35);
result = run_study(cfg);
```

## 输出位置

默认产物写到：

- `outputs/`

典型输出包括：

- `cogging_ff_table.mat`
- 仿真阶段生成的中间结果

运行后通常还会在工作区得到：

- `cogging_comp_result`
- `cogging_metrics`

## 适合拿来比较的维度

- 扰动谐波阶次
- 扰动幅值
- 表点数
- 相位超前
- 输出限幅
- 学习窗口与验证窗口

## 这个目录适合放什么

- 谐波阶次变化实验
- 速度参考变化实验
- 相位超前和输出限幅对比
- 对比不同学习窗口/验证窗口的实验脚本

## 不适合放什么

- 通用参数初始化
- 模型构建通用函数
- 多个实验都复用的算法函数

这些应该继续留在公共层。

## 典型使用建议

如果你只是想先验证“角度同步 FF 对周期扰动有没有效果”，先保持：

- `harmonic1 = 1`
- `amp2 = 0`
- `table_points = 180`

先把补偿链路跑通，再逐步增加谐波复杂度。