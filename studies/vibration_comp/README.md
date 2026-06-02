# Vibration Compensation Study

这个目录收口最基础的振动补偿效果展示。

## 入口

```matlab
cd studies/vibration_comp
init_project_paths
result = run_effect_demo;
```

## 这个入口做什么

`run_effect_demo` 会调用现有基础振动补偿展示流程，自动完成：

- baseline 仿真
- 离线学习前馈表
- offline FF 仿真
- 对比补偿前后速度纹波

这是最适合“先确认整条补偿链路能不能跑通”的入口。

## 常用运行方式

### 1. 默认运行

```matlab
cd studies/vibration_comp
init_project_paths
result = run_effect_demo;
```

### 2. 如果你想手工分步跑

```matlab
cd /home/user/study/matlab-practice/average-inverter
motor_control_params;
show_vibration_comp_effect;
```

### 3. 如果你只想先学习补偿表

```matlab
cd /home/user/study/matlab-practice/average-inverter
motor_control_params;
table_info = learn_vibration_ff_table;
```

## 输出位置

默认产物写到：

- `outputs/`

常见输出包括：

- `vib_ff_table.mat`
- 仿真对比图窗口
- 工作区中的 `vib_compare_result`

## 常见关注点

- 是否能稳定学习到非零前馈表
- 补偿前后速度纹波标准差是否下降
- `iq_ff` 是否过早饱和
- 学习窗口和验证窗口是否选得合理

## 适合作为新实验起点的情况

- 想先复用现有 offline FF 验证框架
- 只是替换学习信号、查表策略或评价指标
- 想快速比较补偿前后速度纹波

如果实验开始明显转向位置扫描、齿槽建表或特定工况复现，就应该新开一个独立 study 子目录，而不是继续堆在这里。

## 从这里再往外扩展时的建议

- 如果只是改学习信号或补偿评价指标，可以继续在这里加脚本
- 如果开始引入新的建表方式或新的实验工况链路，建议新建独立 study 目录