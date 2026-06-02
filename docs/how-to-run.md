# How To Run

这份文档只回答一件事：这个工程里的 `.m` 文件应该怎么运行。

## 运行前准备

### 1. 打开工程根目录

在 MATLAB 里把当前目录切到项目根目录：

```matlab
cd /home/user/study/matlab-practice/average-inverter
```

### 2. 初始化路径

如果你准备从 `studies/` 子目录或者其他子目录运行入口，先执行：

```matlab
init_project_paths
```

这个函数会把下面这些公共目录加到 MATLAB path：

- 项目根目录
- `algorithms/`
- `build_modules/`
- `studies/`

## 最推荐的运行方式

优先从 `studies/` 目录下的实验入口运行。

原因：

- 每个实验有独立目录
- 输出会默认写到该实验自己的 `outputs/`
- 不会继续把结果文件堆到根目录

### 1. 位置扫描建表

```matlab
cd /home/user/study/matlab-practice/average-inverter/studies/cogging_position_scan
init_project_paths
result = run_study;
```

### 2. 扫描到验证一键复现

```matlab
cd /home/user/study/matlab-practice/average-inverter/studies/cogging_position_scan
init_project_paths
result = reproduce_validation;
```

### 3. 齿槽转矩补偿研究

```matlab
cd /home/user/study/matlab-practice/average-inverter/studies/cogging_torque_comp
init_project_paths
result = run_study;
```

### 4. 基础振动补偿展示

```matlab
cd /home/user/study/matlab-practice/average-inverter/studies/vibration_comp
init_project_paths
result = run_effect_demo;
```

## 从根目录直接运行的常用命令

如果你暂时还在沿用旧入口，也可以直接在根目录运行下面这些 `.m` 文件。

### 模型生成

生成控制参数并打开主 FOC 模型：

```matlab
motor_control_params;
build_model_and_open;
```

只生成主模型：

```matlab
motor_control_params;
build_average_inverter_foc_model;
open_system('average_inverter_foc');
```

生成振动补偿测试模型：

```matlab
motor_control_params;
build_vibration_comp_test;
open_system('vibration_comp_test');
```

### 基础研究入口

振动补偿效果展示：

```matlab
show_vibration_comp_effect;
```

齿槽转矩补偿研究：

```matlab
result = run_cogging_torque_comp_study;
```

位控扫描建表：

```matlab
result = run_cogging_position_scan_study;
```

扫描到验证一键复现：

```matlab
result = reproduce_position_scan_validation;
```

位置环 PIDREG3 测试：

```matlab
result = run_position_pidreg3_test;
```

### 独立分析脚本

速度环带宽估计：

```matlab
report = estimate_speedloop_bandwidth;
```

位置环 chirp 带宽估计：

```matlab
report = estimate_positionloop_bandwidth_chirp;
```

位置环扫频辨识：

```matlab
report = identify_positionloop_closedloop_tf_swept_sine;
```

比较 PIDREG3 在 `Ki = 0` 时的行为：

```matlab
result = analyze_position_pidreg3_ki0_effect;
result = compare_position_pidreg3_ki0_fullmodel;
```

### 外部示例相关入口

如果你已经安装并配置好相关 MATLAB 示例和目标板，也可以运行：

```matlab
run_host_offset
```

这个脚本不是本项目内部模型入口，而是跳到外部 Example 目录，打开并启动主机侧 offset 计算模型。

## 结果文件会写到哪里

### 从 `studies/` 入口运行时

结果默认写到各自实验目录下的：

- `outputs/`

### 从根目录旧入口运行时

结果通常写到当前工作目录。

如果当前目录是项目根目录，那么这些文件会直接出现在根目录，例如：

- `cogging_ff_table.mat`
- `cogging_scan_ff_table.mat`
- `position_scan_iq_result.mat`
- `validated_scan_ff_table.csv`

因此更推荐从 `studies/` 入口运行。

## 只想打开模型，不马上跑仿真

打开位置扫描模型：

```matlab
open_cogging_position_scan_model;
```

打开扫描表验证模型：

```matlab
open_position_scan_ff_validation_model;
```

## 其他独立目录怎么运行

### 零点瞬态研究

```matlab
cd /home/user/study/matlab-practice/average-inverter/zero_response_study
run_zero_transient_study
```

同极点、不同零点的最小示例：

```matlab
cd /home/user/study/matlab-practice/average-inverter/zero_response_study
run_same_poles_zero_example
```

### z 域稳定性演示

```matlab
cd /home/user/study/matlab-practice/average-inverter/z_stability_demo
run_z_stability_demo
```

连续域与离散域模型对比：

```matlab
cd /home/user/study/matlab-practice/average-inverter/z_stability_demo
compare_s_z_models
```

### 开关采样窗口研究

```matlab
cd /home/user/study/matlab-practice/average-inverter/switching_sampling_study
run_switching_sampling_study
```

常用入口还有：

```matlab
run_triangle_carrier_study
run_mcu_sampling_window_study
run_rl_sampling_impact_study
run_rotating_rl_sampling_study
```

## 常见建议

### 建议 1

新实验优先放到 `studies/<name>/`，并提供一个短入口，例如 `run_study.m`。

### 建议 2

如果某个 `.m` 文件只是公共函数，不要单独 `run` 它，例如：

- `algorithms/*.m`
- `build_modules/*.m`

这些文件是被模型生成脚本或 MATLAB Function block 间接调用的。

### 建议 3

如果你不确定某个 `.m` 文件是入口还是库，先看：

- [m-file-index.md](/home/user/study/matlab-practice/average-inverter/docs/m-file-index.md)
