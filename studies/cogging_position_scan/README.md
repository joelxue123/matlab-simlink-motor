# Cogging Position Scan Study

这个目录收口“位置扫描建表 + 离线验证”这条研究链。

## 入口

```matlab
cd studies/cogging_position_scan
init_project_paths
result = run_study;
```

一键复现扫描到验证：

```matlab
cd studies/cogging_position_scan
init_project_paths
result = reproduce_validation;
```

## 两个入口的区别

- `run_study`：直接运行“正反向扫描 + 建表 + 验证”主流程
- `reproduce_validation`：按一组更固定的轻量参数复现“扫描结果提取 + 离线验证 + 导出”链路

如果你主要想研究扫描策略本身，用 `run_study`。

如果你主要想稳定复现一条标准链路并导出表文件，用 `reproduce_validation`。

## 常用参数

`run_study(cfg)` 支持传入结构体覆盖参数，最常用的是：

- `scan_points`：扫描点数，常见 72、180、360
- `hold_time`：每个位置点保持时间
- `settle_time`：每个位置点的稳定等待时间
- `avg_time`：每个位置点末尾取平均的时间窗
- `position_bandwidth_scale`：相对默认位置环带宽的比例
- `position_output_limit_scale`：相对默认速度限幅的比例
- `use_iq_meas`：是否用 `iq_meas` 建表，而不是 `iq_ref`
- `plot_results`：是否画图
- `ff_table_file`：输出前馈表文件名
- `amp1`、`harmonic1`、`phase1_deg`：主周期负载项
- `amp2`、`harmonic2`、`phase2_deg`：第二谐波负载项

`reproduce_validation(cfg)` 常用参数：

- `scan_points`
- `hold_time`
- `avg_time`
- `settle_time`
- `plot_results`
- `scan_file_name`
- `ff_table_file`
- `ff_csv_file`
- `ff_text_file`

## 示例命令

### 1. 默认运行

```matlab
cd studies/cogging_position_scan
init_project_paths
result = run_study;
```

### 2. 用更稀疏的扫描点快速试跑

```matlab
cfg = struct('scan_points', 72, 'hold_time', 0.03, 'plot_results', true);
result = run_study(cfg);
```

### 3. 改成用 `iq_meas` 建表

```matlab
cfg = struct('use_iq_meas', true, 'scan_points', 180);
result = run_study(cfg);
```

### 4. 复现并导出到指定文件名

```matlab
cfg = struct( ...
	'scan_points', 72, ...
	'ff_table_file', 'scan_table.mat', ...
	'ff_csv_file', 'scan_table.csv', ...
	'ff_text_file', 'scan_table.txt');
result = reproduce_validation(cfg);
```

## 这里为什么单独成目录

这条链路会同时生成多类产物：

- 扫描结果 `mat`
- 前馈表 `mat/csv/txt`
- 验证阶段对比结果

如果仍然直接从根目录跑，结果文件会持续堆在项目根目录，后面很难判断哪个文件属于哪次实验。

## 输出位置

所有默认产物都会写到：

- `outputs/`

常见输出包括：

- `cogging_scan_ff_table.mat`
- `position_scan_iq_result.mat`
- `validated_scan_ff_table.mat`
- `validated_scan_ff_table.csv`
- `validated_scan_ff_table.txt`

## 典型使用场景

- 想从位置环稳态电流中提取周期扰动前馈表
- 想比较不同扫描点数、平均窗口、位置环带宽对建表质量的影响
- 想把扫描结果导出给控制器侧使用

## 不建议在这里做的事情

- 通用评价函数堆在本目录
- 模型构建公共函数堆在本目录
- 多个实验都会调用的导出逻辑只留在这里

这些应该回收到公共层。

## 复用的公共库

这个目录本身只放实验入口，不复制公共实现。实际复用的仍然是根目录公共层：

- `motor_control_params.m`
- `build_average_inverter_foc_model.m`
- `build_vibration_comp_test.m`
- `apply_cogging_load_config.m`
- `cogging_load_config.m`
- `algorithms/`
- `build_modules/`

## 推荐约定

- 新的扫描参数对比，优先在这里新增入口，而不是回根目录新增脚本
- 如果需要专门的后处理图表，也放在这个目录里
- 如果某个后处理开始被多个实验共用，再抽回公共库