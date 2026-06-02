# M-File Index

这份文档记录项目里主要 `.m` 文件的用途，以及它们属于哪一层。

说明：

- 这里只记录主要顶层 `.m` 文件和关键公共库
- 不记录每个文件内部的 `local_*` 局部函数
- `algorithms/` 和 `build_modules/` 里的文件默认视为公共库，不是用户直接入口

## 1. 推荐入口：Studies

| 文件 | 类型 | 作用 | 直接运行 |
| --- | --- | --- | --- |
| `studies/cogging_position_scan/run_study.m` | 实验入口 | 位置扫描建表并验证 | 是 |
| `studies/cogging_position_scan/reproduce_validation.m` | 实验入口 | 扫描到验证一键复现 | 是 |
| `studies/cogging_torque_comp/run_study.m` | 实验入口 | 齿槽转矩补偿研究 | 是 |
| `studies/vibration_comp/run_effect_demo.m` | 实验入口 | 基础振动补偿效果展示 | 是 |
| `studies/_template/run_study.m` | 模板 | 新实验模板入口 | 否 |

## 2. 根目录兼容入口

| 文件 | 类型 | 作用 | 直接运行 |
| --- | --- | --- | --- |
| `show_vibration_comp_effect.m` | 研究入口 | 基线与 offline FF 对比展示 | 是 |
| `run_cogging_torque_comp_study.m` | 研究入口 | 齿槽转矩补偿主入口 | 是 |
| `run_cogging_position_scan_study.m` | 研究入口 | 正反向位置扫描建表并验证 | 是 |
| `reproduce_position_scan_validation.m` | 研究入口 | 扫描到验证链路一键复现 | 是 |
| `run_position_pidreg3_test.m` | 研究入口 | 位置环 PIDREG3 测试 | 是 |
| `run_host_offset.m` | 外部示例入口 | 打开并运行 QEP offset host 模型 | 是，依赖外部示例 |

## 3. 参数与配置公共层

| 文件 | 类型 | 作用 | 直接运行 |
| --- | --- | --- | --- |
| `motor_control_params.m` | 参数脚本 | 初始化 `motor / inverter / control / simcfg` | 是，脚本 |
| `cogging_load_config.m` | 配置函数 | 统一齿槽/周期负载配置 | 否 |
| `apply_cogging_load_config.m` | 配置函数 | 把负载配置写回 `control.vib` | 否 |
| `init_project_paths.m` | 工具函数 | 初始化项目路径 | 是 |

## 4. 模型构建与打开

| 文件 | 类型 | 作用 | 直接运行 |
| --- | --- | --- | --- |
| `build_model_and_open.m` | 模型入口 | 生成并打开主 FOC 模型 | 是 |
| `build_average_inverter_foc_model.m` | 模型构建 | 生成 `average_inverter_foc.slx` | 是 |
| `build_vibration_comp_test.m` | 模型构建 | 生成 `vibration_comp_test.slx` | 是 |
| `build_speedloop_kf_test.m` | 模型构建 | 生成速度环 KF 测试模型 | 是 |
| `build_openloop_test.m` | 模型构建 | 生成开环测试模型 | 是 |
| `open_cogging_position_scan_model.m` | 模型打开 | 打开位置扫描模型 | 是 |
| `open_position_scan_ff_validation_model.m` | 模型打开 | 打开扫描表验证模型 | 是 |

## 5. 数据提取、导出与验证

| 文件 | 类型 | 作用 | 直接运行 |
| --- | --- | --- | --- |
| `save_position_scan_iq_table.m` | 数据提取 | 从扫描仿真提取每步 `iq` | 是 |
| `validate_position_scan_ff_table.m` | 验证入口 | 由扫描结果生成 FF 表并做离线验证 | 是 |
| `export_position_scan_ff_csv.m` | 导出工具 | 从扫描结果直接导出 CSV/MAT | 是 |
| `learn_vibration_ff_table.m` | 学习工具 | 从日志离线学习振动补偿表 | 是 |
| `evaluate_vibration_comp_test.m` | 评价工具 | 计算补偿前后纹波指标 | 是 |
| `evaluate_speedloop_kf_test.m` | 评价工具 | 评价速度环 KF 测试结果 | 是 |

## 6. 分析、辨识与对比脚本

| 文件 | 类型 | 作用 | 直接运行 |
| --- | --- | --- | --- |
| `estimate_speedloop_bandwidth.m` | 分析脚本 | 估计速度环带宽 | 是 |
| `estimate_positionloop_bandwidth_chirp.m` | 分析脚本 | 用 chirp 估计位置环带宽 | 是 |
| `identify_positionloop_closedloop_tf_swept_sine.m` | 辨识脚本 | 用扫频数据拟合闭环传函 | 是 |
| `analyze_position_pidreg3_ki0_effect.m` | 对比脚本 | 研究 `Ki = 0` 时 Ui/Kc 影响 | 是 |
| `compare_position_pidreg3_ki0_fullmodel.m` | 对比脚本 | 在全模型中对比 `Ki = 0` 情况 | 是 |
| `check_position_pidreg3_ui_current.m` | 检查脚本 | 重建当前工程位置环 Ui | 是 |
| `validate_pid_reg3_calc.m` | 校验脚本 | 校验 PIDREG3 离散实现 | 是 |
| `sweep_speedloop_kf.m` | 扫参脚本 | 扫速度环/KF 参数 | 是 |
| `sweep_vibration_phase_advance.m` | 扫参脚本 | 扫补偿相位超前 | 是 |
| `compare_zero_vector_sampling.m` | 分析脚本 | 比较零矢量分配与采样窗口 | 是 |

## 7. Linux / C2000 工具脚本

| 文件 | 类型 | 作用 | 直接运行 |
| --- | --- | --- | --- |
| `prepare_c2000_gui_build_linux.m` | 环境工具 | 准备 Linux 下 C2000 GUI 构建环境 | 是 |
| `diagnose_c2000_gui_build_linux.m` | 环境工具 | 诊断 Linux 下 GUI 构建问题 | 是 |
| `fix_controlsuite_registry_linux.m` | 环境工具 | 修复 ControlSUITE 注册表路径 | 是 |
| `register_c2000_tools.m` | 环境工具 | 注册 C2000 工具链 | 是 |
| `setup_serial_12m.m` | 环境工具 | 串口 12M 配置脚本 | 是 |
| `launch_matlab_12m.sh` | Shell 脚本 | Linux 下启动 MATLAB 的辅助脚本 | 否，Shell |
| `setup_controlsuite_compat.sh` | Shell 脚本 | Linux 下兼容性设置 | 否，Shell |

## 8. 公共算法库：algorithms/

这些文件主要被 Simulink 模型或 MATLAB Function block 间接调用，不建议单独当入口运行。

| 文件 | 作用 |
| --- | --- |
| `algorithms/abc2dq_fcn.m` | 三相电流到 `dq` 变换 |
| `algorithms/dq2abc_fcn.m` | `dq` 电压到三相调制量 |
| `algorithms/current_pi_fcn.m` | 电流环 PI |
| `algorithms/speed_pi_fcn.m` | 速度环 PI |
| `algorithms/kalman_speed_estimator.m` | 速度/角度 Kalman 估计 |
| `algorithms/periodic_load_torque.m` | 周期负载模型 |
| `algorithms/vibration_compensator.m` | 在线振动补偿 |
| `algorithms/vibration_ff_lookup.m` | 查表式前馈补偿 |
| `algorithms/position_pidreg3_fcn.m` | 位置环 PIDREG3 |
| `algorithms/traj_planner.m` | 位置轨迹规划 |
| `algorithms/pos_unwrap.m` | 位置解缠 |
| `algorithms/position_scan_ref_fcn.m` | 扫描位置参考 |
| `algorithms/position_sine_ref_fcn.m` | 正弦位置参考 |
| `algorithms/position_chirp_ref_fcn.m` | Chirp 位置参考 |

## 9. 模型搭建库：build_modules/

这些文件是 `build_*.m` 模型构建脚本内部使用的模块拼装函数。

| 文件前缀 | 作用 |
| --- | --- |
| `add_*.m` | 添加 `Goto/From` 等通用块 |
| `create_subsystem.m` | 创建子系统骨架 |
| `embed_algorithm.m` | 把算法函数嵌入 MATLAB Function block |
| `populate_*.m` | 往子系统填充具体算法结构 |

这类文件不作为用户入口文档化到运行级别，但属于关键公共库。

## 10. 独立研究子目录

### `zero_response_study/`

| 文件 | 作用 | 直接运行 |
| --- | --- | --- |
| `zero_response_study/run_zero_transient_study.m` | 比较不同零点对瞬态响应的影响 | 是 |
| `zero_response_study/run_same_poles_zero_example.m` | 相同极点、不同零点的最小示例 | 是 |

### `z_stability_demo/`

| 文件 | 作用 | 直接运行 |
| --- | --- | --- |
| `z_stability_demo/run_z_stability_demo.m` | z 域稳定性演示入口 | 是 |
| `z_stability_demo/analyze_z_stability.m` | z 域稳定性分析函数 | 是 |
| `z_stability_demo/plot_z_analysis.m` | z 域绘图函数 | 否，通常被调用 |
| `z_stability_demo/compare_s_z_models.m` | 连续域/离散域模型对比 | 是 |

### `switching_sampling_study/`

这个目录已经有自己的 README，主要入口包括：

- `run_switching_sampling_study`
- `run_triangle_carrier_study`
- `run_mcu_sampling_window_study`
- `run_rl_sampling_impact_study`
- `run_rotating_rl_sampling_study`

## 11. 快速判断一个 `.m` 文件该不该直接运行

可以按下面规则判断：

### 可以直接运行

- 文件名像 `run_*`、`build_*`、`open_*`、`show_*`、`evaluate_*`、`estimate_*`
- 顶层函数明显是实验入口、模型入口、分析入口

### 不建议直接运行

- 位于 `algorithms/`
- 位于 `build_modules/`
- 文件主要是给模型构建脚本或 MATLAB Function block 调用

## 12. 配套运行文档

如何运行见：

- [how-to-run.md](/home/user/study/matlab-practice/average-inverter/docs/how-to-run.md)