# matlab-practice 协同文档

本文件用于 `matlab-practice/` 目录内的 AI 协同、MATLAB/Simulink 建模、MBD 代码生成和实验交接。已有笔记负责具体知识点，本文件负责沟通入口和跨目录衔接。

## 目录定位

- MATLAB/Simulink 电机控制与模型化开发练习目录。
- 内容包括电流环、速度环、Clarke/Park、PI、定点 Q 格式、Embedded Coder、HPM EtherCAT 轨迹实验、阻抗控制、参数辨识和学习笔记。
- 该目录常作为算法原型、仿真验证、代码生成和硬件实验数据分析的中间层。

## 进入前先看

- `MBD_DEVELOPMENT_NOTES.md`：MBD 开发笔记。
- `MBD_ERRATA.md`：已知问题和勘误。
- 各子项目的 `README.md`：具体模型、脚本和实验说明。
- `motor_control_modules/`：电机控制模块。
- `motor_current_loop_mbd/`、`motor_speed_current_loop_mbd/`：闭环 MBD 练习。
- `fixed_point_pi_q14_simulink/`：定点 PI/Q14 相关笔记。
- `identification/coreless_motor_12v_identification/`：12V 空心杯小惯量参数辨识文献综述与仿真入口。
- `motor_performance_characterization/`：电机性能检测与谐波/传感器误差回归测试。
- `hpm_ethercat_sine_0p3_1hz_30s/` 与 `hpm_ethercat_sine_0p3_1hz_30s_f2win/`：HPM EtherCAT 轨迹实验。

## AI 协同边界

- 修改 Simulink/MATLAB 任务前，要确认 MATLAB 版本、模型入口、数据字典、代码生成目标和依赖路径。
- MBD 代码生成任务要记录 solver、采样时间、数据类型、接口映射和生成代码位置。
- 不要把仿真结论直接当作硬件结论；硬件实测需记录固件版本、采样频率、负载和数据文件。
- `slprj/`、自动生成代码和临时缓存一般不作为手工维护重点，除非任务明确要求分析生成结果。

## 常用任务

- 建立或整理 Simulink 控制模型。
- 生成可嵌入 C 代码，并检查接口、类型和可重入性。
- 设计电流环/速度环/PI/观测器/定点化实验。
- 分析 HPM 或关节板实测数据，并形成可复现实验报告。
- 做小惯量电机参数辨识：先 MATLAB/Simulink 仿真验证，再读取硬件 CSV 估计 `R/L/Ke/Kt/J/B/Tc`。
- 做电机性能检测回归：编码器、电流采样、谐波、转矩波动和台架标定。

## 与其他目录的接口

- `HPM6E00EVK-RevC/`：轨迹 CSV、F2 反馈、EtherCAT 主站与 HPM 固件验证。
- `green-joint/`：关节固件控制算法、定点化和协议反馈字段。
- `device_sdk/`：测试信号、CSV 数据、频响和实时绘图分析。
- `PMSM_STUDIO_APP_v0.67/`：将仿真和算法解释转化为教学内容。

## 交接记录

### 2026-06-10 - 建立目录协同文档

- 目标：为 MATLAB/Simulink 练习目录建立 AI 协同入口。
- 已完成：新增本 `DOCS.md`，记录目录定位、入口文件、协同边界和跨目录接口。
- 关键文件：`DOCS.md`，`MBD_DEVELOPMENT_NOTES.md`，`MBD_ERRATA.md`。
- 验证方式：后续 MATLAB/MBD 任务从本文件和相关子项目 README 开始阅读。
- 风险/未决：需要在具体任务中确认 MATLAB 版本、模型路径和目标代码生成配置。
- 下一步：每次模型或数据分析任务结束后，补充模型入口、验证命令和结果摘要。

### 2026-06-10 - 增加 12V 空心杯小惯量辨识入口

- 目标：把空心杯小惯量参数辨识从聊天结论沉淀为可复用学习入口。
- 已完成：新增 `identification/coreless_motor_12v_identification/README.md`。
- 关键结论：不要直接用 `J = Te / diff(speed)` 作为主方法；优先使用电气参数 sanity check、双向 torque pulse、位置拟合加速度、摩擦项联合最小二乘，必要时加已知惯量。
- 与硬件衔接：硬件执行细节继续参考 `HPM6E00EVK-RevC/INERTIA_IDENTIFICATION_PLAN.md`，分析时间基准使用 `cmd_seq * 0.001`。
- 下一步：建立 MATLAB 合成数据和 estimator，再扩展到 Simulink plant/harness。

### 2026-06-10 - 增加电流传感器谐波回归测试

- 目标：记录电流零漂、三相电流增益不一致、噪声/PWM 纹波在 dq 坐标中的典型频率特征。
- 已完成：新增 `motor_performance_characterization/`。
- 关键结论：电流 offset 主要表现为 `1x` 电频率；电流 gain mismatch 主要表现为 `2x` 电频率；随机噪声是宽带，不是固定倍频。
- 验证方式：运行 `matlab -batch "run('motor_performance_characterization/run_current_sensor_harmonic_regression_test.m')"`。
- 输出：`results/current_sensor_harmonic_metrics.csv`、`results/current_sensor_harmonic_regression.png`、`results/current_sensor_fault_dq_timeseries.png`。
