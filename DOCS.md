# matlab-practice 协同文档

本文件用于 `matlab-practice/` 目录内的 AI 协同、MATLAB/Simulink 建模、MBD 代码生成和实验交接。已有笔记负责具体知识点，本文件负责沟通入口和跨目录衔接。

## 目录定位

- MATLAB/Simulink 电机控制与模型化开发练习目录。
- 内容包括电流环、速度环、Clarke/Park、PI、定点 Q 格式、Embedded Coder、HPM EtherCAT 轨迹实验、阻抗控制、参数辨识和学习笔记。
- 该目录常作为算法原型、仿真验证、代码生成和硬件实验数据分析的中间层。

## 进入前先看

- `docs/README.md`：本仓库长期 MBD 协作入口，包含进度、模型开发规范、可复用模块使用和 AI 协作规则。
- `docs/modeling_scope_decision_guide.md`：平均电压模型、开关型模型、MBD/codegen 和仿真 harness 的边界指南。
- `MBD_DEVELOPMENT_NOTES.md`：MBD 开发笔记。
- `MBD_ERRATA.md`：已知问题和勘误。
- 各子项目的 `README.md`：具体模型、脚本和实验说明。
- `motor_control_modules/`：电机控制模块。
- `motor_current_loop_mbd/`、`motor_speed_current_loop_mbd/`：闭环 MBD 练习。
- `fixed_point_pi_q14_simulink/`：定点 PI/Q14 相关笔记。
- `identification/coreless_motor_12v_identification/`：12V 空心杯小惯量参数辨识文献综述与仿真入口。
- `identification/pmsm_electrical_parameter_identification/`：PMSM 电阻、电感、磁链和编码器对齐辨识 sandbox。
- `motor_performance_characterization/`：电机性能检测与谐波/传感器误差回归测试。
- `pwm_deadtime_sampling_mbd/`：高占空比死区/采样窗口 valid 判定 MBD 模块，可生成 C。
- `pwm_deadtime_compensation_mbd/`：50us dq 合成相电流极性的死区 duty 补偿 MBD 模块，可生成可重入 C。
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
- 做 PMSM 电参辨识：估计 `Rs/Ld/Lq/psi_f`，并为无感/有感角度协同和编码器对齐提供数据。
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

### 2026-06-10 - 增加 PMSM 电参辨识与编码器对齐 sandbox

- 目标：建立 `Rs/Ld/Lq/psi_f` 和编码器 offset/残差辨识的 MATLAB 第一版，用于后续无感观测器和有感编码器协同。
- 已完成：新增 `identification/pmsm_electrical_parameter_identification/`。
- 方法：standstill d/q 电压阶跃估计 `Rs/Ld/Lq`，旋转反电势估计 `psi_f`，sensorless/sensored angle 差值估计 encoder offset 与 1x/2x 残差。
- 验证方式：运行 `matlab -batch "run('identification/pmsm_electrical_parameter_identification/run_pmsm_electrical_id_demo.m')"`。
- 当前合成数据误差：`Rs -0.010%`，`Ld 0.513%`，`Lq -0.050%`，`psi_f -0.002%`，encoder offset 误差 `1.18e-05 rad`。

### 2026-06-10 - 增加 docs 规范入口

- 目标：把 `matlab-practice/` 的进度、模型开发规范、模块复用方法和 AI 协作规则集中管理，避免每次协作按不同风格重建。
- 已完成：新增 `docs/README.md`、`docs/progress.md`、`docs/model_development_standard.md`、`docs/reusable_modules_usage.md`、`docs/ai_collaboration_rules.md`。
- 关键规则：未来新 MBD 模块优先遵守 `docs/model_development_standard.md`；共享模块复用优先遵守 `docs/reusable_modules_usage.md`；AI 接手任务先读 `docs/README.md`。
- 验证方式：文档链接和 git diff 检查。

### 2026-06-10 - 增加高占空比死区采样窗口 MBD 模块

- 目标：把 `average-inverter/switching_sampling_study/` 的死区/低边采样窗口经验，抽成可生成 C 的 MBD 模块。
- 已完成：新增 `pwm_deadtime_sampling_mbd/`，包含 build、test、codegen 脚本、`.sldd` 和 `.slx`。
- 复现条件：`R=4 ohm`，`L=100uH`，`PWM=20kHz`，`deadtime=500ns`，`modulation=0.9`。
- 复现结果：RL 采样点相对周期平均值 RMS 误差 `0.302540 A`，采样相纹波 `3.348031 A pk-pk`。
- MBD 测试结果：`usable_low=[45.5, 0.5, 23.0] us`，`sample_valid=[1,0,1]`，高占空比相被判定 invalid。
- 生成接口：`DeadtimeSamplingWindowStep(const pwm_phase_duty_t *rtu_duty_in, pwm_sampling_status_t *rty_status_out)`。

### 2026-06-10 - 增加死区 duty 补偿 MBD 模块

- 目标：把 50us 周期的 dq 合成相电流极性死区补偿做成用户级 MBD 算法 core，而不是只放在开关级验证 harness 里。
- 已完成：新增 `pwm_deadtime_compensation_mbd/`，包含 build、test、codegen 脚本、`.sldd`、`.slx` 和生成代码。
- 算法：由 `id/iq/sin_theta_e/cos_theta_e` 合成三相电流极性；小电流区不补偿，过渡区按 `gain=clamp((abs(i_synth)-current_zero)/(current_full-current_zero),0,1)` 平滑放大，大电流区按合成相电流极性满补偿。
- 默认：`sample time=50us`，`comp_duty=0.01000`，`current_zero=0.02A`，`current_full=0.10A`，`polarity=-1`。
- 生成接口：`DeadtimeCompensationStep(const pwm_deadtime_comp_input_t *rtu_comp_in, pwm_deadtime_comp_output_t *rty_comp_out)`。
- 验证：功能测试 PASS，开关级 `Universal Bridge + PMSM` smoke test PASS。

### 2026-06-11 - 增加模型选择和 MBD 边界指南

- 目标：固定“什么时候用平均电压模型、什么时候用开关型模型、什么模块 MBD 化、什么模块保持原有仿真形态”的工程判断。
- 已完成：新增 `docs/modeling_scope_decision_guide.md`。
- 关键结论：平均电压模型用于控制主线和长时间闭环仿真；开关型模型用于 PWM 边沿、死区、采样窗口、电流纹波和器件导通验证；可交付算法 core 和稳定 adapter 做 MBD/codegen；物理 plant、验证 harness、参数扫描和报告脚本保持原有仿真/研究形态。
- 已同步：`docs/README.md`、`docs/model_development_standard.md`、`docs/progress.md`、`MBD_DEVELOPMENT_NOTES.md`。
