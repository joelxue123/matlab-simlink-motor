# Progress

本文件记录 `matlab-practice/` 的 MBD 和电机算法里程碑。新任务完成后，把目录、入口脚本、验证命令和结果摘要写到这里。

## 2026-06-26：确认 MBD 生成代码与 green-joint 固件同步

目录：

```text
motor_speed_pi_mbd/
green_joint_current_loop_mbd/
green_joint_digital_twin/
../green-joint/Module/MBD/green_joint_speed_loop/
../green-joint/Module/MBD/green_joint_current_loop/
```

目标：

```text
确认当前 MBD 源模型生成的 C/H 与 green-joint 固件侧 MBD 目录一致。
同时修复会导致后续 AI 误判的生成目录漂移和参数源头不统一问题。
```

成果：

- `motor_speed_pi_mbd/generate_speed_pi_code.m` 和 `green_joint_current_loop_mbd/generate_green_joint_current_loop_code.m` 在 `slbuild` 前固定 `cd(script_dir)`，避免从其它脚本串联调用时把 `*_ert_rtw` 生成到错误目录。
- `motor_speed_pi_mbd/build_speed_pi_model.m` 不再默认删除已有 `speed_pi_interface.sldd`，并保留已有 `Kp_speed/Ki_speed/Kaw_speed/IqLimitDefault` 参数值，防止 green-joint 速度环参数被通用示例默认值覆盖。
- 电流环接口源头 `green_joint_current_loop_mbd/interface.yaml` 和 `interface.json` 已统一为当前 green-joint 调参默认值：`CurDKp=CurQKp=1.0`，`CurDKi=CurQKi=20000.0`。
- `green_joint_digital_twin/setup_green_joint_current_loop_twin.m` 保留物理公式计算值为 `*_Physical`，但控制器默认值切到当前 green-joint 调参默认值，避免物理估算值和运行调参值混成一个源头。
- 已删除误生成的临时目录 `green_joint_digital_twin/speed_pi_model_ert_rtw`，正式 speed PI 生成目录只保留 `motor_speed_pi_mbd/speed_pi_model_ert_rtw`。

当前追踪关系：

```text
speed PI 源模型:
  matlab-practice/motor_speed_pi_mbd/speed_pi_model.slx
  matlab-practice/motor_speed_pi_mbd/speed_pi_interface.sldd
  matlab-practice/motor_speed_pi_mbd/speed_pi_model_ert_rtw/

speed PI 固件副本:
  green-joint/Module/MBD/green_joint_speed_loop/
  green-joint/Module/Src/green_joint_speed_loop_mbd_adapter.c

current PI 源模型:
  matlab-practice/green_joint_current_loop_mbd/green_joint_current_loop_model.slx
  matlab-practice/green_joint_current_loop_mbd/green_joint_current_loop_interface.sldd
  matlab-practice/green_joint_current_loop_mbd/green_joint_current_loop_model_ert_rtw/

current PI 固件副本:
  green-joint/Module/MBD/green_joint_current_loop/
  green-joint/Module/Src/green_joint_current_loop_mbd_adapter.c
```

同步检查：

```bash
diff -qr matlab-practice/motor_speed_pi_mbd/speed_pi_model_ert_rtw green-joint/Module/MBD/green_joint_speed_loop
diff -qr matlab-practice/green_joint_current_loop_mbd/green_joint_current_loop_model_ert_rtw green-joint/Module/MBD/green_joint_current_loop
```

当前结果：

```text
两个 diff 都只剩 MATLAB codegen 元数据/报告文件差异：
  buildInfo.mat / codeInfo.mat / codedescriptor.dmr / compileInfo.mat
  html/ / rtw_proj.tmw / rtwtypeschksum.mat / *.mk / tmwinternal/

实际进入固件编译的 .c/.h 文件已同步。
```

关键参数抽查：

```text
speed_pi_model_data.c:
  Kp_speed = 0.302884579
  Ki_speed = 19.0308
  Kaw_speed = 125.663704
  speed_integrator_delta = 0.0005

green_joint_current_loop_model.c:
  CurDKp = CurQKp = 1.0
  CurDKi = CurQKi = 20000.0
  PiCorrectionGain = 400.0
  VoltageLimitRatio = 0.577
  VoltageModulationRatio = 0.9
```

验证命令：

```bash
matlab -batch "run('matlab-practice/green_joint_digital_twin/sync_green_joint_speed_loop_twin_parameters.m'); run('matlab-practice/motor_speed_pi_mbd/generate_speed_pi_code.m')"
GJ_DT_ALLOW_UNSAFE_REBUILD=1 matlab -batch "run('matlab-practice/green_joint_current_loop_mbd/generate_green_joint_current_loop_code.m')"
matlab -batch "run('matlab-practice/green_joint_digital_twin/sync_green_joint_current_loop_twin_parameters.m'); run('matlab-practice/motor_speed_pi_mbd/run_speed_pi_smoke_test.m'); run('matlab-practice/green_joint_current_loop_mbd/run_green_joint_current_loop_smoke_test.m')"
matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_average_motor_twin_smoke_test.m'); run('matlab-practice/green_joint_digital_twin/run_green_joint_average_motor_speed_step_test.m')"
matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_average_motor_square_wave_test.m')"
```

验证结果：

```text
Speed PI smoke test passed.
Green-joint current-loop MBD smoke test passed.
Green-joint average motor twin smoke test passed.
Speed step 0 -> 4 rad/s joint-side passed:
  rise time to 90% = 6 ms
  settling time = 32 ms
  overshoot = 16.235 %
  |iq_ref| max = 1.2496 A
  voltage_mag_norm max = 0.668392
1kHz current square-wave passed:
  Kp/Ki = 1 / 20000
  gain@1kHz fundamental = 0.997516
  lag@1kHz fundamental = 54.7103 deg / 151.973 us
  voltage_mag_norm max = 0.164503
```

## 2026-06-26：green-joint 速度环合入 V1 主线并考虑减速比

目录：

```text
green_joint_digital_twin/
motor_speed_pi_mbd/
```

目标：

```text
把速度环从孤立设计/数值验证合入 V1 average motor digital twin。
速度环输入使用减速器输出端 rad/s，输出使用电机端 Iq A，并把 183.35 减速比进入 PI 设计。
```

成果：

- `build_green_joint_average_motor_twin_model.m` 接入 `SpeedPiModelRef -> GreenJointCurrentLoopModelRef` 主线，并修复 `joint_speed_ref_step -> speed_ref_to_speed_pi` 漏接线。
- `speed_input_bus_creator` 三个输入显式命名为 `wm_ref/wm_meas/iq_limit`，消除 Model Reference bus 名称警告。
- 新增 [sync_green_joint_speed_loop_twin_parameters.m](/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/sync_green_joint_speed_loop_twin_parameters.m)，把 `design_green_joint_speed_loop.m` 的 1615 + gear ratio 参数写入 `motor_speed_pi_mbd/speed_pi_interface.sldd`。
- 新增 [run_green_joint_average_motor_speed_step_test.m](/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/run_green_joint_average_motor_speed_step_test.m)，直接运行 V1 主模型，不建立孤立速度环 plant。
- `setup_green_joint_current_loop_twin.m` 将 `motor.B` 默认改为 0，阻尼等待辨识，避免继承旧 average-inverter 示例的 `1e-4` 假阻尼。
- `run_green_joint_average_motor_twin_smoke_test.m` 改为 5ms、0.3A 的低能量 current-only sanity test。
- `motor_speed_pi_mbd/run_speed_pi_smoke_test.m` 修复临时 harness 关闭警告。

关键公式：

```text
SpeedPiStep 输入速度 = joint-side rad/s
SpeedPiStep 输出电流 = motor-side Iq A
J_speed_loop = J_motor * gear_ratio + J_load_output / gear_ratio
```

当前 1615 参数：

```text
gear_ratio = 183.35
J_motor = 0.034 kg*mm^2 = 3.4e-8 kg*m^2
J_speed_loop = 6.2339e-6 kg*m^2
Kt = 0.00517276217 N*m/A
Kp_speed = 0.302884591 A/(rad/s)
Ki_speed = 19.0308001 A/rad
Kaw_speed = 125.663706 1/s
```

验证命令：

```bash
matlab -batch "run('matlab-practice/green_joint_digital_twin/sync_green_joint_speed_loop_twin_parameters.m')"
matlab -batch "run('matlab-practice/green_joint_digital_twin/build_green_joint_average_motor_twin_model.m')"
matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_average_motor_twin_smoke_test.m')"
matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_average_motor_speed_step_test.m')"
matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_average_motor_square_wave_test.m')"
matlab -batch "run('matlab-practice/motor_speed_pi_mbd/run_speed_pi_smoke_test.m')"
```

当前结果：

```text
speed step 0 -> 4 rad/s joint-side:
  final joint speed = 4 rad/s
  final speed error = 0 rad/s
  rise time to 90% = 6 ms
  settling time = 32 ms
  overshoot = 16.235 %
  |iq_ref| max = 1.2496 A
  |iq| max = 1.22198 A
  voltage_mag_norm max = 0.668392

current-only smoke:
  iq_ref final = 0.3 A
  iq final = 0.292832 A
  voltage_mag_norm max = 0.234304

1kHz current square-wave:
  gain@1kHz fundamental = 0.997516
  lag@1kHz fundamental = 54.7103 deg / 151.973 us
  voltage_mag_norm max = 0.164503
```

## 2026-06-26：digital twin 电流环改为 Model Reference

目录：

```text
green_joint_digital_twin/
```

目标：

```text
消除 digital twin 对 GreenJointCurrentLoopStep 的复制块依赖。
让 green_joint_current_loop_mbd 成为唯一电流环源头，digital twin 通过 Model Reference 和 dictionary reference 正式复用。
```

成果：

- `build_green_joint_current_loop_twin_model.m` 改为 `GreenJointCurrentLoopModelRef`。
- `build_green_joint_average_motor_twin_model.m` 改为 `GreenJointCurrentLoopModelRef`。
- `green_joint_average_motor_twin_interface.sldd` 通过 `addDataSource` 引用 `green_joint_current_loop_interface.sldd`。
- 删除 v1 twin 字典中重复生成的 GJ 类型、Bus 和电流环参数，避免双源定义。

验证命令：

```bash
matlab -batch "run('matlab-practice/green_joint_digital_twin/build_green_joint_current_loop_twin_model.m')"
matlab -batch "run('matlab-practice/green_joint_digital_twin/build_green_joint_average_motor_twin_model.m')"
matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_current_loop_twin_smoke_test.m')"
matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_average_motor_twin_smoke_test.m')"
matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_average_motor_square_wave_test.m')"
```

当前结果：

```text
v0 current-loop twin smoke test passed.
v1 average-motor twin smoke test passed.
1kHz current square-wave test passed.
```

## 2026-06-26：记录 green-joint MBD 主线纪律

目录：

```text
docs/model_development_standard.md
docs/digital_twin_architecture_plan.md
docs/ai_collaboration_rules.md
green_joint_digital_twin/README.md
```

目标：

```text
防止后续继续东写一个模型、西写一个模型。
明确重新造模型前必须通知用户；临时验证可以存在，但必须回归统一 MBD 主线。
```

结论：

```text
green-joint MBD 主线按 电机/驱动器/减速器 -> 电流环 -> 速度环 -> 位置环 -> 测试状态 逐层衍生。
速度环验证必须复用电流环和 plant wrapper。
位置环验证必须复用速度环、电流环和 plant wrapper。
临时 prototype/temporary/scratch 模型只能验证单一假设，不能成为第二套系统。
验证完成后，要么把结论迁回 ControllerWrapper/PlantWrapper/TestSupervisor/scenario catalog，
要么标记为历史/废弃/参考。
```

验证：

```text
本次只更新工程纪律文档，未触发 MATLAB/Simulink 模型更新。
```

## 2026-06-24：green-joint 速度环物理量纲初始设计

目录：

```text
green_joint_digital_twin/
```

目标：

```text
根据用户提供的转子转动惯量 0.034 kg*mm^2，建立速度环 PI 初始参数设计脚本。
注意：本条最初记录为 rotor-only，2026-06-26 已被“考虑减速比”的主线方案修正。
```

成果：

- 建立 [design_green_joint_speed_loop.m](/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/design_green_joint_speed_loop.m)
- 更新 [scenarios/README.md](/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/scenarios/README.md)
- 更新 [controller_wrapper/README.md](/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/controller_wrapper/README.md)
- 更新 [green_joint_digital_twin/README.md](/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/README.md)

验证命令：

```bash
matlab -batch "run('matlab-practice/green_joint_digital_twin/design_green_joint_speed_loop.m')"
```

已修正后的当前设计：

```text
J_motor = 0.034 kg*mm^2 = 3.4e-8 kg*m^2
gear_ratio = 183.35
J_speed_loop = 6.2339e-6 kg*m^2
Kt = 0.00517276217 N*m/A
Ts_speed = 500 us
selected bring-up bandwidth = 20 Hz
Kp_speed = 0.302884591 A/(rad/s)
Ki_speed = 19.0308001 A/rad
Kaw_speed = 125.663706 1/s
iq_limit_initial = 0.1 A
```

注意：

```text
SpeedPiStep 输入为输出端速度，输出为电机端 Iq。
速度环设计应使用 J_motor * gear_ratio + J_load_output / gear_ratio。
```

## 2026-06-24：green-joint 统一测试 Harness 中长期纪律更新

目录：

```text
docs/mbd_test_state_management_architecture.md
docs/digital_twin_architecture_plan.md
green_joint_digital_twin/README.md
```

目标：

```text
明确后续不按短期孤岛模型开发速度环/位置环测试，而是基于基础模块复用和统一 test harness。
```

当前结论：

```text
新增速度环测试时，不创建独立 speed_step_harness_copy_of_everything.slx。
先登记 speed_step_* scenario。
复用 SpeedPiStep -> GreenJointCurrentLoopStep -> DqToAbcDutyStep。
接入统一 GreenJointControllerWrapper、PlantWrapper、LoggerAssessment。
TestSupervisor / Test Sequence 只管理测试状态，不污染可交付 controller core。
```

验证：

```text
本次只更新长期工程文档，未触发 MATLAB/Simulink 模型更新。
```

## 2026-06-24：green-joint PiCorrectionGain 抗饱和验证

目录：

```text
green_joint_digital_twin/
```

目标：

```text
验证固件默认 gj_mbd_pi_correction_gain = 400.0f 是否适合作为电流环 back-calculation anti-windup 增益。
```

成果：

- 建立 [run_green_joint_pi_correction_gain_sweep.m](/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/run_green_joint_pi_correction_gain_sweep.m)
- 生成 `results/pi_correction_gain_sweep_kp1_ki20000.csv`
- 生成 `results/pi_correction_gain_sweep_kp1_ki20000.png`
- 更新 [green_joint_digital_twin/README.md](/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/README.md)

验证命令：

```bash
matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_pi_correction_gain_sweep.m')"
```

当前结论：

```text
PiCorrectionGain = 400 1/s
Ts = 50 us
Kaw * Ts = 0.02
anti-windup time constant ~= 2.5 ms

Kaw=400 在 4A -> 1.5A 饱和退出测试中：
  exit saturation ~= 3.255 ms
  settling ~= 3.425 ms

结论：400 是合理的安全默认值，不是危险的大值；如果硬件饱和退出太慢，优先试 800 或 1200。
```

## 2026-06-24：green-joint V1 物理模型 1kHz 电流方波测试

目录：

```text
green_joint_digital_twin/
```

目标：

```text
增加一个电流环测试场景，使用 V1 Average-Value Inverter + Surface Mount PMSM 物理模型，
输入 1kHz iq_ref 方波，作为后续硬件波形对齐的主线测试。
```

成果：

- 建立 [run_green_joint_average_motor_square_wave_test.m](/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/run_green_joint_average_motor_square_wave_test.m)
- 建立 [build_green_joint_average_motor_square_wave_harness.m](/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/build_green_joint_average_motor_square_wave_harness.m)
- 建立 [run_green_joint_average_motor_square_wave_harness_test.m](/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/run_green_joint_average_motor_square_wave_harness_test.m)
- 生成 [green_joint_average_motor_square_wave_harness.slx](/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/green_joint_average_motor_square_wave_harness.slx)
- 更新 [green_joint_digital_twin/README.md](/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/README.md)
- 补齐 [build_green_joint_average_motor_twin_model.m](/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/build_green_joint_average_motor_twin_model.m) 中的 V1 合并字典字段，使其匹配当前 `green_joint_current_loop_mbd` 的 9 字段输出 Bus 和 `VoltageModulationRatio`

默认场景：

```text
scenario: current_square_1khz_0p3A_average_motor_v1
iq_ref:   +/-0.3 A
period:   1 ms full period, 0.5 ms half-period
Ts:       50 us current loop, 5 us plant
Kp/Ki:    1.0 / 20000.0
plant:    V1 Average-Value Inverter + Surface Mount PMSM
```

验证命令：

```bash
matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_average_motor_square_wave_test.m')"
matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_average_motor_square_wave_harness_test.m')"
```

当前结果：

```text
iq positive peak       = 0.336826 A
iq negative peak       = -0.337194 A
iq peak-to-peak        = 0.67402 A
iq/ref p-p gain        = 1.12337
gain@1kHz fundamental  = 1.01339
lag@1kHz fundamental   = 55.2157 deg / 153.377 us
voltage_mag_norm max   = 0.143269
```

可视化 `.slx` harness 验证结果：

```text
model                  = green_joint_average_motor_square_wave_harness
iq positive peak       = 0.336826 A
iq negative peak       = -0.336827 A
iq/ref p-p gain        = 1.12275
voltage_mag_norm max   = 0.142212
```

手动运行入口：

```matlab
run('/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/setup_green_joint_current_loop_twin.m')
open_system('/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/green_joint_average_motor_square_wave_harness.slx')
sim('green_joint_average_motor_square_wave_harness')
```

输出文件：

```text
green_joint_digital_twin/results/current_square_1khz_0p3A_average_motor_v1.png
green_joint_digital_twin/results/current_square_1khz_0p3A_average_motor_v1.csv
```

## 2026-06-24：MBD 测试状态统一管理架构

目录：

```text
docs/mbd_test_state_management_architecture.md
```

目标：

```text
以中长期方案为主导，解决 matlab-practice 中大量 MBD 测试例子缺少统一状态切换和统一管理的问题。
```

成果：

- 建立 [mbd_test_state_management_architecture.md](/home/user/study/AI+MOTOR/matlab-practice/docs/mbd_test_state_management_architecture.md)
- 更新 [docs/README.md](/home/user/study/AI+MOTOR/matlab-practice/docs/README.md)
- 更新 [digital_twin_architecture_plan.md](/home/user/study/AI+MOTOR/matlab-practice/docs/digital_twin_architecture_plan.md)
- 更新 [model_development_standard.md](/home/user/study/AI+MOTOR/matlab-practice/docs/model_development_standard.md)

当前结论：

```text
控制器模型和测试模型分离。
产品状态机可以用 Stateflow 并按需 codegen。
测试状态切换放入 Test Harness/Test Sequence/Stateflow TestSupervisor。
大量零散 MBD 示例先分类为 MODULE/PLANT/SCENARIO/STUDY/LEGACY，再逐步收编到统一 harness。
```

验证：

```text
本次只更新文档，未触发 MATLAB/Simulink 模型更新。
```

## 2026-06-23：green-joint 接入已有平均电压电机模型 v1

目录：

```text
green_joint_digital_twin/
```

目标：

```text
根据 matlab-practice 文档和已有平均电压模型主线，
把 green-joint 电流环 twin 从简化 dq plant 升级为 Average-Value Inverter + Surface Mount PMSM。
```

成果：

- 建立 [sync_green_joint_current_loop_twin_parameters.m](/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/sync_green_joint_current_loop_twin_parameters.m)
- 建立 [build_green_joint_average_motor_twin_model.m](/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/build_green_joint_average_motor_twin_model.m)
- 建立 [run_green_joint_average_motor_twin_smoke_test.m](/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/run_green_joint_average_motor_twin_smoke_test.m)
- 生成 [green_joint_average_motor_twin_model.slx](/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/green_joint_average_motor_twin_model.slx)
- 生成 `green_joint_average_motor_twin_interface.sldd` 合并接口字典
- 更新 [green_joint_digital_twin/README.md](/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/README.md)
- 更新 [digital_twin_architecture_plan.md](/home/user/study/AI+MOTOR/matlab-practice/docs/digital_twin_architecture_plan.md)

模型结构：

```text
GreenJointCurrentLoopStep
  -> DqToAbcDutyStep
  -> phase_duty_t [0, 1]
  -> Average-Value Inverter
  -> Surface Mount PMSM
  -> MotorClarkeParkStep
  -> id_fbk / iq_fbk
```

复用来源：

```text
GreenJointCurrentLoopStep: green_joint_current_loop_mbd/
DqToAbcDutyStep:          motor_float_open_loop_mbd/
MotorClarkeParkStep:      motor_clarke_park_struct/
Average-Value Inverter:   mcbplantlib
Surface Mount PMSM:       mcblib
```

验证命令：

```bash
GJ_DT_REBUILD_BEFORE_TEST=1 matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_average_motor_twin_smoke_test.m')"
```

当前结果：

```text
iq_ref final          = 3 A
id final              = 0.000295401 A
iq final              = 0.557888 A
vd final              = -0.249145 V
vq final              = 6.91952 V
wm final              = 47.9616 rad/s
voltage_mag_norm max  = 1
Green-joint average motor twin smoke test passed.
```

注意：

```text
v1 是自由转子平均电压电机模型，q 轴电流阶跃会产生转矩并加速电机。
因此 v1 当前验证的是平均电压模型链路可运行，不是严格锁轴电流环带宽测试。
默认母线电压已按 green-joint 12V 系统设置。
电机参数已按用户提供的线间 R=2ohm、L=55uH 设置；相参数为 Rs=1ohm、Ld=Lq=27.5uH。
当前 3A q 轴阶跃会触发电压上限。
```

## 2026-06-23：green-joint 平均电压电流环数字孪生 v0

目录：

```text
green_joint_digital_twin/
```

目标：

```text
使用已经实现的 green_joint_current_loop_mbd 模块，
把 green-joint 打造成完整的电流环模式，并先使用 dq 平均电压模型闭环验证。
```

成果：

- 建立 [setup_green_joint_current_loop_twin.m](/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/setup_green_joint_current_loop_twin.m)
- 建立 [build_green_joint_current_loop_twin_model.m](/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/build_green_joint_current_loop_twin_model.m)
- 建立 [run_green_joint_current_loop_twin_smoke_test.m](/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/run_green_joint_current_loop_twin_smoke_test.m)
- 生成 [green_joint_current_loop_twin_model.slx](/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/green_joint_current_loop_twin_model.slx)
- 更新 [green_joint_digital_twin/README.md](/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/README.md)
- 更新 [digital_twin_architecture_plan.md](/home/user/study/AI+MOTOR/matlab-practice/docs/digital_twin_architecture_plan.md)

模型结构：

```text
id_ref = 0 A
iq_ref = 0 -> 3 A step
vbus = 12 V
  -> green_joint_current_loop_input_t
  -> GreenJointCurrentLoopStep
  -> vd_cmd / vq_cmd
  -> dq average plant
  -> id_fbk / iq_fbk
```

平均 plant：

```text
did/dt = (vd - Rs * id) / Ld
diq/dt = (vq - Rs * iq) / Lq
```

当前边界：

```text
先只验证电流 PI、Vd 优先限幅、anti-windup 和物理量纲参数。
暂不接 Clarke/Park、inverse Park、SVPWM、开关级逆变器、ADC/PWM 延迟、速度环。
```

验证命令：

```bash
matlab -batch "run('matlab-practice/green_joint_digital_twin/run_green_joint_current_loop_twin_smoke_test.m')"
```

当前结果：

```text
iq_ref final          = 3 A
iq final              = 3 A
iq tracking error     = 2.38419e-07 A
id final              = 0 A
vd final              = 0 V
vq final              = 3 V
voltage_mag_norm max  = 0.433276
Green-joint current-loop digital twin smoke test passed.
```

## 2026-06-23：green-joint 数字孪生资产盘点与路线建立

目录：

```text
docs/digital_twin_architecture_plan.md
green_joint_digital_twin/
```

目标：

```text
查看当前目录中已经存在的 MBD 模块和电机模型，
确定如何组合成 green-joint 数字孪生，而不是重新从零搭建。
```

成果：

- 建立 [digital_twin_architecture_plan.md](/home/user/study/AI+MOTOR/matlab-practice/docs/digital_twin_architecture_plan.md)
- 建立 [green_joint_digital_twin/README.md](/home/user/study/AI+MOTOR/matlab-practice/green_joint_digital_twin/README.md)
- 更新 [docs/README.md](/home/user/study/AI+MOTOR/matlab-practice/docs/README.md)，把数字孪生路线加入阅读顺序和当前焦点

已确认可复用资产：

- MBD 控制器 core：`green_joint_current_loop_mbd/`
- 通用控制模块库：`motor_control_modules/`
- 平均电压闭环参考：`motor_current_loop_mbd/`、`motor_speed_current_loop_mbd/`、`motor_float_open_loop_mbd/`
- 研究 plant：`average-inverter/`
- 开关级专项验证：`average-inverter/switching_sampling_study/`
- 电流滤波：`motor_current_filter_mbd/`
- Clarke/Park：`motor_clarke_park_struct/`
- 死区补偿和采样窗口：`pwm_deadtime_compensation_mbd/`、`pwm_deadtime_sampling_mbd/`
- 参数辨识：`identification/pmsm_electrical_parameter_identification/`
- ADC/PWM 时序：`adc_interrupt_current_loop_test/`

当前结论：

```text
目录中已经具备数字孪生的主要积木：
MBD controller core、平均电压 plant、开关级专项验证、参数辨识和部分真实数据分析入口。
缺口是统一的 green_joint_digital_twin harness 和硬件日志对齐流程。
```

推荐 v0：

```text
GreenJointCurrentLoopStep
  + 简化 dq 平均 plant
  + id/iq/vd/vq/voltage_mag_norm 对比
```

暂不做：

```text
完整 FOC 替换、开关 MOSFET 级模型、ADC/PWM 寄存器、死区补偿、速度环、齿槽补偿。
```

验证：

```text
本次为资产盘点和文档路线建立，未触发 MATLAB/Simulink 模型更新。
```

## 2026-06-23：Simulink 模型更新卡死排查与安全重建规则

目录：

```text
docs/simulink_hang_troubleshooting.md
green_joint_current_loop_mbd/
```

目标：

```text
记录 green_joint_current_loop_mbd 在 MATLAB Desktop / Simulink 中卡在“模型更新”的排查结论，
并把安全重建 .slx/.sldd 的规则沉淀给后续 AI。
```

成果：

- 建立 [simulink_hang_troubleshooting.md](/home/user/study/AI+MOTOR/matlab-practice/docs/simulink_hang_troubleshooting.md)
- 为 [build_green_joint_current_loop_model.m](/home/user/study/AI+MOTOR/matlab-practice/green_joint_current_loop_mbd/build_green_joint_current_loop_model.m) 增加安全重建守卫
- 为 [generate_green_joint_current_loop_dictionary.m](/home/user/study/AI+MOTOR/matlab-practice/green_joint_current_loop_mbd/generate_green_joint_current_loop_dictionary.m) 增加安全重建守卫
- 新增 [assert_green_joint_safe_rebuild_environment.m](/home/user/study/AI+MOTOR/matlab-practice/green_joint_current_loop_mbd/assert_green_joint_safe_rebuild_environment.m)
- 将卡死排查文档加入 [docs/README.md](/home/user/study/AI+MOTOR/matlab-practice/docs/README.md) 阅读顺序
- 将安全重建规则加入 [model_development_standard.md](/home/user/study/AI+MOTOR/matlab-practice/docs/model_development_standard.md) 和 [ai_collaboration_rules.md](/home/user/study/AI+MOTOR/matlab-practice/docs/ai_collaboration_rules.md)

当前结论：

```text
本次卡死更像是 MATLAB Desktop / Simulink UI、文件占用、模型缓存和脚本重建流程冲突，
不是 green_joint_current_loop_mbd 的 PI 算法代数环或求解器死循环。
```

高风险模式：

```text
Desktop 正打开模型或数据字典
  + batch/build 脚本删除并重建同名 .slx/.sldd
  + 脚本自动 arrangeSystem 或 SimulationCommand='update'
  -> Simulink 可能长时间卡在模型更新
```

已经执行的工程约束：

- build 脚本不再自动 `SimulationCommand='update'`。
- build 脚本不再自动 `Simulink.BlockDiagram.arrangeSystem(...)`。
- 重建 `.slx/.sldd` 前检查其它 MATLAB 进程是否占用生成产物。
- 默认不允许从 MATLAB Desktop 运行会删除/重建生成模型的数据字典脚本。

后续 AI 必须遵守：

```text
需要重建模型/字典时，优先使用 matlab -batch。
不要在 Desktop 打开同名 .slx/.sldd 时重建。
不要为了“看一下”反复 open/update 模型。
如果只是文档或接口梳理，不要触发 Simulink 模型更新。
```

验证命令：

```bash
matlab -batch "run('matlab-practice/green_joint_current_loop_mbd/run_green_joint_current_loop_smoke_test.m')"
matlab -batch "run('matlab-practice/green_joint_current_loop_mbd/generate_green_joint_current_loop_code.m')"
```

当前结果：

```text
前序验证显示 smoke test 和 ERT codegen 可通过。
本次文档化只记录规则，不再额外触发 Simulink 更新。
```

## 2026-06-22：green-joint 电流 PI MBD 替换工程启动

目录：

```text
green_joint_current_loop_mbd/
```

目标：

```text
把 green-joint 当前电流环中的 d/q PI 抽成 MBD 模块，
用 A、V、s 的物理量纲接口替代无量纲经验参数，
先只输出物理 Vd/Vq 和单位圆归一化 Vd/Vq，后级 inverse Park/SVPWM 仍由原模块处理。
```

成果：

- 建立 [current_loop_mbd_replacement_plan.md](/home/user/study/AI+MOTOR/green-joint/docs/current_loop_mbd_replacement_plan.md)
- 建立 [green_joint_current_loop_mbd/README.md](/home/user/study/AI+MOTOR/matlab-practice/green_joint_current_loop_mbd/README.md)
- 建立 [interface.yaml](/home/user/study/AI+MOTOR/matlab-practice/green_joint_current_loop_mbd/interface.yaml)
- 建立 [interface.json](/home/user/study/AI+MOTOR/matlab-practice/green_joint_current_loop_mbd/interface.json)
- 建立 [generate_green_joint_current_loop_dictionary.m](/home/user/study/AI+MOTOR/matlab-practice/green_joint_current_loop_mbd/generate_green_joint_current_loop_dictionary.m)
- 建立 [build_green_joint_current_loop_model.m](/home/user/study/AI+MOTOR/matlab-practice/green_joint_current_loop_mbd/build_green_joint_current_loop_model.m)
- 建立 [run_green_joint_current_loop_smoke_test.m](/home/user/study/AI+MOTOR/matlab-practice/green_joint_current_loop_mbd/run_green_joint_current_loop_smoke_test.m)
- 建立 [generate_green_joint_current_loop_code.m](/home/user/study/AI+MOTOR/matlab-practice/green_joint_current_loop_mbd/generate_green_joint_current_loop_code.m)
- 建立 [verify_green_joint_current_pi_codegen.m](/home/user/study/AI+MOTOR/matlab-practice/green_joint_current_loop_mbd/verify_green_joint_current_pi_codegen.m)

第一阶段 MBD 边界：

- 输入：`id_ref`、`iq_ref`、`id_fbk`、`iq_fbk`、`vbus`
- 输入单位：电流使用 A，电压使用 V
- 输出：`vd_cmd`、`vq_cmd`、`voltage_mag`、`vd_norm`、`vq_norm`、`voltage_mag_norm`
- MBD 只做 d/q PI、Vd 优先电压分配、单位圆标准化
- 平台层继续负责 ADC、Clarke/Park、电流滤波、inverse Park、SVPWM、TIM/PWM 寄存器、fault/state machine

当前状态：

- 已完成固件事实抽取和替换边界定义
- 已完成第一版用户接口合同
- 已完成 `interface.json -> .sldd` 生成器
- 已完成 `GreenJointCurrentLoopStep` 模型生成
- 已将边界收窄为电流 PI-only，避免一次替换过多
- 已实现 d/q PI 积分状态、Vd 优先电压分配、单位圆归一化
- 已增加 codegen 守卫，防止误改回等比例限幅或断开 back-calculation
- 已确认 ERT reusable codegen 通过

验证命令：

```bash
matlab -batch "run('matlab-practice/green_joint_current_loop_mbd/run_green_joint_current_loop_smoke_test.m')"
matlab -batch "run('matlab-practice/green_joint_current_loop_mbd/generate_green_joint_current_loop_code.m')"
```

当前结果：

```text
Green-joint current-loop MBD smoke test passed.
Successful completion of code generation for: green_joint_current_loop_model
Green-joint current PI codegen verification passed.
```

生成接口状态：

```text
GreenJointCurrentLoopStep_Init(...)
GreenJointCurrentLoopStep(
  const green_joint_current_loop_input_t *rtu_loop_in,
  green_joint_current_loop_output_t *rty_loop_out,
  DW_GreenJointCurrentLoopStep_T *localDW,
  P_GreenJointCurrentLoopStep_T *localP)
```

注意：

```text
由于 d/q PI 积分器已经进入 MBD，固件 adapter 需要长期保存 localDW 状态。
生成 C 已检查：先 clamp Vd，再用 sqrt(Vlimit^2 - Vd^2) 限制 Vq，并包含 PiCorrectionGain 反算抗饱和。
```

## 2026-06-22：MBD 用户接口合同纪律

目录：

```text
docs/mbd_interface_contract_standard.md
```

目标：

```text
把用户可编辑接口从 build 脚本和 .sldd 中抽离出来，
形成 interface.yaml/json -> .sldd -> Simulink -> generated C headers 的长期流程。
```

成果：

- 建立 [mbd_interface_contract_standard.md](/home/user/study/AI+MOTOR/matlab-practice/docs/mbd_interface_contract_standard.md)
- 更新 [README.md](/home/user/study/AI+MOTOR/matlab-practice/docs/README.md) 的阅读顺序
- 更新 [model_development_standard.md](/home/user/study/AI+MOTOR/matlab-practice/docs/model_development_standard.md) 的标准目录和数据字典规则
- 更新 [ai_collaboration_rules.md](/home/user/study/AI+MOTOR/matlab-practice/docs/ai_collaboration_rules.md) 的默认开发路线
- 更新 [mbd_future_direction.md](/home/user/study/AI+MOTOR/matlab-practice/docs/mbd_future_direction.md) 的未来方向

当前结论：

```text
interface.yaml = 用户接口合同
.sldd          = Simulink/Embedded Coder 数据字典
.m            = 生成器和模型构建工具
generated .h   = C 工程交付接口
```

后续新 MBD 模块优先按这个流程建立接口。已有 `.m` 手写 Bus/Parameter 的模块标记为
`[TRANSITION]`，发生实质接口修改时优先补 `interface.yaml`。

## 2026-06-17：green-joint 电流滤波 MBD 化起步

目录：

```text
motor_current_filter_mbd/
```

目标：

```text
把 green-joint 当前的 dq 电流反馈低通整理成独立的 MBD 模块，
为后续代码生成和接口合同管理做准备。
```

成果：

- 建立 [current_loop_filter_mbd_plan.md](/home/user/study/AI+MOTOR/green-joint/docs/current_loop_filter_mbd_plan.md)
- 建立 [motor_current_filter_mbd/README.md](/home/user/study/AI+MOTOR/matlab-practice/motor_current_filter_mbd/README.md)
- 建立 [build_current_filter_model.m](/home/user/study/AI+MOTOR/matlab-practice/motor_current_filter_mbd/build_current_filter_model.m)
- 建立 [run_current_filter_smoke_test.m](/home/user/study/AI+MOTOR/matlab-practice/motor_current_filter_mbd/run_current_filter_smoke_test.m)

模块边界：

- 输入：`id_raw`、`iq_raw`、`v_mag_norm`
- 输出：`id_f`、`iq_f`、`alpha`
- 先只做 controller-side dq 低通，不把 platform-side 扇区融合混进来

当前状态：

- MATLAB smoke test 已通过
- ERT 代码生成已通过
- 生成 `CurrentFilterStep()` 可复用函数接口，输入/输出结构体来自 `.sldd` 的 Bus 合同

验证命令：

```bash
matlab -batch "run('matlab-practice/motor_current_filter_mbd/run_current_filter_smoke_test.m')"
```

当前结果：

```text
low-v alpha = 0.95
mid-v alpha = 0.78125
mid-v id_f  = 0 A
mid-v iq_f  = 10 A
Current filter smoke test passed.
```

代码生成命令：

```bash
matlab -batch "cd('matlab-practice/motor_current_filter_mbd'); run('build_current_filter_model.m'); slbuild('current_filter_model');"
```

代码生成结果：

```text
Successful completion of code generation for: current_filter_model
```

## 2026-06-15：当前 MBD 主线梳理与 legacy 标识

目录：

```text
docs/current_mbd_landscape.md
docs/README.md
docs/ai_collaboration_rules.md
```

目标：

```text
把当前工程的 MBD 主线、研究线、旧技术线和产品化断点梳理清楚，
方便后续 AI 接手时先判断目录角色，再决定修改方式。
```

成果：

- 建立当前 MBD 现状地图。
- 为后续 AI 定义统一标签：

```text
[KEEP]
[TRANSITION]
[LEGACY]
[RESEARCH]
[PRODUCT GAP]
```

- 明确 `motor_*_mbd/` 和 `motor_control_modules/` 是现代可交付主线。
- 明确 `average-inverter/` 是研究平台，不是新的嵌入式交付模板。
- 明确 `dexterous_hand_impedance_plan/`、`主动阻尼控制/` 等旧路径属于 legacy。
- 历史记录：当时 `green-joint/` 仍以手写固件为主。当前主线已推进到
  电流环、速度环、速度 PLL、MIT 由 MBD 生成代码和 adapter 接管，位置环/状态机仍需继续闭环。

使用方式：

- 新 AI 进入仓库先读：

```text
docs/current_mbd_landscape.md
```

- 再决定后续任务应走：

```text
现代 MBD 模块扩展
or 研究验证
or 历史兼容维护
or 固件集成补链
```

## 2026-06-04：现代 MBD 类型与可重入代码主线

目录：

```text
simple_add_reentrant/
```

目标：

```text
z = x + y
```

成果：

- 使用 `.sldd` 管理接口类型。
- 使用 `Simulink.AliasType` 生成稳定 C typedef。
- 生成 ERT reusable/reentrant 风格接口。
- 确认新项目不再以 MPT 为主线，MPT 只作为 legacy 知识。

客户修改类型入口：

```text
simple_add_reentrant/build_simple_add_reentrant_model.m
customer_interface_config()
```

## 2026-06-04：motor_t 结构体与 Clarke/Park 变换

目录：

```text
motor_clarke_park_struct/
```

目标：

```text
motor_t { ia, ib, ic, theta_e }
  -> Clarke/Park
motor_dq_t { i_alpha, i_beta, id, iq }
```

成果：

- 使用 `.sldd + Simulink.Bus` 定义结构体接口。
- 使用 `Simulink.NumericType` 定义定点电流类型。
- 建立功能测试 harness，可观察输入输出波形。
- 证明 `.sldd` 对结构体接口和定点类型长期维护很有价值。

验证命令：

```bash
matlab -batch "run('motor_clarke_park_struct/run_motor_clarke_park_function_test.m')"
```

当前结果：

```text
Maximum error: 0.000244
Motor Clarke/Park functional test passed.
```

## 2026-06-04：float-first 电机开环仿真架构

目录：

```text
motor_float_open_loop_mbd/
```

目标：

```text
open_loop_cmd_t
  -> DqToAbcDutyStep
  -> phase_duty_t
  -> Average-Value Inverter
  -> Surface Mount PMSM
  -> plant_feedback_t
```

成果：

- 建立 float-first 的电机开环 MBD 架构。
- 控制器侧接口使用 `.sldd + AliasType('single')`。
- Average-Value Inverter 输入 duty 明确为 `[0, 1]`。
- plant 边界允许显式转 double，控制器合同仍保持 single。
- 25 us plant/PWM tick 使用 Rate Transition 表达。

验证命令：

```bash
matlab -batch "run('motor_float_open_loop_mbd/run_open_loop_smoke_test.m')"
```

当前结果：

```text
wm      = 26.8943 rad/s
theta_e = 26.2228 rad
ia      = 4.46792 A
ib      = 8.83261 A
ic      = -13.3005 A
Open-loop smoke test passed.
```

## 2026-06-05：电流环 PI 框图模块

目录：

```text
motor_current_pi_mbd/
```

目标：

```text
current_pi_input_t -> CurrentPiStep -> current_pi_output_t
```

成果：

- 不使用 MATLAB Function block。
- 不使用黑盒 PID Controller block。
- 用基础 Simulink block 显式实现 P、I、动态电压限幅、back-calculation anti-windup。
- 参数放入 `.sldd`，如 `Kp_id/Ki_id/Kaw_id/Kp_iq/Ki_iq/Kaw_iq/VLimitRatio`。

当前公式：

```text
error = ref - meas
u_pre_sat = Kp * error + integrator
u_sat = clamp(u_pre_sat, -0.577 * vdc, 0.577 * vdc)
integrator_next = integrator + Ts * (Ki * error + Kaw * (u_sat - u_pre_sat))
```

验证命令：

```bash
matlab -batch "run('motor_current_pi_mbd/run_current_pi_smoke_test.m')"
```

当前结果：

```text
vd_ref range = [0, 0] V
vq_ref range = [13.3204, 39.236] V
v_limit      = 39.236 V
Current PI smoke test passed.
```

## 2026-06-06：速度环、电流环与电机集成

目录：

```text
motor_current_loop_mbd/
motor_speed_pi_mbd/
motor_speed_current_loop_mbd/
```

成果：

- 增加速度环 PI。
- 建立速度环、电流环和电机 plant 的集成仿真。
- 明确模块化方向：速度环、电流环、变换、SVPWM/duty 输出、plant/adapter 分层。
- 形成“算法模块平台无关，寄存器配置在平台适配层”的边界。

后续要求：

- 集成模型要记录各环采样时间。
- 电压饱和和 anti-windup 要单独做回归测试。
- 速度环、电流环输出波形要保存到 `results/` 或 `reports/`。

## 2026-06-07：可复用控制模块库

目录：

```text
motor_control_modules/
```

成果：

- 建立 team-facing shared dictionary：

```text
motor_control_modules/motor_control_interface.sldd
```

- 建立 Simulink library：

```text
motor_control_modules/motor_control_lib.slx
```

- 当前模块：

```text
SpeedPiStep
CurrentPiStep
DqToAbcDutyStep
MotorClarkeParkStep
OpenLoopCommand
```

构建命令：

```bash
matlab -batch "run('motor_control_modules/setup_motor_control_modules.m'); run('motor_control_modules/build_motor_control_interface_dictionary.m'); run('motor_control_modules/build_motor_control_module_library.m')"
```

基线调度：

```text
25us PWM tick
50us current loop
100us speed loop
```

## 2026-06-10：12V 空心杯小惯量辨识

目录：

```text
identification/coreless_motor_12v_identification/
```

成果：

- 建立小惯量辨识的 MATLAB sandbox。
- 合成数据验证双向 torque pulse、位置拟合、摩擦项联合估计。
- 形成结论：不要直接用 `J = Te / diff(speed)` 作为主方法。

验证命令：

```bash
matlab -batch "run('identification/coreless_motor_12v_identification/run_coreless_motor_identification_demo.m')"
```

当前结果：

```text
J true / estimate = 1.2e-06 / 1.214e-06 kg*m^2
J error           = 1.17%
```

输出：

```text
identification/coreless_motor_12v_identification/results/coreless_identification_timeseries.png
identification/coreless_motor_12v_identification/results/coreless_identification_windows.png
```

## 2026-06-10：电机性能检测与电流传感器谐波回归

目录：

```text
motor_performance_characterization/
```

成果：

- 建立电流 offset、gain mismatch、噪声对 dq 电流谐波影响的回归测试。
- 形成性能检测清单：电流采样、编码器非线性、转矩波动、MTPA、弱磁、台架标定等。

验证命令：

```bash
matlab -batch "run('motor_performance_characterization/run_current_sensor_harmonic_regression_test.m')"
```

当前结论：

```text
current offset / zero drift -> 1x electrical ripple in id/iq
current gain mismatch       -> 2x electrical ripple in id/iq
random noise                -> broadband
PWM ripple                  -> PWM frequency and sidebands
deadtime voltage error      -> often 6x electrical torque ripple
```

输出：

```text
motor_performance_characterization/results/current_sensor_harmonic_metrics.csv
motor_performance_characterization/results/current_sensor_harmonic_regression.png
motor_performance_characterization/results/current_sensor_fault_dq_timeseries.png
```

## 2026-06-10：PMSM 电参辨识与编码器对齐 sandbox

目录：

```text
identification/pmsm_electrical_parameter_identification/
```

目标：

```text
Rs / Ld / Lq / psi_f
encoder electrical offset
encoder residual 1x / 2x
```

成果：

- standstill d/q 电压阶跃估计 `Rs/Ld/Lq`。
- 旋转反电势估计 `psi_f`。
- sensorless/sensored angle 差值估计 encoder offset 和 1x/2x 残差。
- 创建 Simulink 波形观察模型。

验证命令：

```bash
matlab -batch "run('identification/pmsm_electrical_parameter_identification/run_pmsm_electrical_id_demo.m')"
matlab -batch "run('identification/pmsm_electrical_parameter_identification/build_pmsm_electrical_id_waveform_model.m')"
```

当前结果：

```text
Rs true / estimate   : 0.42 / 0.419958 ohm
Ld true / estimate   : 0.00025 / 0.000251283 H
Lq true / estimate   : 0.00036 / 0.000359821 H
psi true / estimate  : 0.018 / 0.0179997 Wb
encoder offset error : 1.17513e-05 rad
```

测试条件记录：

```text
identification/pmsm_electrical_parameter_identification/results/pmsm_electrical_id_test_conditions.txt
```

重要经验：

```text
不要把 noisy di/dt 回归作为电感辨识第一版。
standstill voltage step 更适合先用指数响应拟合：
  i(t) = Iinf * (1 - exp(-t/tau))
  R = Vinf / Iinf
  L = R * tau
```

## 2026-06-10：高占空比死区采样窗口 MBD 模块

旧研究目录：

```text
average-inverter/switching_sampling_study/
```

新增 MBD 目录：

```text
pwm_deadtime_sampling_mbd/
```

背景：

```text
R = 4 ohm
L = 100 uH
PWM = 20 kHz
deadtime = 500 ns
```

复现结果：

```text
modulation ratio = 0.9
RL sample error RMS = 0.302540 A
sampled phase ripple pk-pk = 3.348031 A
```

MBD 模块目标：

```text
pwm_phase_duty_t
  -> DeadtimeSamplingWindowStep
  -> pwm_sampling_status_t
```

核心公式：

```text
T_low_ideal = max((1 - duty) * T_pwm, 0)
T_usable    = max(T_low_ideal - 2 * dead_time - adc_settle_time, 0)
valid       = T_usable >= min_valid_window
```

功能测试命令：

```bash
matlab -batch "run('pwm_deadtime_sampling_mbd/run_pwm_deadtime_sampling_window_test.m')"
```

当前测试结果：

```text
usable_low = [45.500 0.500 23.000] us
sample_valid = [1 0 1]
all_samples_valid = 0
```

C 代码生成命令：

```bash
matlab -batch "run('pwm_deadtime_sampling_mbd/generate_pwm_deadtime_sampling_code.m')"
```

当前生成接口：

```c
extern void DeadtimeSamplingWindowStep(const pwm_phase_duty_t *rtu_duty_in,
  pwm_sampling_status_t *rty_status_out);
```

修正边界：

```text
DeadtimeSamplingWindowStep 不是开关级死区物理仿真。
它只是可生成 C 的采样窗口 valid 判定模块。
真正观察死区造成的电流纹波/偏移，需要使用开关 MOS + 电机 plant。
```

已补开关级 smoke test：

```text
average-inverter/switching_sampling_study/run_switching_deadtime_motor_smoke_test.m
```

该测试构建：

```text
center-aligned PWM + deadtime gates
  -> Universal Bridge MOSFET/Diodes
  -> SPS Permanent Magnet Synchronous Machine
  -> phase current logging
```

当前开关级结果：

```text
Vdc = 12 V
PWM = 20 kHz
deadtime = 500 ns
R = 4 ohm
L = 100 uH
deadtime compensation update = 50 us
deadtime compensation duty = 0.01000
deadtime compensation current source = dq_synthesized
deadtime compensation id/iq = [0.0 0.2] A
deadtime compensation current_zero/current_full = [0.02 0.10] A
deadtime compensation polarity = -1
ia/ib/ic pk-pk = [0.2731 1.1862 1.1045] A
sum current RMS = 0 A
```

极性校准历史记录：

```text
no compensation max phase pk-pk ~= 1.2217 A
polarity = +1 max phase pk-pk ~= 1.2441 A
polarity = -1 max phase pk-pk ~= 1.1993 A
```

这组三行是早期 full-step deadband 补偿下的极性校准基线；当前补偿算法已经改成
`current_zero/current_full` 平滑补偿。

结论：

```text
高占空比时，某相低边窗口可能被 deadtime + ADC settle 压缩到无效。
这个判断应该作为 current_valid/ADC adapter 的上游保护信号，而不是让电流环或无感观测器直接相信所有采样点。
```

## 2026-06-10：50us 死区 duty 补偿 MBD 模块

目录：

```text
pwm_deadtime_compensation_mbd/
```

目标：

```text
pwm_deadtime_comp_input_t
  -> DeadtimeCompensationStep
  -> pwm_deadtime_comp_output_t
```

成果：

- 把死区补偿策略做成用户级 MBD 算法 core。
- 使用 `.sldd + AliasType + Bus` 固定接口合同。
- 使用基础 Simulink blocks 先由 `id/iq/sin_theta_e/cos_theta_e` 合成三相电流，再实现 `abs/compare/switch/product/saturation`，不依赖手写 C。
- 生成 ERT reusable/reentrant C 接口。
- 生成 `pwm_deadtime_compensation_lib.slx` 作为模块包内的独立库产物。
- 同步把 `DeadtimeCompensationStep` 放入团队总库 `motor_control_modules/motor_control_lib.slx`。
- 开关级 `average-inverter/switching_sampling_study` 继续作为 MOS + PMSM 物理验证 harness；它插入 `motor_control_lib.slx/DeadtimeCompensationStep`，只保留 bus adapter、类型转换和 plant 验证，不复制算法本体。

算法：

```text
ia_synth = id*cos(theta_e) - iq*sin(theta_e)
i_beta   = id*sin(theta_e) + iq*cos(theta_e)
ib_synth = -0.5*ia_synth + sqrt(3)/2*i_beta
ic_synth = -0.5*ia_synth - sqrt(3)/2*i_beta

gain_x   = clamp((abs(i_synth_x) - current_zero) / (current_full - current_zero), 0, 1)
active_x = enable && gain_x > 0
sign_x   = +1 when i_synth_x > 0, else -1
comp_x   = polarity * sign_x * comp_duty * gain_x when active_x else 0
d_x_out  = clamp(d_x + comp_x, 0, 1)
```

默认配置：

```text
sample time = 50 us
DeadtimeCompDuty = 0.01000
DeadtimeCompCurrentZero_A = 0.02 A
DeadtimeCompCurrentFull_A = 0.10 A
DeadtimeCompCurrentInvRange_1perA = 12.5 1/A
DeadtimeCompPolarity = -1
```

功能测试命令：

```bash
matlab -batch "run('pwm_deadtime_compensation_mbd/run_pwm_deadtime_compensation_test.m')"
```

当前结果：

```text
input dq = [id=0.0 iq=0.2] A, theta_e = 0 rad
synth current = [0.0 0.1732 -0.1732] A
duty_out = [0.05000 0.94000 0.51000]
comp = [-0.00000 -0.01000 0.01000]
active = [0 1 1]
```

C 代码生成命令：

```bash
matlab -batch "run('pwm_deadtime_compensation_mbd/generate_pwm_deadtime_compensation_code.m')"
```

当前生成接口：

```c
extern void DeadtimeCompensationStep(const pwm_deadtime_comp_input_t
  *rtu_comp_in, pwm_deadtime_comp_output_t *rty_comp_out);
```

开关级验证：

```bash
matlab -batch "cd('average-inverter/switching_sampling_study'); set(0,'DefaultFigureVisible','off'); run_switching_deadtime_motor_smoke_test"
```

当前结果：

```text
deadtime compensation: enable=1, duty=0.01000, update=50.00 us
current source = dq_synthesized
id/iq = [0.0 0.2] A
current_zero/current_full = [0.02 0.10] A
ia/ib/ic pk-pk = [0.2731 1.1862 1.1045] A
deadtime comp range = a[-0 9.04e-06], b[-0.01 -0.01], c[0.01 0.01]
sum current RMS = 0 A
```

边界：

```text
DeadtimeCompensationStep 是可交付算法模块。
DeadtimeSamplingWindowStep 是采样窗口 valid 判定模块。
Universal Bridge + PMSM 是物理开关级验证 harness。
```

## 2026-06-11：死区补偿应用到开关型电机验证

目录：

```text
average-inverter/switching_sampling_study/
pwm_deadtime_compensation_mbd/
```

本次修正：

- `pwm_deadtime_compensation_mbd/` 的 MBD core 使用 `id/iq/sin_theta_e/cos_theta_e` 合成相电流判极性。
- `pwm_deadtime_compensation_mbd/build_pwm_deadtime_compensation_library.m` 生成模块包内的 `pwm_deadtime_compensation_lib.slx`。
- `motor_control_modules/build_motor_control_module_library.m` 将 `DeadtimeCompensationStep` 放入 `motor_control_lib.slx`。
- 开关型 `Universal Bridge MOSFET/Diodes + SPS PMSM` smoke test 插入 `motor_control_lib.slx/DeadtimeCompensationStep`。
- `build_switching_sampling_study_model.m` 只做接口适配：`theta_e_deg -> sin/cos`、标量信号转 `pwm_deadtime_comp_input_t`、输出 bus 转 double。
- plant 真实 `ia/ib/ic` 只用于验证和波形分析，不作为补偿极性输入。

验证命令：

```bash
matlab -batch "cd('average-inverter/switching_sampling_study'); set(0,'DefaultFigureVisible','off'); run_switching_deadtime_motor_smoke_test"
```

当前结果：

```text
deadtime_comp_current_source = dq_synthesized
deadtime_comp_id_A = 0
deadtime_comp_iq_A = 0.2
ia/ib/ic pk-pk = [0.2731 1.1862 1.1045] A
deadtime_comp_range = a[-0 9.04e-06], b[-0.01 -0.01], c[0.01 0.01]
Result: PASS
```

继承规则：

```text
MBD core 的输入语义变更后，必须同步检查开关级验证 harness。
不能只让可生成 C 的模块通过测试，却让物理验证模型仍使用旧信号路径。
验证 harness 可以写 adapter，但不能复制算法内部实现；优先插入库模块。
```

## 2026-06-11：模型选择与 MBD 边界指南

目录：

```text
docs/modeling_scope_decision_guide.md
```

目标：

```text
记录什么时候使用平均电压模型，什么时候使用开关型模型；
记录什么模块按 MBD/codegen 标准沉淀，什么模块保持原有仿真/研究形态。
```

成果：

- 新增模型选择专题指南。
- 在 `docs/README.md` 增加阅读入口。
- 在 `docs/model_development_standard.md` 增加简化模型选择规则。
- 在 `MBD_DEVELOPMENT_NOTES.md` 追加长期工程判断。

关键结论：

```text
平均电压模型：
  控制算法主线、长时间闭环仿真、PI/速度环/MTPA/弱磁初步验证、
  MBD 模块集成和可生成 C 的算法接口检查。

开关型模型：
  PWM 边沿、死区、采样窗口、电流纹波、MOSFET/Diode 导通、
  共模/零相量分配/DPWM 对采样条件影响的专项验证。

MBD/codegen：
  可交付给固件或同事复用的算法 core 和稳定 adapter。

原有仿真/脚本：
  物理 plant、Simscape/SPS、波形观察 harness、参数扫描、CSV 分析和报告。
```

推荐工作流：

```text
平均模型先跑通控制。
开关模型专项验证 PWM/ADC/死区问题。
把验证得到的补偿/判定逻辑沉淀成 MBD 模块。
不要把开关级 plant 强行 codegen。
```

## 下一个方向

- 用真实 CSV 接入 PMSM 电参辨识。
- 把电机性能检测做成完整回归测试套件。
- 建立 `Ld(id,iq)`、`Lq(id,iq)`、`psi_f(T)`、`Rs(T)` 的标定数据结构。
- 将 sensorless observer 与 encoder angle 对齐做成 MBD 模块。
- 对共享模块库建立版本号、release checklist 和 codegen API 检查。
