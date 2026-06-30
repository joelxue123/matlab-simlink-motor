# MBD Test State Management Architecture

本文定义 `matlab-practice/` 后续 MBD 测试状态管理的中长期方案。

目标不是再增加一个新的零散测试模型，而是把已经存在的大量 MBD 示例、研究模型和
green-joint 数字孪生逐步收编到统一测试体系中。后续 AI 或工程师接手时，先按本文判断：

```text
这个内容是产品控制器、测试调度器、测试场景、plant，还是历史研究资产？
```

## 核心结论

长期方向：

```text
控制器模型和测试模型分离。
测试状态统一由 harness 管理。
可交付算法保持模块化和可生成代码。
零散 MBD 示例逐步登记、分类、复用，而不是重复重建。
```

从 2026-06-24 起，本仓库新增 green-joint 测试按中长期架构执行：

```text
不再为每个测试新建一套完整孤岛 .slx。
不再复制 controller/plant 实现来做 speed/current/position 专项测试。
新增测试先登记为 scenario，再接入统一 TestHarness。
只有基础模块可以独立建模、独立 smoke test、独立 codegen。
```

允许独立存在的对象：

```text
Controller module: CurrentLoop / SpeedLoop / PositionLoop / DqToDuty / ClarkePark
Plant module: Average PMSM / Switching PMSM / Sensor / Delay / Load
Scenario definition: current_square / speed_step / saturation_exit
Unified harness: 组合 controller module、plant module 和 scenario
```

不允许继续扩散的对象：

```text
speed_step_harness_copy_of_everything.slx
current_square_harness_copy_of_everything.slx
position_test_with_duplicated_controller.slx
```

不要把所有测试状态塞进控制器 core：

```text
错误方向：
ControllerModel 内部长期包含 CurrentSquareTest / SpeedStepTest / SaturationExitTest / SweepTest / FaultInject

推荐方向：
ControllerModel 只保留产品真实需要的 mode/state
TestHarness 负责选择测试场景、切换状态、生成参考值、判断 pass/fail
```

## 工业推荐分层

### Layer 1: Product Controller

用途：

```text
真实产品控制算法。
未来可能生成 C 替换固件局部代码。
```

包含：

```text
CurrentLoop
SpeedLoop
PositionLoop
Protection
ModeManager
必要的产品状态机
```

规则：

- 可以使用 Stateflow，但只表达产品真实状态。
- 不放一次性测试流程。
- 不依赖 base workspace 的临时变量。
- 输入输出使用 `.sldd + Bus + AliasType/NumericType + Parameter`。
- 需要交付固件时使用 `ERT + Reusable function`。

### Layer 2: Test Harness

用途：

```text
统一管理测试状态、参考值生成、工况切换和结果采集。
```

包含：

```text
TestSupervisor
ScenarioSelector
ReferenceGenerator
FaultInjection
Assessment
Logger
PlantWrapper
```

规则：

- 默认不生成到嵌入式代码。
- 可以用 Stateflow、Test Sequence、Signal Editor、From Workspace、Simulink Test。
- 可以引用多个 controller/plant variant。
- 所有测试场景都要有唯一场景名和入口脚本。

### Layer 3: Plant

用途：

```text
提供电机、逆变器、传感器、延时和负载模型。
```

包含：

```text
平均电压 plant
开关级 plant
传感器/滤波/延时
负载/摩擦/扰动
真实日志回放
```

规则：

- V1 平均电压模型作为 green-joint 电流环和速度环主判断模型。
- V0 简化 dq plant 只作为辅助 sanity check。
- 开关级模型只做 PWM/ADC/死区/采样窗口专项验证。
- plant 不强行 codegen。

### Layer 4: Scenario Library

用途：

```text
把测试用例从模型结构中剥离出来，形成可复用测试资产。
```

场景示例：

```text
current_square_1khz_0p3A
current_step_0_to_1p5A
current_saturation_exit_4A_to_1p5A
speed_step_0_to_100radps
position_step_0_to_1rad
fault_overcurrent_trip
vbus_drop_12V_to_9V
```

规则：

- 简单参考波形优先用 Signal Editor 或 From Workspace。
- 有步骤、有断言的测试优先用 Test Sequence。
- 有复杂模式切换、故障恢复、互锁逻辑时使用 Stateflow TestSupervisor。
- 大规模组合测试使用 Simulink Test Manager。

## 工具选择规则

| 任务 | 推荐工具 | 是否进入量产代码 |
| --- | --- | --- |
| 产品状态机，例如 Idle/Run/Fault/Calibrate | Stateflow | 可以 |
| 测试流程，例如电流方波、饱和退出、速度阶跃 | Test Harness + Test Sequence | 默认不进入 |
| 复杂测试状态切换，例如校准后进入多阶段测试 | Stateflow TestSupervisor | 默认不进入 |
| 多 plant 或多控制器方案切换 | Variant Subsystem / Variant Manager | 视对象而定 |
| 简单输入波形 | Signal Editor / From Workspace | 不进入 |
| 批量测试、覆盖率、pass/fail 管理 | Simulink Test Manager | 不进入 |
| 可复用算法模块 | Simulink Subsystem / Model Reference / Library | 可以 |

判断口诀：

```text
产品必须拥有的状态 -> Controller/Stateflow，可生成代码。
测试才需要的状态 -> Harness/Test Sequence/Stateflow TestSupervisor，不生成代码。
多个算法/plant 选择 -> Variant。
多个测试用例管理 -> Simulink Test。
```

## green-joint 推荐长期结构

```text
green_joint_controller/
  current_loop/
  speed_loop/
  position_loop/
  protection/
  mode_manager/

green_joint_digital_twin/
  controller_wrapper/
  plant_average_v1/
  plant_switching_studies/
  test_harness/
  scenarios/
  results/
  reports/

motor_control_modules/
  shared controller blocks
  shared dictionaries
  shared library links
```

当前不一定马上重排目录，但新增模型和脚本应朝这个结构靠拢。

green-joint 统一测试顶层的目标形态：

```text
green_joint_control_test_harness.slx
  TestSupervisor / Test Sequence
    -> scenario_bus
    -> reference_bus
    -> fault_injection_bus
  GreenJointControllerWrapper
    -> SpeedPiStep
    -> GreenJointCurrentLoopStep
    -> DqToAbcDutyStep
  PlantWrapper
    -> AverageValuePMSM variant
    -> SwitchingPMSM variant
    -> LogReplay variant
  LoggerAssessment
    -> result bus
    -> pass/fail metrics
```

`GreenJointControllerWrapper` 只能组合已有基础模块，不能在 wrapper 内重新实现 PI、Clarke/Park、SVPWM。
如果需要修改算法，回到对应 `*_mbd/` 模块目录修改并更新模块测试。

## green-joint TestSupervisor 状态建议

第一阶段只建立少量稳定状态：

```text
Idle
CurrentSquareTest
CurrentStepTest
CurrentSaturationExitTest
FaultStop
```

第二阶段加入外环：

```text
SpeedStepTest
SpeedSweepTest
PositionStepTest
PositionTrajectoryTest
```

第三阶段加入鲁棒性和故障：

```text
VbusDropTest
OverCurrentFaultTest
SensorOffsetTest
EncoderDelayTest
LoadDisturbanceTest
```

TestSupervisor 输出建议：

```text
test_enable
control_mode
input_mode
id_ref
iq_ref
speed_ref
position_ref
fault_injection_cmd
scenario_id
```

Controller 输出建议：

```text
id
iq
vd_cmd
vq_cmd
voltage_mag_norm
duty_a
duty_b
duty_c
state_observer
fault_status
```

速度环测试接入规则：

```text
SpeedStepTest / SpeedSweepTest 属于 Scenario，不属于新的独立模型。
SpeedLoop 输出 iq_ref，CurrentLoop 继续输出 vd/vq。
SpeedLoop 采样时间默认慢于 CurrentLoop，跨速率必须显式 Rate Transition。
速度反馈来自 PlantWrapper 的机械速度 wm_meas，单位 rad/s。
iq_limit 是速度环输出限幅，单位 A，必须作为 scenario/controller 参数记录。
```

推荐连接：

```text
TestSupervisor.speed_ref
  -> SpeedPiStep(wm_ref, wm_meas, iq_limit)
  -> iq_ref
  -> Rate Transition, Ts_speed to Ts_current
  -> GreenJointCurrentLoopStep(id_ref, iq_ref, id_fbk, iq_fbk, vbus)
  -> DqToAbcDutyStep
  -> PlantWrapper
  -> wm_meas/id_fbk/iq_fbk
```

## 统一现有零散 MBD 示例

当前目录中已经有很多有价值的 MBD 示例。问题不是它们没有价值，而是缺少统一状态切换和统一索引。

后续不要粗暴删除旧例子，先分类：

| 分类 | 说明 | 处理方式 |
| --- | --- | --- |
| `[MODULE]` | 可复用、可 codegen 的控制算法 | 保留模块目录，纳入 controller 或 module library |
| `[PLANT]` | 电机、逆变器、传感器、负载模型 | 纳入 plant wrapper 或专项 plant |
| `[SCENARIO]` | 方波、阶跃、扫频、故障注入 | 迁移成 scenario library |
| `[STUDY]` | 机理研究、参数扫描、论文复现 | 保留为 research study，不强行 codegen |
| `[LEGACY]` | 旧技术路线或历史模型 | 保留历史价值，不作为新模板 |

迁移顺序：

```text
1. 先登记场景名、输入、输出、采样时间、依赖模块。
2. 再把重复的参考值生成逻辑抽到 TestSupervisor 或 scenario 数据。
3. 再把成熟 controller 抽成可复用 MBD 模块。
4. 最后用 Test Manager 或统一 run 脚本批量运行。
```

## 推荐测试场景命名

测试场景名必须包含对象、输入形式和关键参数：

```text
current_square_1khz_0p3A
current_step_0_to_1p5A
current_saturation_exit_4A_to_1p5A
speed_step_0_to_100radps
position_step_0_to_1rad
```

脚本命名：

```text
run_<target>_<scenario>_test.m
build_<target>_<harness>_model.m
compare_<target>_<scenario>_log.m
```

结果文件命名：

```text
<scenario>_<controller_version>_<plant_version>.csv
<scenario>_<controller_version>_<plant_version>.png
<scenario>_<controller_version>_<plant_version>.md
```

## 中长期实施路线

### Phase 0: 现在开始执行

目标：

```text
新测试不再新建孤岛模型。
green-joint 数字孪生先建立统一 harness 的目录、场景和模块边界。
```

动作：

- 新增 MBD 测试前，先检查 `docs/current_mbd_landscape.md` 和本文。
- 所有新测试场景写入对应 README 或 progress。
- 新增速度环、位置环测试前，先登记 scenario，不直接复制 `.slx`。
- 当前已有独立 harness 只作为迁移来源，不作为新增测试模板。
- green-joint 电流环测试优先使用 V1 平均电压模型。
- V0 简化模型只保留为快速 sanity check。

### Phase 1: 统一 green-joint 电流环测试

目标：

```text
电流方波、阶跃、饱和退出测试由同一个 harness 管理。
```

动作：

- 在 `green_joint_digital_twin/` 下建立 test harness 结构。
- 用 Test Sequence 或 Stateflow TestSupervisor 生成 `id_ref/iq_ref/control_mode/input_mode`。
- 统一记录 `iq_ref/iq/vd/vq/voltage_mag_norm/duty`。
- 对齐硬件 scope 的采样顺序和仿真日志。

### Phase 2: 扩展到速度环和位置环

目标：

```text
外环测试复用同一套 harness，不重新建一套孤岛模型。
```

动作：

- 增加 `SpeedStepTest` 和 `PositionStepTest` scenario。
- 复用 `motor_speed_pi_mbd/SpeedPiStep` 或迁移后的 green-joint speed module。
- 复用 `green_joint_current_loop_mbd/GreenJointCurrentLoopStep` 作为内环。
- 使用 Variant 或 Model Reference 切换 controller module 版本。
- 使用同一 PlantWrapper 接平均电压模型。
- 将 pass/fail 指标加入测试场景。

### Phase 3: 建立测试资产目录

目标：

```text
所有测试场景、模块和 plant 都能被索引和复用。
```

动作：

- 建立 scenario catalog，例如 `scenarios/README.md` 或 `scenarios/catalog.yaml`。
- 每个场景记录采样时间、参考输入、预期响应、通过条件。
- 使用统一 `run_all_smoke_tests.m` 或 Simulink Test Manager 运行基础回归。
- 把成熟算法沉淀到 `motor_control_modules/` 或独立 `*_mbd/` 包。

### Phase 4: 产品化回归

目标：

```text
MBD 模型、生成 C、固件日志形成闭环验证。
```

动作：

- 每个可交付 controller module 都有 smoke test、codegen check、硬件日志对比。
- 每次调参都能追踪到物理量纲参数和测试结果。
- 固件只接收通过 harness 验证的生成代码。
- 测试 harness 保留在仿真环境，不污染量产控制器。

## 禁止扩散的模式

以下做法短期看快，长期会浪费资产：

- 每个测试单独复制一个完整 `.slx`。
- 把测试专用 Stateflow 放进可交付控制器 core。
- 在 base workspace 里散放 `Kp/Ki/Ts/Vbus`。
- 用模型内部硬编码方波、阶跃、扫频，外部无法选择。
- 同一个电流环 PI 在多个模型里各自实现一遍。
- 只保存截图，不保存场景参数、采样时间和结果数据。
- 用开关级模型承担所有控制整定。
- 把 research 模型误当成 codegen 交付模板。

## 给后续 AI 的执行规则

后续 AI 接到 MBD 测试任务时，先执行：

```text
1. 判断任务属于 controller、harness、scenario、plant、study 还是 legacy。
2. 检查是否已有模块或场景能复用。
3. 如果是测试状态切换，优先修改 harness，不要污染 controller core。
4. 如果是算法交付，按 MBD/codegen 标准建立模块和 `.sldd`。
5. 如果是 plant 物理验证，按平均模型或开关模型目标选择，不强行 codegen。
6. 更新 README/progress，避免测试资产再次变成孤岛。
```

## 一句话

```text
未来不是更多零散 MBD 例子，而是一个统一 harness 管理很多可复用模块、plant 和测试场景。
```
