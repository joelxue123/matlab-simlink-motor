# Model Development Standard

本文件定义 `matlab-practice/` 的 MBD 模型开发标准。后续新模型、新模块、新测试优先遵守本文件。

## 总原则

- 新 MBD 模块必须能被他人复用，不只是当前聊天能跑。
- 算法核心必须平台无关，不包含 MCU 寄存器、HAL、芯片头文件、RTOS 细节。
- 接口合同集中在 `.sldd`，不要把类型、Bus、参数散落在 base workspace 或脚本深处。
- 每个模型必须有可重复运行的 build/test 入口。
- 不手工修改生成的 C/H 文件。
- 仿真结论和硬件结论分开记录，硬件结论必须带数据文件、采样频率、固件版本或平台版本。

## 主线优先与临时模型纪律

`green-joint` 后续 MBD 开发必须按层级主线推进：

```text
电机/驱动器/减速器 plant
  -> 电流环
  -> 速度环
  -> 位置环
  -> 状态机 / 测试场景
```

规则：

- 默认不新建孤岛模型；新增速度环、位置环、状态切换测试时，优先接入统一 digital twin / test harness。
- 如果确实需要重新搭一个极简模型做机理验证，必须在开始前明确告知用户：目的、范围、预计保留还是废弃。
- 临时验证模型必须显式命名为 `prototype`、`temporary`、`scratch` 或写入 README，不能伪装成生产主线。
- 临时模型只允许验证单一问题，例如一个 PI 饱和公式、一个延时假设、一个单位换算，不允许逐步长成第二套系统。
- 验证完成后必须回归主线：把结论、参数、模块或测试场景迁回统一 controller/plant/harness。
- 如果临时模型不再需要，必须在文档中标记为历史/废弃/参考，不允许让后续 AI 误认为它是正式路线。
- 重新生成固件 C 前，必须反查源模型、数据字典、adapter、调用点和验证命令，不能只看生成代码。
- 代码生成脚本必须在 `slbuild` 前切换到自己的脚本目录，防止从 digital twin 或其它 harness 串联调用时把 `*_ert_rtw` 生成到错误目录。
- 同步到固件后必须用 `diff -qr` 比对源 `*_ert_rtw` 与固件 `Module/MBD/*` 目录；只允许 MATLAB 元数据/报告文件差异，`.c/.h` 不允许漂移。

允许的例外：

```text
为了快速验证一个数学假设，可以临时造最小模型。
但它只能是验证工具，不是新主线。
最后必须回到统一 MBD 主线。
```

## 模型选择规则

详细决策见：

```text
docs/modeling_scope_decision_guide.md
```

简化判断：

```text
平均电压模型：
  用于控制算法主线、长时间闭环仿真、PI/速度环/MTPA/弱磁初步验证、
  MBD 模块集成和可生成 C 的算法接口检查。

开关型模型：
  用于 PWM 边沿、死区、采样窗口、电流纹波、MOSFET/Diode 导通、
  共模/零相量分配/DPWM 对采样条件影响的专项验证。

MBD/codegen：
  用于会交付给固件或同事复用的算法 core 和稳定 adapter。

原有仿真/脚本：
  用于物理 plant、Simscape/SPS、波形观察 harness、参数扫描、CSV 分析和报告。
```

推荐组合：

```text
平均模型先跑通控制。
开关模型专项验证 PWM/ADC/死区问题。
把验证得到的补偿/判定逻辑沉淀成 MBD 模块。
不要把开关级 plant 强行 codegen。
```

## 有限 MBD 边界

本仓库采用“有限 MBD”原则，不把所有东西都强行做成可生成 C 的 MBD 模块。

必须按 MBD 标准开发的对象：

- 用户级控制算法。
- 可交付给固件工程的算法 core。
- 电流环、速度环、坐标变换、SVPWM/duty、deadtime compensation、采样 valid 判定、observer、参数辨识在线算法。
- 需要稳定 C 接口、Bus、类型、参数和可重入函数的模块。

可以保持原有仿真/研究形态的对象：

- Simscape/SPS 物理 plant。
- Universal Bridge、MOSFET/Diodes、PMSM、RL winding 等开关级功率级模型。
- 波形观察 harness。
- 一次性参数扫描和论文/机理研究脚本。
- 只服务验证的 scope、To Workspace、plot/report 脚本。

边界判断：

```text
会交付给用户/同事/固件，并且会变成稳定接口的算法 -> MBD。
只用于证明物理现象、复现实验、观察波形的 plant/harness -> 保持仿真需求优先。
```

例如：

```text
DeadtimeCompensationStep -> pwm_deadtime_compensation_mbd/，MBD/codegen 算法模块
DeadtimeSamplingWindowStep -> pwm_deadtime_sampling_mbd/，MBD/codegen adapter 模块
Universal Bridge + PMSM deadtime plant -> Simulink/SPS 验证 harness
```

## 标准目录结构

新建可复用 MBD 模块时，优先使用：

```text
<module_name>/
  README.md
  interface.yaml                      preferred for new modules
  build_<module_name>_model.m
  generate_<module_name>_dictionary.m preferred when interface.yaml is used
  run_<module_name>_smoke_test.m
  generate_<module_name>_code.m        optional
  <module_name>_interface.sldd
  <module_name>.slx
  results/                             optional
  reports/                             optional
```

如果模块属于共享库，则放入：

```text
motor_control_modules/
```

如果模块属于某个实验 sandbox，则放入对应专题目录，例如：

```text
identification/<topic_name>/
motor_performance_characterization/
```

## README 必须写清楚

每个模块的 `README.md` 至少包含：

- 模块目标。
- 输入 Bus、输出 Bus。
- 主要参数。
- 单位和数据类型。
- 采样时间。
- 构建命令。
- 测试命令。
- 代码生成命令，如果该模块需要交付 C。
- 当前验证结果。
- 已知限制。

## 数据字典标准

接口合同优先放在 `.sldd`：

```text
Simulink.AliasType
Simulink.NumericType
Simulink.Bus
Simulink.Parameter
```

推荐规则：

- 浮点第一版仍使用工程类型名，例如 `T_MotorCurrent`，底层可绑定到 `single`。
- 定点接口使用 `Simulink.NumericType`，不要把 `fixdt(...)` 字符串散落在每个 block。
- 结构体接口使用 `Simulink.Bus`。
- 参数使用 `Simulink.Parameter`，并记录单位、范围和默认值。
- 字典构建脚本要能重复运行。
- 如果 `.sldd` 被 MATLAB 占用，脚本应先关闭打开的 dictionary 或提示处理。

长期用户接口纪律见：

```text
docs/mbd_interface_contract_standard.md
```

新模块优先采用：

```text
interface.yaml / interface.json
  -> generator .m
  -> interface.sldd
```

`.sldd` 仍是 Simulink/Embedded Coder 的正式数据字典，但用户不应长期直接修改
`.sldd` 或 build 脚本深处的 Bus、类型、Parameter 定义。

## Simulink 安全重建规则

重建 `.slx/.sldd` 是高风险动作，因为 MATLAB Desktop 可能已经打开同名模型、数据字典或缓存文件。

危险组合：

```text
Desktop 正打开模型或数据字典
  + 脚本 delete/recreate 同名 .slx/.sldd
  + 脚本自动 open_system / arrangeSystem / SimulationCommand='update'
```

规则：

- 默认用 `matlab -batch` 重建模型、字典和生成代码。
- build/generate 脚本默认不要自动 `open_system(...)`。
- build/generate 脚本默认不要自动 `Simulink.BlockDiagram.arrangeSystem(...)`。
- build/generate 脚本默认不要自动 `set_param(model, 'SimulationCommand', 'update')`。
- 如果脚本会删除并重建 `.slx/.sldd`，必须先检查是否有其它 MATLAB 进程占用相关文件。
- Desktop 调试模型时，不要同时运行 batch 重建脚本。
- smoke test 或 codegen 可以显式触发 update；普通 build 阶段只负责构建和保存。

如果出现 Simulink 卡在“模型更新”，先读：

```text
docs/simulink_hang_troubleshooting.md
```

示例命名：

```text
T_MotorCurrent
T_MotorVoltage
T_MotorAngle
T_MotorSpeed
motor_t
motor_dq_t
current_pi_input_t
current_pi_output_t
phase_duty_t
```

## 接口与 Bus 规则

- 模块边界优先传 Bus，不传大量散落标量线。
- Bus 字段使用物理意义命名，例如 `ia/ib/ic/theta_e/id/iq/vdc`。
- Bus 字段必须记录单位。
- 上游输出不是目标 Bus 时，用 adapter 子系统组装或映射。
- 不因为某个上游模块变化而污染算法模块接口。
- 多模块之间必须共享同一个 `.sldd` 或有明确的接口转换层。

## 数据类型规则

- 新的控制器仿真第一版优先 float，也就是 `single`，便于 MBD 到嵌入式 C 的过渡。
- 定点化作为单独阶段推进，不能随意把角度、三角函数、SVPWM 全部改成整数。
- 固定点电流、电压、角度必须记录：

```text
Signedness
WordLength
FractionLength / slope
范围
LSB
饱和策略
溢出策略
```

- 累加器通常要比输入更宽，例如电流 PI 积分器不能盲目使用与输入相同的定点类型。

## 采样时间与调度

当前电机控制基线：

```text
25us PWM tick / plant boundary
50us current loop
100us speed loop
```

规则：

- 模型使用 fixed-step discrete solver。
- 每个模块 README 必须写采样时间。
- 多速率边界显式使用 Rate Transition。
- 不用 Zero-Order Hold 伪装跨任务数据同步。
- plant、controller、logger 的采样时间要分开记录。

## 测试状态管理

后续 MBD 测试不再默认新建孤立模型。测试状态要进入统一 harness，而不是污染可交付 controller core。

推荐分层：

```text
Product Controller:
  电流环、速度环、位置环、保护、必要产品状态机，可生成代码。

Test Harness:
  TestSupervisor、ReferenceGenerator、ScenarioSelector、FaultInjection、Logger，不默认生成代码。

Plant:
  平均电压模型、开关级专项模型、传感器/延时/负载模型。

Scenario Library:
  current_square、current_step、saturation_exit、speed_step、position_step 等可复用测试场景。
```

规则：

- 产品状态机可以用 Stateflow，并可按需生成代码。
- 测试状态切换优先放在 Test Sequence 或 Stateflow TestSupervisor。
- 简单波形优先使用 Signal Editor 或 From Workspace。
- 多 controller/plant 版本切换使用 Variant 或 Model Reference。
- 多测试用例和 pass/fail 管理后续使用 Simulink Test Manager。
- 新增测试场景必须写清楚场景名、采样时间、输入、输出、预期结果和运行入口。

详细规则见：

```text
docs/mbd_test_state_management_architecture.md
```

## PI 与限幅规则

当前电流环 PI 推荐透明框图：

```text
error = ref - meas
u_pre_sat = Kp * error + integrator
u_sat = clamp(u_pre_sat, dynamic_limit)
integrator_next = integrator + Ts * (Ki * error + Kaw * (u_sat - u_pre_sat))
```

规则：

- 第一版优先用基础 Simulink blocks，便于审查生成代码。
- 饱和输出和 anti-windup 都要显式可见。
- `Kaw` 过大会导致饱和附近积分状态快速修正，可能引入输出波动，需要单独测试。
- PI 模块必须做饱和阶段、刚超过饱和、退出饱和三类测试。

## 代码生成规则

用于嵌入式 C 交付时：

```text
SystemTargetFile = ert.tlc
GenerateSampleERTMain = off
GenCodeOnly = on
CodeInterfacePackaging = Reusable function
```

接口目标：

```c
void Step(const input_t *in, output_t *out);
```

或无状态小函数：

```c
T_Out Step(T_In x, T_In y);
```

规则：

- 生成 C 是产物，不是手工源代码。
- 修改算法时改 `.m/.slx/.sldd`。
- 生成后检查 header、typedef、函数签名、全局变量和状态结构。
- 有状态模块要确认多实例/可重入要求。
- controller core 不包含 ADC/PWM/encoder/timer/HAL。

## 测试标准

每个模块至少有一个：

```text
run_<module>_smoke_test.m
```

高风险模块需要功能测试或回归测试：

- Clarke/Park：数值误差、角度扫描。
- PI：饱和、anti-windup、退出饱和、限幅边界。
- SVPWM/duty：输出范围 `[0, 1]`、电压利用率、马鞍波 sanity check。
- 参数辨识：合成数据真值误差。
- 电流采样：offset/gain mismatch/noise 的频谱回归。

测试输出建议放在：

```text
results/
reports/
```

重要图像保存为 PNG，关键数值保存为 CSV/TXT，便于下一次 AI 或人直接检查。

## 文档更新规则

完成重要任务后：

- 更新本模块 `README.md`。
- 更新 `docs/progress.md`。
- 若改变共识，更新 `MBD_DEVELOPMENT_NOTES.md`。
- 若修正错误，更新 `MBD_ERRATA.md`。
- 若改变复用方式，更新 `docs/reusable_modules_usage.md`。

这条规则很重要：没有记录的模型，很容易在下一轮变成一次性产物。
