# MBD Development Notes

这个文件是当前仓库的 MBD 学习与工程化继承笔记。目标是让以后的人、我自己、以及其他 AI 进入这个文件夹后，能快速接上已经形成的判断和实践路径。

配套勘误文档：

```text
MBD_ERRATA.md
```

本文件记录当前正确路线；勘误文档记录已经犯过的错误、修正依据和预防规则。

## 长期协作入口

2026-06-10 起，`docs/` 是 `matlab-practice/` 的规范入口：

```text
docs/README.md
docs/progress.md
docs/model_development_standard.md
docs/reusable_modules_usage.md
docs/ai_collaboration_rules.md
```

后续 AI 或人接手时，先看 `docs/README.md`，再进入具体模块目录。`MBD_DEVELOPMENT_NOTES.md`
继续记录学习判断和工程经验，`MBD_ERRATA.md` 继续记录勘误。

## 用户接口合同前移

2026-06-22 形成新的接口维护纪律：

```text
interface.yaml / interface.json
  -> generator .m
  -> interface.sldd
  -> Simulink model
  -> generated C headers
```

原因：

```text
用户和后续 AI 很难长期维护 build 脚本深处的 Simulink.BusElement、
Simulink.AliasType、Simulink.Parameter 代码，也不应该手工修改 .sldd。
```

新的角色划分：

```text
interface.yaml = 用户可编辑接口合同
.sldd          = Simulink/Embedded Coder 使用的数据字典
.m            = 生成器和模型构建工具
.slx          = 算法图
generated .h   = C 工程接口
```

详细纪律见：

```text
docs/mbd_interface_contract_standard.md
```

后续新 MBD 模块优先采用 `interface.yaml`。已有模块如果接口仍写在 `.m` 中，
先标记为 `[TRANSITION]`，等发生实质接口变更时再迁移，不强行一次性重写。

## 有限 MBD 原则

2026-06-10 形成新的工程边界：

```text
用户级算法、稳定接口、可交付 C core -> 按 MBD 标准。
物理 plant、开关器件、验证 harness、探索脚本 -> 保持原来的仿真/研究需求。
```

含义：

```text
电流 PI、速度 PI、Clarke/Park、SVPWM/duty、deadtime compensation、
current_valid、observer、在线参数辨识等用户级算法，应使用 .sldd、Bus、
AliasType/NumericType、可重入接口和 codegen 检查。

Universal Bridge、MOSFET/Diodes、PMSM/SPS plant、波形观察模型、参数扫描脚本，
优先服务物理复现、可视化和验证，不强行做成可生成 C 的 MBD 模块。
```

这个原则用于防止过度 MBD：MBD 是算法交付方式，不是所有仿真对象的唯一形态。

## 当前里程碑

2026-06-04 形成了一个重要结论：

```text
新项目不要优先学 MPT。
现代 MBD/Embedded Coder 主线应优先学习：

Simulink Data Dictionary (.sldd)
  + Simulink.AliasType / Simulink.NumericType / Simulink.Bus
  + Code Mappings / Embedded Coder Dictionary
  + ERT reusable / reentrant C interface
```

MPT 仍然需要认识，但主要用于读老项目、维护 legacy 工程，不作为新项目首选方案。

## 已完成的四个关键例子

### 1. 最小可重入加法模块

目录：

```text
simple_add_reentrant/
```

目标：

```text
z = x + y
```

关键点：

- 使用 `add_interface.sldd` 管理接口类型。
- 使用 `Simulink.AliasType` 生成稳定 typedef。
- 模型端口不直接写 `single`，而是引用 `T_AddIn`、`T_AddOut`、`T_AddAcc`。
- 生成 ERT C 代码。
- 生成接口示例：

```c
typedef real32_T T_AddIn;
typedef real32_T T_AddOut;
typedef real32_T T_AddAcc;

extern T_AddOut AddStep(T_AddIn rtu_x, T_AddIn rtu_y);
```

客户要改数据类型时，不改生成的 `.c/.h`，而改：

```text
simple_add_reentrant/build_simple_add_reentrant_model.m
```

里的：

```matlab
customer_interface_config()
```

例如：

```matlab
cfg.inputBaseType = 'int16';
cfg.outputBaseType = 'int16';
cfg.accumulatorBaseType = 'int32';
```

### 2. motor_t 结构体 + Clarke/Park 变换

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

关键点：

- 使用 `motor_interface.sldd` 管理结构体和标量类型。
- `T_MotorCurrent` 用 `Simulink.NumericType` 定义为定点电流类型。
- `T_MotorAngle` 暂时用 `Simulink.AliasType('single')` 定义。
- `motor_t`、`motor_dq_t` 用 `Simulink.Bus` 定义。
- Simulink 模型边界使用 Bus 结构体。
- 内部用 Bus Selector / Simulink 基础运算块 / Bus Creator 实现算法。
- 已补模型级功能测试 `run_motor_clarke_park_function_test.m`。
- 测试脚本会保存可视化 harness：`motor_clarke_park_function_test_harness.slx`。
- 当前电流接口类型为 `sfix16_En12`：
  - signed 16-bit storage
  - 12 fractional bits
  - LSB = `2^-12 = 0.000244140625`
  - approximate range = `[-8, 7.999755859375]`
- 生成结构体接口：

```c
typedef int16_T T_MotorCurrent;
typedef real32_T T_MotorAngle;

typedef struct {
  T_MotorCurrent ia;
  T_MotorCurrent ib;
  T_MotorCurrent ic;
  T_MotorAngle theta_e;
} motor_t;

typedef struct {
  T_MotorCurrent i_alpha;
  T_MotorCurrent i_beta;
  T_MotorCurrent id;
  T_MotorCurrent iq;
} motor_dq_t;

extern void MotorClarkeParkStep(const motor_t *rtu_motor_in,
                                motor_dq_t *rty_dq_out);
```

这个例子证明 `.sldd` 对结构体接口非常有价值：类型、字段、头文件、生成代码命名都可以集中管理。

功能测试当前结果：

```text
Maximum error: 0.000244
Motor Clarke/Park functional test passed.
Saved visual test harness:
  motor_clarke_park_struct/motor_clarke_park_function_test_harness.slx
```

### 3. float 电机开环仿真架构

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

关键点：

- 参考 `average-inverter` 的电机、母线、电压开环、平均逆变器和 PMSM 原理。
- 不沿用 legacy 的散落 `algorithms/*.m` 交付方式，而是建立新的 MBD 模块包。
- 控制器接口统一使用 float，也就是 `.sldd` 中的 `Simulink.AliasType('single')`。
- 类型名仍然使用工程语义，例如 `T_MotorVoltage`、`T_MotorCurrent`、`T_MotorAngle`。
- 模块边界使用 Bus 合同，例如 `open_loop_cmd_t`、`phase_duty_t`、`plant_feedback_t`。
- `OpenLoopCommand` 使用离散角度累加器，不用 `Clock` 直接生成 `theta = omega*t`。
- `phase_duty_t` 表示 Average-Value Inverter 使用的 `[0, 1]` duty。这个判断以
  `average-inverter/speedloop_kf_test.slx` 和 `algorithms/dq2abc_fcn.m` 为准：
  `duty = v_svpwm / Vdc + 0.5`。
- Average-Value Inverter 的输入边界明确用 `PhaseDuty_RateTransition_25us` 和
  `Vdc_RateTransition_25us` 表达 25 us 多速率任务边界，即
  `simcfg.Ts_plant = 25e-6`。
- Motor Control Blockset plant 边界允许显式转换到 double，以保证库模块传播稳定。
- 控制器侧合同仍保持 single；库模块内部是否 double 不污染控制器接口。
- 仿真调度量、sample time 仍使用 double，因为 Simulink 的 SampleTime 参数期望 double 标量。

文件：

```text
motor_float_open_loop_mbd/build_motor_float_open_loop_model.m
motor_float_open_loop_mbd/run_open_loop_smoke_test.m
motor_float_open_loop_mbd/README.md
motor_float_open_loop_mbd/motor_float_interface.sldd
motor_float_open_loop_mbd/motor_float_open_loop_model.slx
```

功能测试命令：

```bash
matlab -batch "run('motor_float_open_loop_mbd/run_open_loop_smoke_test.m')"
```

当前测试结果：

```text
Open-loop smoke test result:
  wm      = 26.8943 rad/s
  theta_e = 26.2228 rad
  ia      = 4.46792 A
  ib      = 8.83261 A
  ic      = -13.3005 A
Open-loop smoke test passed.
```

测试中出现的 single precision loss warning 可以接受。它们表示 `2*pi`、`sqrt(3)/2`、`1/3`
这类常数被量化到 single，这是 float-first 里程碑的预期行为。

这个例子是新的电机仿真架构基线：先把“开环电压 -> 逆变器 -> 电机 plant -> feedback bus”
跑通，再逐步替换开环命令为 Clarke/Park、current reference、PI、限幅、SVPWM 和闭环 FOC。

### 4. 电流环 PI 框图模块

目录：

```text
motor_current_pi_mbd/
```

目标：

```text
current_pi_input_t
    -> CurrentPiStep
    -> current_pi_output_t
```

关键点：

- 不使用 MATLAB Function block。
- 不直接使用黑盒 PID Controller block。
- 使用 Simulink 基础框图搭建 P、I、动态限幅和 back-calculation anti-windup。
- 参数 `Kp_id`、`Ki_id`、`Kaw_id`、`Kp_iq`、`Ki_iq`、`Kaw_iq`、`VLimitRatio`
  放在 `current_pi_interface.sldd`。
- 接口类型用 `.sldd + Simulink.AliasType('single')`。
- 输入输出使用 Bus：

```text
current_pi_input_t {
  id_ref
  iq_ref
  id_meas
  iq_meas
  omega_e
  vdc
}

current_pi_output_t {
  vd_ref
  vq_ref
}
```

当前 PI 公式：

```text
error = ref - meas
u_pre_sat = Kp * error + integrator
u_sat = clamp(u_pre_sat, -0.577 * vdc, 0.577 * vdc)
integrator_next = integrator + Ts * (Ki * error + Kaw * (u_sat - u_pre_sat))
```

功能测试命令：

```bash
matlab -batch "run('motor_current_pi_mbd/run_current_pi_smoke_test.m')"
```

当前测试结果：

```text
vd_ref range = [0, 0] V
vq_ref range = [13.3204, 39.236] V
v_limit      = 39.236 V
Current PI smoke test passed.
```

本阶段结论：

```text
电流环 PI 第一版应优先用基础 Simulink 框图搭建。
这样比直接用 PID block 更透明，也比 MATLAB Function 更符合当前 MBD 继承标准。
```

## 推荐学习顺序

1. `Simulink.AliasType`
   - 学会给 C 类型起稳定工程名。
   - 例如 `T_MotorCurrent`、`T_AddIn`。

2. `Simulink.NumericType`
   - 学定点、缩放、字长、小数位。
   - 以后做电机电流环、MCU 定点实现时会用到。
   - 推荐作为定点接口类型的工程源头，放进 `.sldd`，并设置 exported typedef。

3. `Simulink.Bus`
   - 学结构体接口。
   - 例如 `motor_t`、`motor_dq_t`、控制器输入输出结构体。

4. Simulink Data Dictionary `.sldd`
   - 把类型、Bus、参数放进项目级数据资产库。
   - 避免依赖 base workspace。

5. ERT / Embedded Coder 可重入接口
   - 学 `ert.tlc`、`Reusable function`、`GenCodeOnly`、`GenerateSampleERTMain = off`。
   - 重点理解“模型接口”和“算法子系统接口”的区别。

6. Code Mappings / Embedded Coder Dictionary
   - 后续控制 storage class、memory section、头文件组织、参数标定方式。

7. MPT
   - 只作为 legacy 维护知识。
   - 能读懂老项目里的 `mpt.Parameter`、`mpt.Signal` 即可。

## 当前工程判断

### 不要直接改生成代码

不要手工修改：

```text
*_ert_rtw/*.c
*_ert_rtw/*.h
```

这些文件由 Simulink Coder 生成。真正的源头是：

```text
build_*.m
*.sldd
*.slx
customer_interface_config()
```

### 客户接口类型应该集中管理

临时 demo 可以写：

```matlab
'OutDataTypeStr', 'single'
```

但工程化示例应写：

```matlab
'OutDataTypeStr', 'T_MotorCurrent'
```

其中 `T_MotorCurrent` 在 `.sldd` 里定义。

### 框图表达算法，类型系统表达数值契约

`motor_clarke_park_model` 的框图里没有显式放很多 Data Type Conversion 模块，也没有在每个
block 上标注 `Q12`，但生成代码里仍然出现了 Q12 缩放、移位和饱和逻辑。这是现代
Simulink 类型系统很有价值的一点：

```text
框图 = 算法结构
.sldd + NumericType = 数值类型契约
Embedded Coder = 把类型契约落实到 C 代码
```

因此，工程化模型不应该为了“看起来是定点”而到处手工插 Convert，也不应该把
`fixdt(...)` 字符串散落在每个 block 上。更好的做法是：

- 端口、Bus 字段、关键中间量使用稳定工程类型名，例如 `T_MotorCurrent`。
- 类型名在 `.sldd` 中统一绑定到 `Simulink.NumericType`。
- 需要强制改变数值域、缩放、溢出策略的边界，才显式使用 Data Type Conversion。
- 生成代码后检查 `.h/.c`，确认 typedef、缩放、移位、饱和逻辑符合预期。

这也是 `.sldd + NumericType` 比“在框图里到处写 Q 格式”更适合长期维护的原因：
算法图保持干净，数值体系集中管理，生成 C 又能真实落到定点实现。

### `.sldd` 文件可能损坏或半创建

如果运行构建脚本时报错类似：

```text
无法获取创建时间/上次修改时间/上次保存时间/生成时间 - 主表中没有行
```

或：

```text
无法创建或打开数据字典，因为另一个具有相同文件名的字典已打开
```

通常是两类问题：

- `.sldd` 文件存在，但内部主表不完整，常见原因是上一次创建或保存字典时
  MATLAB/Simulink 被中断。
- 在交互式 MATLAB 中反复运行脚本时，旧模型、测试 harness 或字典句柄还占着同名
  `.sldd`。

处理原则：

- 不要手工改 `.sldd` 内部文件。
- 构建脚本应该能检测字典打不开的情况。
- 重新生成模型前，先关闭已加载的旧模型和打开的数据字典。
- 对不可用的 `.sldd` 先改名备份，再重新创建。
- 重新运行 build script，让 `customer_interface_config()` 重新生成类型、Bus、参数定义。

`motor_clarke_park_struct/build_motor_clarke_park_model.m` 已加入这个保护逻辑：

```text
close_open_data_dictionaries()
open_or_create_data_dictionary()
backup_bad_dictionary()
```

测试脚本 `run_motor_clarke_park_function_test.m` 在调用 build 前也会先关闭 harness 和已打开的
data dictionary，避免交互式测试后残留句柄影响下一次构建。

### 结构体接口优先用 Bus

如果 C 接口需要：

```c
motor_t input;
motor_dq_t output;
```

Simulink 里应该用：

```text
Simulink.Bus + Bus Element + Data Dictionary
```

而不是用零散的 Inport/Outport 拼接口。

### 多模块连接优先靠共享接口契约

当 `motor_clarke_park_model` 的输入来自其他模块，例如开环算法模块，最好的连接方式不是
让两个模块互相猜信号线，而是让它们共享同一个 `.sldd` 接口契约。

已创建集成示例：

```text
motor_clarke_park_struct/build_motor_open_loop_integration_model.m
motor_clarke_park_struct/motor_open_loop_integration_model.slx
```

顶层连接：

```text
OpenLoopMotorInputStep -> motor_t -> MotorClarkeParkStep -> motor_dq_t
```

原则：

- 上游模块如果输出的就是 `Bus: motor_t`，可以直接连接到 `MotorClarkeParkStep`。
- 下游模块的输入端口也声明为 `Bus: motor_t`，双方引用同一个 `motor_interface.sldd`。
- 模块之间传递的是完整结构体/Bus，不是散落的 `ia`、`ib`、`ic`、`theta_e` 线。
- 如果上游输出是多个标量，用 adapter 子系统或 Bus Creator 组装成 `motor_t`。
- 如果上游输出是另一种结构体，用 adapter 子系统做字段映射，不要污染算法模块接口。
- Bus Creator 的输入信号名必须匹配 Bus 元素名，例如 `ia`、`ib`、`ic`、`theta_e`。

这个阶段的核心能力是“接口契约设计”：算法模块保持稳定，上游/下游变化通过 adapter 层吸收。

### 不要把新 MBD 模块退回散落的 MATLAB Function 脚本

`average-inverter/algorithms/` 里已有一些 `*.m` 算法函数，它们适合仿真快速验证或 legacy
工程延续。但今天形成的新标准不能倒退：

```text
新的嵌入式交付模块
  不应只是 algorithms/foo_fcn.m
  不应只是 MATLAB Function block
  不应绕过 .sldd / Bus / NumericType
```

新的可复用 MBD 模块应做成独立模块包：

```text
algorithms/<module_name>_mbd/
  README.md
  build_*.m
  run_*_test.m
  generate_*_code.m
  *.sldd
  *.slx
```

已按这个标准迁入：

```text
average-inverter/algorithms/motor_clarke_park_mbd/
```

这个目录保留 `.sldd + NumericType + Bus + 可重入模型 + 功能测试`，而不是散落
`motor_clarke_park_fcn.m` 这类脚本。后续新里程碑模块也应该采用这种形态。

### 可重入接口是嵌入式交付重点

优先生成类似：

```c
extern void Step(const input_t *in, output_t *out);
```

或无状态纯函数：

```c
extern T_Out Step(T_In x, T_In y);
```

避免把算法 API 设计成依赖全局 root I/O 的形式。

## 已创建的 Codex Skill

为了让后续 AI 复用这次经验，已创建本地 Skill：

```text
/home/user/.codex/skills/mbd-simulink-codegen/
```

主要文件：

```text
/home/user/.codex/skills/mbd-simulink-codegen/SKILL.md
/home/user/.codex/skills/mbd-simulink-codegen/references/modern-add-template.md
```

下次可以对 AI 说：

```text
用 mbd-simulink-codegen skill，继续做一个现代 Simulink 代码生成例子
```

## 工具放大器地图

这个章节记录“使用工具超越自我”的长期路线。核心判断：

```text
手写代码是能力底座。
工具链是能力放大器。
测试和审查是安全边界。
笔记和 skill 是长期记忆。
```

目标不是停止写代码，而是从“每次亲手造零件”升级为：

```text
我定义规格、接口、类型、验证规则；
工具稳定生成和检查；
我负责判断、验收、集成、迭代。
```

### 当前优先工具链

这些工具和当前 MBD/嵌入式方向最贴近，应优先逐步学通：

| 工具/能力 | 放大的能力 | 当前状态 | 下一步 |
| --- | --- | --- | --- |
| Simulink Data Dictionary `.sldd` | 类型、Bus、参数集中管理 | 已在两个例子中使用 | 引入参数、单位、范围、版本管理习惯 |
| `Simulink.AliasType` | 稳定 C typedef 名称 | 已在加法和角度类型中使用 | 总结 alias 与 NumericType 的边界 |
| `Simulink.NumericType` | 定点字长、缩放、符号统一管理 | 已实现 `T_MotorCurrent = sfix16_En12` | 做 Q15 电流、定点角度、range analysis |
| `Simulink.Bus` | C 结构体接口建模 | 已实现 `motor_t`、`motor_dq_t` | 扩展到 FOC 输入输出结构体 |
| Embedded Coder / ERT | 生成可嵌入 C 代码 | 已生成 reusable/reentrant C | 学 Code Mappings 和 Embedded Coder Dictionary |
| Code Mappings | storage class、参数、信号代码形态 | 待实现 | 做 PI 参数可标定例子 |
| Simulink Test / SIL / PIL | 模型与生成代码对比验证 | 已有基础 functional test | 增加 SIL 对比 |
| Polyspace / 静态分析 | 溢出、死代码、MISRA、运行时错误检查 | 未开始 | 后续对生成代码做静态分析 |
| A2L / XCP / CANape / INCA | 标定和在线观测 | 未开始 | 等参数例子成熟后再接入 |
| MCU wrapper 模板 | 生成代码接真实中断、ADC、PWM | 未开始 | 写 `MotorControl_Init()` 和 `10kHz_ISR()` 模板 |

### 更大的工具版图

这些工具不一定马上实现，但要知道它们属于哪一类放大器：

| 类别 | 代表工具 | 放大的能力 |
| --- | --- | --- |
| AI 编程协作 | Codex、Claude Code、Cursor | 代码理解、脚本生成、重构、测试、文档沉淀 |
| 自动测试 | MATLAB Unit Test、Simulink Test、pytest、CI test | 快速反馈、回归验证、敢于迭代 |
| 静态分析 | Polyspace、MISRA Checker、clang-tidy、cppcheck | 提前发现溢出、未初始化、死代码、规范违规 |
| 建模与代码生成 | Simulink、Stateflow、Embedded Coder、AUTOSAR Blockset | 从手写代码升级到模型、接口、代码生成 |
| 标定与测量 | ASAP2/A2L、XCP、CANape、INCA | 参数标定、在线观测、实验效率 |
| 系统仿真 | Simscape、PLECS、CarSim、Amesim | 没有完整硬件时验证系统行为 |
| 控制设计 | Control System Toolbox、System Identification、Optimization Toolbox | 控制器设计、辨识、参数优化 |
| 工程自动化 | Git、GitLab CI、Jenkins、CMake、Make | 构建、测试、发布、协作可追溯 |
| 硬件调试 | J-Link、Trace32、示波器、逻辑分析仪、CAN 分析仪 | 看见真实硬件和总线行为 |
| 文档知识库 | Markdown、Doxygen、Sphinx、MkDocs、Obsidian | 长期记忆、团队继承、AI 继承 |

### 工具更新规则

以后每接触一个新工具，都按这个格式追加：

```text
工具名：
  解决什么重复劳动：
  放大哪种能力：
  当前掌握程度：
  与 MBD/嵌入式链路的关系：
  下一步最小实践：
```

每次形成新判断，都要同步更新：

- 本文件。
- 如果这个判断来自纠错，要更新 `MBD_ERRATA.md`。
- 对应例子的 README。
- 必要时更新 `/home/user/.codex/skills/mbd-simulink-codegen/`。

## 给后续 AI 的工作原则

进入本仓库处理 MBD/Simulink/Embedded Coder 任务时：

1. 先读本文件。
2. 再读 `MBD_ERRATA.md`，避免重复已知错误。
3. 再看相关例子目录的 README。
4. 新项目默认使用 `.sldd + AliasType/NumericType/Bus`。
5. 不要优先使用 MPT，除非用户明确说在维护老工程。
6. 不要手工改 generated C。
7. 不要默认做 gcc 编译、git 操作或额外封装，除非用户明确要求。
8. 如果要改客户接口类型，优先改 `customer_interface_config()`。
9. 如果要做结构体接口，优先用 `Simulink.Bus` 放进 `.sldd`。
10. 生成代码后，优先检查 `.h` 文件里的 typedef、struct、函数原型。
11. 把重要结论继续追加到本文件，形成可继承学习轨迹。
12. 如果修正了错误判断，要追加到 `MBD_ERRATA.md`。
13. 如果引入或讨论了能显著放大工程能力的新工具，要更新“工具放大器地图”。
14. 对工具保持“双重态度”：积极使用它放大能力，同时用测试、审查和底层知识验收结果。

## 后续必须实现的 MBD 工程路线图

下面不是普通建议，而是本仓库后续 MBD 学习与工程化必须逐步实现的清单。以后继续做 MBD 任务时，应优先从这里取题推进。

### Todo 1：定点类型体系（已完成第一版）

目标：

```text
用 Simulink.NumericType 做一个定点版 motor_t 电流接口。
```

已实现：

- 在 `motor_clarke_park_struct/build_motor_clarke_park_model.m` 的
  `customer_interface_config()` 中定义客户可改的定点接口：

```matlab
cfg.currentTypeKind = 'fixed';
cfg.currentSignedness = 'Signed';
cfg.currentWordLength = 16;
cfg.currentFractionLength = 12;
```

- 在 `motor_interface.sldd` 中生成 `T_MotorCurrent`，
  类型为 `Simulink.NumericType`。
- `T_MotorCurrent` 设置为 exported alias，生成到 `motor_types.h`。
- 生成 C 代码后，`motor_types.h` 中为：

```c
typedef int16_T T_MotorCurrent;
```

- 生成算法中可以看到 Q12 缩放，例如 `0.000244140625F`、移位、饱和逻辑。
- 功能测试已按定点量化更新，当前最大误差为 `0.000244`。

本阶段结论：

- `Simulink.AliasType('single')` 适合“给已有基础类型起稳定工程名”。
- `Simulink.NumericType` 适合“定义数值体系本身”，尤其是定点的符号位、字长、小数位、缩放。
- 对 MCU/电机控制这类嵌入式项目，电流、电压、占空比、ADC 量纲以后应优先用
  `NumericType` 做清楚。

后续仍要实现：

- 为角度设计定点格式，例如 per-unit angle 或 Q15 angle。
- 设计 sin/cos LUT 或 CORDIC，而不是直接把 `theta_e` 粗暴改成定点后继续用
  浮点三角函数。
- 做系统化 range analysis，确认电流范围、PI 累加器范围、乘法中间量范围。

### Todo 2：FOC 电流环结构体接口

目标：

```text
把 Clarke/Park 例子升级成 FOC 电流环输入输出结构体。
```

要实现：

- 输入结构体，例如 `foc_input_t`：
  - `ia`
  - `ib`
  - `ic`
  - `theta_e`
  - `id_ref`
  - `iq_ref`
  - `vdc`
- 输出结构体，例如 `foc_output_t`：
  - `vd`
  - `vq`
  - `v_alpha`
  - `v_beta`
  - `duty_a`
  - `duty_b`
  - `duty_c`
- 内部逐步加入 Clarke、Park、PI、电压限幅、逆 Park、SVPWM 或简化调制。

### Todo 3：可标定参数与 Code Mappings

目标：

```text
引入 Simulink.Parameter 和 Code Mappings，演示可标定 PI 参数。
```

要实现：

- 在 `.sldd` 中定义 `Kp_id`、`Ki_id`、`Kp_iq`、`Ki_iq`。
- 使用 `Simulink.Parameter` 管理参数值、类型、范围、单位。
- 使用 Code Mappings 配置参数在 C 代码中的 storage class。
- 生成类似可标定全局参数或 const 参数的代码。
- 对比“普通常量”“InlineParams”“可调参数”的代码差异。

### Todo 4：Embedded Coder Dictionary

目标：

```text
学习并实践 storage class、memory section、头文件组织。
```

要实现：

- 创建或配置 Embedded Coder Dictionary。
- 定义项目级 storage class。
- 定义 calibration、signal、state、constant 的代码放置规则。
- 演示参数放到指定 header/source。
- 演示 memory section 或 pragma 的基本形式。

### Todo 5：SIL 对比验证

目标：

```text
做 SIL 对比：Simulink 模型输出 vs generated C 输出。
```

要实现：

- 保留模型级功能测试 harness。
- 增加 SIL 或 generated-code-in-loop 测试。
- 同一批输入同时跑 Simulink 模型和生成 C。
- 自动比较误差并输出 PASS/FAIL。
- 总结测试阈值、浮点误差、定点误差的处理规则。

### Todo 6：MCU 工程接入模板

目标：

```text
总结一套 MCU 工程接入模板。
```

要实现：

- 给出 `init()`、`step()`、输入采样、输出更新的 C 调用框架。
- 说明如何在固定周期中断中调用生成函数。
- 说明结构体输入输出如何和 ADC/PWM/FOC 变量对接。
- 明确采样周期必须和 Simulink `FixedStep` 一致。
- 形成一个最小 MCU wrapper，例如：

```c
void MotorControl_Init(void);
void MotorControl_10kHz_ISR(void);
```

### Todo 7：MBD 文档与 AI 继承机制

目标：

```text
让每个里程碑都有 README、测试入口、生成代码说明、经验回写。
```

要实现：

- 每个新例子都要有 README。
- 每个关键模型都要有功能测试。
- 每次形成新判断，都要更新本文件。
- 重要工作流同步沉淀到 `mbd-simulink-codegen` skill。
- 后续 AI 进入仓库时，默认先读本文件。

## 里程碑：ID 阶跃电流闭环集成

目录：

```text
motor_current_loop_mbd/
```

目标：

```text
把 CurrentPiStep、MotorClarkeParkStep、DqToAbcDutyStep 接入
Average-Value Inverter + Surface Mount PMSM 仿真环境。
第一版只做 id_ref 阶跃，iq_ref = 0。
```

架构：

```text
id_ref step, iq_ref = 0
  -> CurrentPiStep
  -> DqToAbcDutyStep
  -> Average-Value Inverter
  -> Surface Mount PMSM
  -> plant_feedback_t
  -> MotorClarkeParkStep
  -> id_meas / iq_meas
  -> CurrentPiStep
```

关键结论：

- 这是集成模型，不是把所有东西揉成一个巨型算法模块。
- 可生成代码候选是 `CurrentPiStep`、`MotorClarkeParkStep`、`DqToAbcDutyStep`。
- `Average-Value Inverter` 和 `Surface Mount PMSM` 是仿真环境。
- 第一版选择 `id_ref` 阶跃、`iq_ref = 0`，因为 SPMSM 在 `iq = 0` 时不产生主要平均转矩，能先验证 d 轴电流闭环而不引入机械运动。
- 控制器周期保持 `50us`，inverter/plant 周期保持 `25us`。
- 多速率边界继续使用 Rate Transition。

当前测试命令：

```bash
matlab -batch "run('motor_current_loop_mbd/run_current_loop_id_step_smoke_test.m')"
```

当前测试结果：

```text
id_ref final  = 3 A
id_meas final = 3 A
iq_meas range = [0, 0] A
vd_ref range  = [0, 4.15697] V
vq_ref range  = [0, 0] V
wm max abs    = 0 rad/s
duty range    = [0.454151, 0.545849]
Current-loop id-step smoke test passed.
```

下一步建议：

- 增加 dq 电压矢量限幅，替代独立 `vd/vq` 轴限幅。
- 增加积分器状态硬限幅作为 anti-windup 之外的兜底保护。
- 再做 `iq_ref` 阶跃测试，最好先锁转子或明确机械负载边界。
- 最后再进入完整 FOC：速度环、电流环、SVPWM、plant 闭环。

## 里程碑：Current PI 饱和 / Anti-Windup 测试

目录：

```text
motor_current_pi_mbd/
```

新增测试：

```text
build_current_pi_saturation_test_model.m
run_current_pi_saturation_test.m
```

测试思想：

```text
CurrentPiStep -> 简单一阶 q 轴电流对象 -> iq_meas -> CurrentPiStep
```

输入：

```text
Vdc = 12V
iq_ref = 0A -> 30A -> 0A
```

对比：

```text
Kaw_iq = 400   默认 back-calculation anti-windup
Kaw_iq = 0     等效关闭 anti-windup
```

当前结果：

```text
With anti-windup:
  final iq                 = 6.05617e-07 A
  release saturation count = 39
  release recovery time    = 0.00675 s
  release |vq| area        = 0.021076 V*s

Without anti-windup:
  final iq                 = 6.924 A
  release saturation count = 561
  release recovery time    = Inf s
  release |vq| area        = 0.193872 V*s
```

结论：

- anti-windup 的效果不能只看“输出有没有被限幅”，必须看“释放饱和后能不能恢复”。
- `Kaw_iq = 0` 时，积分器 windup 会让输出长时间停在正饱和附近。
- `Kaw_iq = 400` 时，释放后约 `6.75ms` 回到 `0.5A` 内。
- 以后每个 PI 模块都应有饱和保持 + 释放恢复测试。

## 里程碑：Speed PI MBD 模块

目录：

```text
motor_speed_pi_mbd/
```

目标：

```text
speed_pi_input_t
  -> SpeedPiStep
  -> speed_pi_output_t
```

接口：

```text
speed_pi_input_t {
  wm_ref
  wm_meas
  iq_limit
}

speed_pi_output_t {
  iq_ref
}
```

公式：

```text
error = wm_ref - wm_meas
iq_pre_sat = Kp_speed * error + integrator
iq_ref = clamp(iq_pre_sat, -iq_limit, iq_limit)
integrator_next = integrator + Ts * (Ki_speed * error + Kaw_speed * (iq_ref - iq_pre_sat))
```

设计来源：

- 速度环参数参考 `average-inverter/motor_control_params.m` 的带宽法。
- 速度环周期 `Ts_speed = 100us`。
- 电流环周期仍为 `50us`。
- 输出 `iq_ref` 作为 q 轴电流环输入。

当前测试命令：

```bash
matlab -batch "run('motor_speed_pi_mbd/run_speed_pi_smoke_test.m')"
```

当前测试结果：

```text
wm_ref final  = 41.8879 rad/s
wm_meas final = 41.8879 rad/s
speed error   = 0 rad/s
iq_ref range  = [-0.765256, 15] A
iq_limit      = 15 A
Speed PI smoke test passed.
```

代码生成命令：

```bash
matlab -batch "run('motor_speed_pi_mbd/generate_speed_pi_code.m')"
```

生成接口：

```c
extern void SpeedPiStep_Init(DW_SpeedPiStep_T *localDW,
                             P_SpeedPiStep_T *localP);

extern void SpeedPiStep(const speed_pi_input_t *rtu_speed_in,
                        speed_pi_output_t *rty_speed_out,
                        DW_SpeedPiStep_T *localDW,
                        P_SpeedPiStep_T *localP);
```

下一步：

```text
SpeedPiStep -> CurrentPiStep -> DqToAbcDutyStep -> Average Inverter -> PMSM
```

集成时要注意：

- 速度环 `100us` 到电流环 `50us` 是多速率边界。
- `iq_ref` 应通过 Rate Transition 进入电流环任务。
- 速度反馈 `wm_meas` 来自 plant 或速度估算器，进入速度环前也要明确采样边界。

## 里程碑：速度环 + 电流环 + 电机闭环集成

日期：2026-06-05

目录：

```text
motor_speed_current_loop_mbd/
```

目标：

```text
SpeedPiStep
  -> CurrentPiStep
  -> DqToAbcDutyStep
  -> Average-Value Inverter
  -> Surface Mount PMSM
  -> MotorClarkeParkStep feedback
```

这一步把前面已经沉淀的独立模块串成完整 float FOC 闭环仿真：

- `SpeedPiStep` 输出 `iq_ref`。
- `CurrentPiStep` 输出 `vd_ref/vq_ref`。
- `DqToAbcDutyStep` 输出 `[0, 1]` 的 `phase_duty_t`。
- Average-Value Inverter 和 Surface Mount PMSM 仍然是 plant/harness。
- `MotorClarkeParkStep` 把 plant 的 `ia/ib/ic/theta_e` 反馈成 `id/iq`。

采样时间：

```text
Plant / Average Inverter: 25us
Current loop:             50us
Speed loop:              100us
```

关键工程结论：

- 这是集成模型，不是一个新的巨型算法模块。
- 可复用/可生成代码候选仍然是速度 PI、电流 PI、dq duty、Clarke/Park 等子系统。
- 多速率边界必须显式表达：
  - `wm_feedback_rt_100us`：plant speed feedback -> speed loop
  - `IqRef_RateTransition_50us`：speed loop `iq_ref` -> current loop
  - `PhaseDuty_RateTransition_25us`：current loop duty -> inverter/plant
- `motor_speed_current_loop_interface.sldd` 同时管理 speed PI、current PI、motor、duty、feedback 的 bus/type/parameter 合同。
- 默认仿真时长设置为 `0.060s`，足够做 smoke test，不把每次回归都变成很长的性能仿真。
- MATLAB batch 仿真后要主动关闭模型和 `.sldd`，否则 Simulink/Scope/WebWindow 资源有时会拖慢退出。

当前测试命令：

```bash
matlab -batch "run('motor_speed_current_loop_mbd/run_speed_current_loop_smoke_test.m')"
```

当前测试结果：

```text
wm_ref final   = 41.8879 rad/s
wm_meas final  = 41.8883 rad/s
speed error    = -0.000385284 rad/s
iq_ref range   = [-0.949188, 15] A
iq_meas range  = [-0.879125, 14.6738] A
id_meas range  = [-0.169437, 0.371049] A
vd_ref range   = [-1.2048, 0] V
vq_ref range   = [0, 20.7848] V
duty range     = [0.235291, 0.764709]
iq_limit       = 15 A
Speed-current-loop smoke test passed.
```

下一步建议：

- 增加 dq 电压矢量限幅，避免独立 `vd/vq` 饱和带来的矢量幅值问题。
- 增加速度环和电流环参数扫频/阶跃对比脚本，区分 smoke test 与性能调参。
- 后续如果要生成嵌入式 C，优先生成 controller 子系统代码；plant/inverter 仍作为仿真验证环境。

## MBD 后续分阶段学习路线图

日期：2026-06-06

目标：

```text
把 MBD 从“能仿真、能生成 C”继续推进到
可验证、可移植、可标定、可维护、可交付的工程资产。
```

### Step 1：巩固算法模块和接口合同

要学习：

- `.sldd` 统一管理类型、Bus、参数。
- 模型边界只暴露物理量，不暴露寄存器。
- 输入输出使用稳定结构体合同，例如 sensor input、command input、duty output、status output。
- 生成代码保持 reentrant / reusable，不手改生成的 `.c/.h`。

当前仓库锚点：

```text
simple_add_reentrant/
motor_current_pi_mbd/
motor_speed_pi_mbd/
motor_speed_current_loop_mbd/
```

完成标准：

```text
任意模块都能说清：
输入是什么、单位是什么、范围是什么、输出是什么、采样周期是什么、生成接口是什么。
```

### Step 2：补 controller-only 代码生成

要学习：

- 只生成控制器代码，不生成 Average Inverter / PMSM plant。
- 明确哪些子系统属于 production code candidate。
- 检查生成头文件、类型文件、函数接口和状态结构体。
- 保持平台无关，不包含 TI/ST/NXP/AUTOSAR 具体头文件。

当前优先目标：

```text
SpeedPiStep
CurrentPiStep
DqToAbcDutyStep
MotorClarkeParkStep
```

完成标准：

```text
生成代码可以被 PC 测试程序、TI 工程、ST 工程或 AUTOSAR wrapper 调用。
```

### Step 3：建立 MIL / SIL / PIL / HIL 验证路线

要学习：

```text
MIL = Model in Loop
SIL = Software in Loop
PIL = Processor in Loop
HIL = Hardware in Loop
```

当前重点：

- 先做 MIL smoke test。
- 再做 SIL back-to-back test：Simulink 输出和生成 C 输出一致。
- 上板前再做 PIL，确认目标处理器上的代码行为一致。
- HIL 用于真实控制器和实时 plant 闭环。

完成标准：

```text
每个关键模块都有模型测试。
每次生成代码后，都能和模型输出做一致性比较。
```

### Step 4：补实时调度和多速率设计

要学习：

- 固定步长离散系统。
- 多速率任务。
- Rate Transition。
- ZOH / data hold。
- 任务延迟。
- ISR 调用顺序。
- WCET 和 jitter。

当前仓库锚点：

```text
Plant / Average Inverter: 25us
Current loop:             50us
Speed loop:              100us
```

完成标准：

```text
能写出平台无关调度框架：
25us PWM tick
50us current loop
100us speed loop
```

### Step 5：建立平台无关移植边界

要学习：

- TI、ST、NXP、AUTOSAR 都只能作为 platform adapter。
- MBD core 不依赖芯片寄存器。
- 输入使用物理量：A、V、rad、rad/s。
- 输出使用抽象执行量：duty、status、fault flag。

推荐结构：

```text
motor_control_core/
  generated/
  include/
  wrapper/

platform/
  ti_c2000_adapter/
  st_stm32_adapter/
  nxp_adapter/
  autosar_adapter/
  linux_sil_adapter/
```

完成标准：

```text
同一份生成 C 可以被不同 platform adapter 调用。
```

### Step 6：补数据类型、范围、饱和和定点

要学习：

- `Simulink.AliasType`
- `Simulink.NumericType`
- 信号单位和范围。
- overflow / rounding / saturation。
- PI 积分器 anti-windup 和兜底限幅。
- duty、电压、电流、角度的量纲规范。

当前优先规则：

```text
float-first 做架构。
定点后移，但类型合同提前设计。
角度和三角函数不要随便定点化。
```

完成标准：

```text
每个重要信号都有类型、单位、范围和越界处理策略。
```

### Step 7：补标定参数体系

要学习：

- 控制参数不能散落在脚本里。
- PI 参数、限幅、滤波、保护阈值应可标定。
- 后续可扩展到 A2L / XCP / 标定工具。

当前优先参数：

```text
Kp_speed
Ki_speed
Kaw_speed
Kp_id
Ki_id
Kaw_id
Kp_iq
Ki_iq
Kaw_iq
IqLimitDefault
VLimitRatio
```

完成标准：

```text
参数来源清晰，默认值可追踪，工程层可替换或标定。
```

### Step 8：补状态机和故障保护

要学习：

- Init
- OffsetCalibration
- RotorAlign
- OpenLoopStart
- ClosedLoopRun
- Fault
- Recover
- Stop

必须补的保护：

- 过流
- 过压
- 欠压
- 过温
- 编码器异常
- ADC 异常
- 速度失控
- duty 越界

完成标准：

```text
控制算法不再只有 PI 闭环，还具备上电、运行、故障、恢复的工程行为。
```

### Step 9：补需求、测试和追踪

要学习：

- 每个模块对应需求。
- 每个需求对应测试。
- 每次修改后回归测试。
- 重要错误写入 `MBD_ERRATA.md`。
- 重要结论写入本文件。

完成标准：

```text
任何后续 AI 或工程师进入仓库后，可以从文档接上路线，不靠口口相传。
```

### Step 10：补模型规范和可维护性

要学习：

- 命名规则。
- Bus 命名。
- 参数命名。
- 采样率标注。
- 模块边界。
- 禁止隐式类型转换。
- 禁止模型变成不可读的长线图。

完成标准：

```text
模型既能生成代码，也能长期维护。
```

## 当前最高优先级

下一阶段先做这五件事：

```text
1. 给 speed-current-loop 增加 controller-only 代码生成入口。
2. 检查生成 C 接口，确认平台无关。
3. 做 SIL back-to-back 测试。
4. 定义 motor_control_core 的输入输出 wrapper 合同。
5. 增加基础状态机和故障保护模块。
```

## 里程碑：团队可复用模块包

日期：2026-06-06

目录：

```text
motor_control_modules/
```

目标：

```text
把已经验证过的 MBD 控制模块整理成团队复用入口，
让同事从模块库、文档和测试入口使用，而不是复制 demo 模型里的 subsystem。
```

当前模块库：

```text
motor_control_modules/motor_control_lib.slx
```

库中模块：

```text
SpeedPiStep
CurrentPiStep
DqToAbcDutyStep
MotorClarkeParkStep
OpenLoopCommand
```

新增文件：

```text
motor_control_modules/README.md
motor_control_modules/setup_motor_control_modules.m
motor_control_modules/slblocks.m
motor_control_modules/build_motor_control_interface_dictionary.m
motor_control_modules/motor_control_interface.sldd
motor_control_modules/build_motor_control_module_library.m
motor_control_modules/tests/run_all_module_smoke_tests.m
motor_control_modules/docs/module_contracts.md
motor_control_modules/docs/reuse_integration_guide.md
motor_control_modules/codegen/README.md
```

使用方式：

```bash
matlab -batch "run('motor_control_modules/setup_motor_control_modules.m'); run('motor_control_modules/build_motor_control_interface_dictionary.m'); run('motor_control_modules/build_motor_control_module_library.m')"
```

验证：

```text
已生成 motor_control_interface.sldd。
已生成 motor_control_lib.slx。
motor_control_lib.slx 已挂载 motor_control_interface.sldd。
已添加 slblocks.m，可注册到 Simulink Library Browser。
已检查库中包含：
  CurrentPiStep
  DqToAbcDutyStep
  MotorClarkeParkStep
  OpenLoopCommand
  SpeedPiStep
```

库浏览器刷新命令：

```matlab
run('motor_control_modules/setup_motor_control_modules.m')
sl_refresh_customizations
```

库浏览器显示名：

```text
Motor Control Modules
```

复用原则：

```text
不复制 demo 模型中的 subsystem。
同事应从 motor_control_lib.slx 引用模块，或后续使用 Model Reference / Protected Model / generated C。
算法模块保持平台无关，不包含 TI/ST/NXP/AUTOSAR 寄存器或 HAL。
```

## 2026-06-10：12V 空心杯小惯量参数辨识

新增专题：

```text
identification/coreless_motor_12v_identification/README.md
```

目标：

```text
先用论文和工程资料建立小惯量参数辨识路线，
再在 MATLAB/Simulink 中做合成数据、plant、test harness 和估计算法。
```

核心判断：

- 空心杯小电机的 `J` 很小，直接使用 `J = Te / diff(speed)` 容易被速度噪声、采样延迟、摩擦和电压饱和带偏。
- 参数辨识不要从 `J` 开始，要先做 `R/L/Ke/Kt` 的 sanity check。
- 机械参数推荐使用：

```text
bidirectional torque pulse
  + position-based acceleration fitting
  + friction-aware least squares
```

机械模型：

```text
Te = J*a + B*w + Tc*sign(w) + Tbias
```

小惯量准确性规则：

```text
优先拟合位置求加速度，不直接差分速度。
正反向脉冲一起做，用来抵消偏置和摩擦影响。
如果原电机惯量太小、瞬态太快，增加已知惯量后辨识 J_total，再相减得到 J_motor。
12V 总线下要警惕 back-EMF 导致的电压饱和，不能把饱和阶段当成恒定 torque 数据。
```

与 HPM 硬件工作流的接口：

```text
/home/user/study/AI+MOTOR/HPM6E00EVK-RevC/INERTIA_IDENTIFICATION_PLAN.md
```

HPM CSV 分析时间基准继续使用：

```text
t_s = cmd_seq * 0.001
```

下一步：

```text
1. MATLAB 合成 12V 空心杯电机数据。
2. 用合成数据验证 estimator 是否能恢复 R/L/Ke/Kt/J/B/Tc。
3. 建立 Simulink plant 和 torque-pulse harness。
4. 再读取 HPM merged_result_csv 做实测辨识。
```

## 2026-06-10：电机性能检测与电流传感器谐波回归

新增专题：

```text
motor_performance_characterization/
```

当前已实现回归测试：

```text
motor_performance_characterization/run_current_sensor_harmonic_regression_test.m
```

记录的判断：

```text
电流 offset / 零漂       -> dq 中 1x electrical ripple
电流 gain mismatch       -> dq 中 2x electrical ripple
随机电流噪声             -> broadband noise
PWM 采样纹波             -> PWM frequency and sidebands
deadtime 电压误差         -> 常见 6x electrical torque ripple
```

验证命令：

```bash
matlab -batch "run('motor_performance_characterization/run_current_sensor_harmonic_regression_test.m')"
```

当前验证结果：

```text
offset_fault:
  h1_A = 0.0899382504
  h2_A ~= 0

gain_mismatch_fault:
  h1_A ~= 0
  h2_A = 0.0561248608
```

这个回归测试是后续电机性能检测体系的第一个 guardrail：不要先怀疑电机本体，先确认电流采样链路是否把 offset/gain/noise 注入到了 dq 电流里。

## 2026-06-10：PMSM 电参辨识与有感/无感角度协同

新增专题：

```text
identification/pmsm_electrical_parameter_identification/
```

目标：

```text
估计 Rs/Ld/Lq/psi_f
估计 encoder electrical offset
分析 encoder residual 1x/2x
为 sensorless observer 与 encoder angle 对齐做准备
```

当前方法：

```text
standstill d-axis voltage step -> Rs, Ld
standstill q-axis voltage step -> Rs, Lq
spin back-EMF / vq slope       -> psi_f
theta_encoder - theta_sensorless circular mean -> encoder offset
offset removal residual FFT/harmonic -> encoder nonlinearity
```

验证命令：

```bash
matlab -batch "run('identification/pmsm_electrical_parameter_identification/run_pmsm_electrical_id_demo.m')"
```

当前合成数据结果：

```text
Rs true / estimate   : 0.42 / 0.419958 ohm
Ld true / estimate   : 0.00025 / 0.000251283 H
Lq true / estimate   : 0.00036 / 0.000359821 H
psi true / estimate  : 0.018 / 0.0179997 Wb
encoder offset error : 1.17513e-05 rad
```

已新增 Simulink 波形观察模型：

```text
identification/pmsm_electrical_parameter_identification/pmsm_electrical_id_waveform_model.slx
```

生成命令：

```bash
matlab -batch "run('identification/pmsm_electrical_parameter_identification/build_pmsm_electrical_id_waveform_model.m')"
```

模型中包含：

```text
Standstill_RL_Step_Test
Flux_Linkage_Spin_Test
Encoder_Alignment_Test
```

测试条件记录：

```text
identification/pmsm_electrical_parameter_identification/results/pmsm_electrical_id_test_conditions.txt
```

当前测试点：

```text
RL step:
  d-axis: vd_step = 1.5 V, id starts near 0 A, we = 0 rad/s
  q-axis: vq_step = 1.8 V, iq starts near 0 A, we = 0 rad/s

Flux spin:
  id ~= 0 A
  iq ~= 0 A
  we = [100, 180, 260, 340, 420] rad/s

Encoder alignment:
  electrical turns = 6
  true offset = 0.35 rad
```

验证：

```text
已运行 sim('pmsm_electrical_id_waveform_model')，通过。
```

重要经验：

```text
不要直接用 noisy di/dt 做电感辨识第一版。
standstill voltage step 更适合用指数响应拟合 tau = L/R：
  i(t) = Iinf * (1 - exp(-t/tau))
  R = Vinf / Iinf
  L = R * tau
```

下一步：

```text
1. 接入真实台架 CSV。
2. 加入温度影响：Rs(T) 和 psi_f(T)。
3. 加入电感饱和：Ld(id,iq), Lq(id,iq) map。
4. 建立 sensorless angle 与 encoder angle 的在线对齐/健康监测模块。
5. 再进入 Simulink/MBD 模块化与代码生成。
```

## 2026-06-10：高占空比死区采样窗口要进入 MBD 接口层

问题背景：

```text
R = 4 ohm
L = 100 uH
PWM = 20 kHz
deadtime = 500 ns
```

在这个条件下，一个 PWM 周期内三相电流斜率变化很大。中心采样适合控制，但不适合盲目当作真实平均值；高占空比时还会出现低边采样窗口被压缩的问题。

已复现旧研究目录：

```text
average-inverter/switching_sampling_study/
```

复现结果：

```text
modulation ratio = 0.9
R = 4 ohm
L = 100 uH
RL sample error RMS = 0.302540 A
sampled phase ripple pk-pk = 3.348031 A
```

新增 MBD 模块：

```text
pwm_deadtime_sampling_mbd/
```

接口：

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

功能测试：

```text
usable_low = [45.500 0.500 23.000] us
sample_valid = [1 0 1]
all_samples_valid = 0
```

生成 C 接口：

```c
extern void DeadtimeSamplingWindowStep(const pwm_phase_duty_t *rtu_duty_in,
  pwm_sampling_status_t *rty_status_out);
```

重要边界修正：

```text
DeadtimeSamplingWindowStep 不是死区物理仿真。
它只负责采样窗口 valid 判定，适合生成 C 放进 ADC/current adapter。

真正分析死区对电机三相电流和电压误差的影响，需要开关级 plant：
  PWM + deadtime gates
    -> Universal Bridge MOSFET/Diodes
    -> switching PMSM / winding model
```

已补开关级 smoke test：

```text
average-inverter/switching_sampling_study/run_switching_deadtime_motor_smoke_test.m
```

当前测试结果：

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

当前工程判断：

```text
current_valid 不应该是电流环内部的附带判断，而应该是 ADC/current adapter 的接口合同。
PI、参数辨识和无感观测器都不应该默认相信高占空比下的所有电流采样点。
```

## 2026-06-10：死区补偿算法本身按 MBD/codegen 交付

新的边界：

```text
死区物理影响分析：
  average-inverter/switching_sampling_study/
  Universal Bridge + MOSFET/Diodes + PMSM plant

采样窗口 valid 判定：
  pwm_deadtime_sampling_mbd/
  DeadtimeSamplingWindowStep

死区 duty 补偿算法：
  pwm_deadtime_compensation_mbd/
  DeadtimeCompensationStep
```

这三个东西不能混用。采样窗口模块不等于死区补偿；死区补偿模块也不等于开关级物理仿真。正确做法是分层：

```text
FOC/SVPWM 输出 duty
  -> DeadtimeCompensationStep 生成补偿后 duty
  -> PWM/platform adapter 输出寄存器或门极
  -> 物理 plant/hardware 产生真实电流
  -> current_valid/ADC adapter 判断采样可信度
```

新增 MBD 模块：

```text
pwm_deadtime_compensation_mbd/
```

接口：

```text
pwm_deadtime_comp_input_t {
  da
  db
  dc
  id
  iq
  sin_theta_e
  cos_theta_e
}

pwm_deadtime_comp_output_t {
  da
  db
  dc
  comp_a
  comp_b
  comp_c
  active_a
  active_b
  active_c
}
```

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

当前默认值：

```text
Ts = 50 us
DeadtimeCompDuty = 0.01000
DeadtimeCompCurrentZero_A = 0.02 A
DeadtimeCompCurrentFull_A = 0.10 A
DeadtimeCompCurrentInvRange_1perA = 12.5 1/A
DeadtimeCompPolarity = -1
```

生成 C 接口：

```c
extern void DeadtimeCompensationStep(const pwm_deadtime_comp_input_t
  *rtu_comp_in, pwm_deadtime_comp_output_t *rty_comp_out);
```

验证命令：

```bash
matlab -batch "run('pwm_deadtime_compensation_mbd/run_pwm_deadtime_compensation_test.m')"
matlab -batch "run('pwm_deadtime_compensation_mbd/generate_pwm_deadtime_compensation_code.m')"
matlab -batch "cd('average-inverter/switching_sampling_study'); set(0,'DefaultFigureVisible','off'); run_switching_deadtime_motor_smoke_test"
```

当前结果：

```text
MBD functional test:
  input dq = [id=0.0 iq=0.2] A, theta_e = 0 rad
  synth current = [0.0 0.1732 -0.1732] A
  duty_out = [0.05000 0.94000 0.51000]
  comp = [-0.00000 -0.01000 0.01000]
  active = [0 1 1]

Switching plant smoke test:
  deadtime compensation: enable=1, duty=0.01000, update=50.00 us
  current source = dq_synthesized
  id/iq = [0.0 0.2] A
  current_zero/current_full = [0.02 0.10] A
  ia/ib/ic pk-pk = [0.2731 1.1862 1.1045] A
  deadtime comp range = a[-0 9.04e-06], b[-0.01 -0.01], c[0.01 0.01]
  sum current RMS = 0 A
```

工程判断：

```text
死区补偿属于用户级算法 core，应该 MBD 化。
优先用 dq 电流和电角度合成相电流极性，而不是直接依赖 ADC 相电流瞬时符号。
小电流区不硬判极性；过渡区平滑放大；大电流区按合成相电流极性满补偿。
开关 MOS、二极管、PMSM、电流纹波观察属于验证 harness，不强行 codegen。
当前补偿参数是编译期默认参数；如果后续要台架在线标定，应把 Simulink.Parameter
升级为 ExportedGlobal 或参数结构，而不是手改生成 C。
```

### 2026-06-11 补充：开关型验证必须跟随 MBD core 的接口语义

这次修正了一个重要继承点：

```text
pwm_deadtime_compensation_mbd/ 已经使用 id/iq/sin_theta_e/cos_theta_e
合成相电流极性。

average-inverter/switching_sampling_study/ 的开关 MOS + PMSM 验证 harness
也必须使用同一极性来源。
```

已完成修改：

```text
pwm_deadtime_compensation_mbd/build_pwm_deadtime_compensation_library.m
  从 pwm_deadtime_compensation_model/DeadtimeCompensationStep 生成模块包内库：
  pwm_deadtime_compensation_lib.slx/DeadtimeCompensationStep

motor_control_modules/build_motor_control_module_library.m
  将 DeadtimeCompensationStep 放入团队总库：
  motor_control_lib.slx/DeadtimeCompensationStep

build_switching_sampling_study_model.m
  插入 motor_control_lib.slx/DeadtimeCompensationStep
  只做接口适配：
    theta_e_deg -> sin_theta_e/cos_theta_e
    duty/id/iq/sin/cos -> pwm_deadtime_comp_input_t
    pwm_deadtime_comp_output_t -> double duty/comp/active logging

run_switching_deadtime_motor_smoke_test.m
  报告 deadtime_comp_current_source = dq_synthesized
  报告 id/iq、三相补偿 duty 范围、active 计数
```

当前验证：

```text
deadtime_comp_current_source = dq_synthesized
deadtime_comp_id_A = 0
deadtime_comp_iq_A = 0.2
ia/ib/ic pk-pk = [0.2731 1.1862 1.1045] A
deadtime_comp_range = a[-0 9.04e-06], b[-0.01 -0.01], c[0.01 0.01]
Result: PASS
```

长期规则：

```text
MBD core 的输入、Bus、类型或信号物理语义变化后，必须同步检查：
1. codegen 功能测试；
2. 集成模型；
3. 开关级/平均模型验证 harness；
4. README 和 progress 记录。

验证 harness 可以有 adapter，但不能复制算法内部实现。
复用模块应该来自库块或 Model Reference；算法只维护一份。
```

## 2026-06-11：平均模型、开关模型和 MBD 边界

新增专题文档：

```text
docs/modeling_scope_decision_guide.md
```

这次形成一个长期判断：

```text
平均电压模型是控制算法主干。
开关型模型是 PWM/ADC/器件细节的显微镜。
MBD 是可交付算法和稳定 adapter 的沉淀方式。
物理 plant、验证 harness 和探索脚本不需要强行 MBD/codegen。
```

平均电压模型适合：

```text
1. 电流环、速度环、FOC 主链路。
2. MTPA、弱磁、负载阶跃、速度阶跃等长时间仿真。
3. Clarke/Park、SVPWM/duty、PI、observer 等算法接口验证。
4. MBD 模块集成和 generated C 接口检查。
```

开关型模型适合：

```text
1. PWM 边沿和死区。
2. 高占空比低边采样窗口。
3. 中心采样与周期平均值误差。
4. MOSFET/Diode 导通路径、续流和器件非理想。
5. 零相量分配、DPWM、common-mode shift 对采样窗口和共模的影响。
6. 单电阻/双电阻采样重构可行性。
```

MBD 化对象：

```text
Clarke/Park
CurrentPiStep / SpeedPiStep
DqToAbcDutyStep / SVPWM/duty adapter
DeadtimeCompensationStep
DeadtimeSamplingWindowStep
observer / sensorless core
在线参数辨识 core
传感器校正、滤波、健康监测
```

保持原有仿真/研究形态的对象：

```text
Universal Bridge
MOSFET / Diodes
Simscape/SPS PMSM plant
RL winding physical model
powergui
Scope / To Workspace / plot/report
参数扫描脚本
真实 CSV 数据分析脚本
平台寄存器/HAL/驱动初始化
```

推荐工程流：

```text
平均模型先跑通控制。
开关模型专项验证 PWM/ADC/死区问题。
把验证得到的补偿、限制、valid 判定沉淀成 MBD 模块。
用 generated C 进入平台适配层。
硬件实测再反向修正参数和边界。
```

最重要的防错规则：

```text
不要用平均模型回答死区和采样窗口问题。
不要用开关型模型承担所有控制算法长时间迭代。
不要把物理 plant 强行生成 C。
不要把平台寄存器/HAL 放进算法 MBD core。
```
