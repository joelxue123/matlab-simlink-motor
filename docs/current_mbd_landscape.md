# Current MBD Landscape

本文件写给后续接手 `matlab-practice/` 的 AI 和工程协作者。

目标不是重复规范，而是回答三个实际问题：

1. 当前工程里的 MBD 主线到底是什么。
2. 哪些目录应该继续沿用。
3. 哪些目录属于旧技术、研究专用或过渡兼容，不应再被当成默认模板。

## 先看结论

当前工程不是一条完全统一的产品型 MBD 流程，而是四层并存：

```text
工程规范层
  -> 现代可交付 MBD 模块层
  -> 研究/验证模型层
  -> 手写固件集成层
```

真正应该继续扩展的新主线是：

```text
.sldd
+ Simulink.AliasType / NumericType / Bus / Parameter
+ fixed-step discrete solver
+ 显式 Rate Transition
+ ERT reusable/reentrant codegen
+ 模块 README + smoke test
```

## 标签规则

后续 AI 在阅读或修改目录时，使用下面标签理解工程角色：

- `[KEEP]`
  当前推荐主线。新模块、新交付能力优先沿用。
- `[TRANSITION]`
  有价值，但仍处于过渡期。可以保留并逐步向主线收口。
- `[LEGACY]`
  旧技术或旧工程形态。允许维护、允许参考，但不要当成新模块模板。
- `[RESEARCH]`
  研究/验证用途。它的价值在于仿真或验证，不要求按交付模块标准彻底改造。
- `[PRODUCT GAP]`
  当前产品化链条中的缺口。看到这类项时，说明模型和固件之间还没有真正闭环。

## 总体判断

### [KEEP] 工程规范与协作入口

这些文件定义了当前认可的 MBD 工程标准，应继续沿用：

- `docs/README.md`
- `docs/progress.md`
- `docs/model_development_standard.md`
- `docs/reusable_modules_usage.md`
- `docs/ai_collaboration_rules.md`
- `MBD_DEVELOPMENT_NOTES.md`
- `MBD_ERRATA.md`

理由：

- 已经明确新项目不用 MPT 作为默认主线。
- 已经明确接口合同优先使用 `.sldd`。
- 已经明确嵌入式交付优先使用 `ert.tlc + Reusable function`。
- 已经明确多速率边界显式使用 `Rate Transition`。

后续 AI 不应绕开这些文档，直接凭个人习惯重建风格。

## 目录分层地图

### [KEEP] 现代可交付 MBD 示例与模块

这些目录是当前最接近“可交付型 MBD 主线”的内容：

#### `simple_add_reentrant/`

角色：

- 最小可重入 ERT 示例。

保留原因：

- 适合作为 `AliasType + .sldd + Reusable function` 的最小教学样例。
- 适合给新 AI 解释“什么叫稳定类型合同”和“什么叫可重入接口”。

结论：

- 继续保留。
- 只作为最小模板，不作为电机控制复杂模块的唯一结构参考。

#### `motor_clarke_park_struct/`

角色：

- 结构体接口、定点类型和 Clarke/Park 变换的现代示例。

保留原因：

- 使用 `.sldd + Bus + NumericType`。
- 已有功能测试入口。
- 适合给后续 AI 解释“接口合同”和“功能模块边界”。

结论：

- 继续沿用。

#### `motor_current_pi_mbd/`
#### `motor_speed_pi_mbd/`
#### `motor_float_open_loop_mbd/`
#### `motor_current_loop_mbd/`
#### `motor_speed_current_loop_mbd/`

角色：

- 电机控制主线上的现代模块化样板。

保留原因：

- 构建脚本可重复运行。
- 使用 `.sldd` 管接口和参数。
- 使用 `ERT`。
- 使用 `Reusable function`。
- 模块 README 和 smoke test 比较完整。
- 明确表达采样时间和多速率边界。

结论：

- 这些目录是后续新模块最应该模仿的模板。
- 新的控制器交付模块应优先参考这组目录，而不是参考 `average-inverter/` 根目录下的旧式脚本组织。

#### `motor_control_modules/`

角色：

- 团队共享控制模块库。

保留原因：

- 已经形成共享 dictionary 和 library。
- 适合作为团队复用入口，而不是从旧模型里复制一团内部子系统。

结论：

- 继续沿用。
- 后续 AI 若要复用控制模块，应优先检查 library link，而不是复制旧块。

### [TRANSITION] 现代 MBD 主线与研究主线之间的桥接目录

#### `pwm_deadtime_compensation_mbd/`
#### `pwm_deadtime_sampling_mbd/`
#### `fixed_point_pi_q14_simulink/`

角色：

- 已经采用较现代的脚本化建模和代码生成思路，但定位更偏专题模块或实验模块。

保留原因：

- 具备继续演化成标准模块的基础。
- 对死区补偿、采样窗口、定点控制等专题很有价值。

限制：

- 它们不是当前控制主链的统一共享接口中心。
- 需要按实际交付需求进一步和 `motor_control_modules/` 或主控制链接口收口。

结论：

- 保留并逐步吸收进主线。

### [RESEARCH] 研究/验证主平台

#### `average-inverter/`

角色：

- 研究型主平台。
- 用于控制结构探索、物理现象分析、专题试验和 study 归档。

保留原因：

- 结构上已经区分了公共层和 `studies/`。
- 有大量研究成果和实验入口。
- 对 average-value inverter、开关采样、齿槽转矩、振动补偿等问题的分析很有价值。

限制：

- 大量子系统仍依赖 `MATLAB Function` 注入脚本。
- 主模型更多依赖 base workspace 配置和 `Goto/From` 标量组织。
- 它适合研究和快速验证，不适合作为“新的嵌入式交付模块默认模板”。

对后续 AI 的规则：

- 可以继续在这里做研究、验证和实验对比。
- 不要把这里的 `MATLAB Function + algorithms/*.m` 结构当成新交付模块模板。
- 如果某个算法在这里验证成熟，下一步应迁移成独立 `*_mbd/` 模块包。

#### `switching_sampling_study/`
#### `discretization_compare/`
#### `zero_response_study/`
#### `studies/*`

角色：

- 研究专题、实验对比、机理分析。

结论：

- 继续保留为 `[RESEARCH]`。
- 不应强行改造成共享控制模块。

### [LEGACY] 旧建模方式或旧交付风格

#### `dexterous_hand_impedance_plan/dynamics_only/simulink/`

角色：

- 旧一代控制器建模与 generated C 验证路径。

判断依据：

- 代码生成仍使用 `grt.tlc`。
- 核心控制器仍大量依赖 `MATLAB Function` block。
- 虽然有 SIL compare，但整体接口风格不是当前主线推荐样式。

保留原因：

- 仍有教学和历史验证价值。
- 其中的 `run_generated_c_sil_compare.m` 对“模型输出和生成 C 对比”仍有参考意义。

结论：

- 明确标记为旧路径。
- 可维护、可复用验证思路，但不要把它当作新模块默认模板。

#### `adc_interrupt_current_loop_test/`

角色：

- 中断触发和采样时序问题的专项验证模型。

判断依据：

- 使用 `grt.tlc`。
- 含 `MATLAB Function` 和触发式建模。
- 更偏时序机理研究，不是产品交付模块。

结论：

- 归类为 `[RESEARCH] + [LEGACY]` 的交叉地带。
- 保留其时序验证价值，但不要拿来作为标准控制模块骨架。

#### `主动阻尼控制/.../ar_algorithm.slx`

角色：

- 外部导入/历史遗留 Simulink 工程。

判断依据：

- 使用 `grt.tlc`。
- `CodeInterfacePackaging = Nonreusable function`。
- 含 Stateflow/XML 形式遗留工程结构。

结论：

- 明确标记为 `[LEGACY]`。
- 只可作为历史参考，不应作为新建模范式。

### [PRODUCT GAP] 手写固件集成层

#### `green-joint/`

角色：

- 当前实际产品/板级控制固件。

现状：

- 主控制链仍以手写控制和手写 PI/FOC 逻辑为主。
- 还没有看到 generated controller code 成为固件算法唯一事实来源。

典型表现：

- `Module/Src/foc.c`
- `Module/Src/pid_reg3.c`

结论：

- `green-joint/` 不是 MBD 源模型目录。
- 它是平台集成和实际产品行为基线。
- 当前它与 `matlab-practice/` 之间仍存在明显产品化断层。

对后续 AI 的规则：

- 不要误以为 “已有很多 MBD 模型 = 固件已经 MBD 化”。
- 如果任务涉及真正产品化，应优先关注“如何把 `*_mbd/` 模块生成代码接入固件”。

## 旧技术标识清单

后续 AI 在看到下面特征时，应主动标记为旧技术或非主线，而不是继续扩散：

### 明确的旧技术/旧形态信号

- `grt.tlc` 用于嵌入式交付主模块。
- `CodeInterfacePackaging = Nonreusable function`。
- 大量依赖 `MATLAB Function` block 承载核心控制逻辑。
- 关键接口不进 `.sldd`，只放 base workspace。
- 大量散落标量线，缺少 `Bus` 接口合同。
- 手工修改 generated C。
- 以 demo/导入工程为中心，而不是以可重复构建脚本为中心。

### 允许存在但不能扩散的旧知识

- `PIDREG3` 风格 PI。
- 手写 FOC/SVPWM 固件实现。
- 旧式 `MATLAB Function` 驱动的研究模型。
- 历史导入的 Stateflow/XML 工程。

这些内容可以维护、可以借鉴，但不应再被默认为新主线。

## 后续 AI 的默认动作

如果未来 AI 接到新任务，默认按下面顺序判断：

1. 如果目标是新建可交付控制模块：
   参考 `motor_current_pi_mbd/`、`motor_speed_pi_mbd/`、`motor_control_modules/`。
2. 如果目标是研究物理现象或专题验证：
   可以在 `average-inverter/` 或 `studies/` 内继续开展。
3. 如果目标是阅读旧项目或历史算法：
   允许参考 `dexterous_hand_impedance_plan/`、`主动阻尼控制/`，但不要复制其建模范式。
4. 如果目标是产品固件落地：
   必须同时看 `matlab-practice/` 的现代 MBD 模块和 `green-joint/` 的手写算法边界。

## 一句话交接结论

```text
当前真正该继承的是 modern *_mbd 模块主线；
average-inverter 继续做研究平台；
旧的 grt / MATLAB Function / Nonreusable 路线要明确打上 legacy 标识；
green-joint 已进入 MBD/adapter 分层主线：电流环、速度环、速度 PLL、MIT 已有
MBD 生成代码和固件 adapter；位置环、状态机和完整 unified test harness 仍需继续闭环。
```
