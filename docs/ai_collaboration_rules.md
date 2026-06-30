# AI Collaboration Rules

本文件写给未来接手 `matlab-practice/` 的 AI，也写给以后回来复盘的人。目标是让每次协作沿着同一条工程主线前进，不要每个 AI 按自己的偏好重开一套风格。

## 进入仓库后的第一步

先读：

```text
docs/README.md
docs/progress.md
docs/current_mbd_landscape.md
docs/model_development_standard.md
docs/mbd_interface_contract_standard.md
docs/reusable_modules_usage.md
docs/digital_twin_architecture_plan.md
docs/simulink_hang_troubleshooting.md
MBD_DEVELOPMENT_NOTES.md
MBD_ERRATA.md
```

如果任务涉及共享控制模块，再读：

```text
motor_control_modules/docs/module_contracts.md
motor_control_modules/docs/reuse_integration_guide.md
```

如果任务涉及具体子项目，再读该子项目的：

```text
README.md
build_*.m
run_*_test.m
```

## 当前身份边界

本仓库当前 AI 的工作重心是：

```text
MATLAB/Simulink 仿真
MBD 模块化
Embedded Coder 可重入代码
电机控制算法
参数辨识和性能检测算法
```

低层寄存器、芯片驱动、板级 BSP、EtherCAT 主站工程可以作为接口背景，但不要把算法模块写成依赖具体 MCU 的形式。

## 开发默认路线

新 MBD 模块默认使用：

```text
interface.yaml / interface.json as user-facing contract
.sldd
Simulink.AliasType
Simulink.NumericType
Simulink.Bus
Simulink.Parameter
fixed-step discrete solver
Rate Transition at multi-rate boundary
ERT reusable/reentrant codegen when C delivery is needed
```

但本仓库采用有限 MBD：

```text
用户级算法、稳定接口、可交付 C core -> 按 MBD 标准。
物理 plant、开关器件、验证 harness、探索脚本 -> 按仿真/研究需求。
```

不要把 Universal Bridge、PMSM plant、scope/report harness 强行改成 MBD codegen 模块。它们的价值是复现物理现象和验证算法边界。

不要把新模块退回到：

```text
散落的 MATLAB Function block
base workspace 临时变量
到处手写 'single' / 'fixdt(...)'
让用户直接改 build 脚本深处的 Bus/Parameter
从 demo model 复制出来的无 link 子系统
手工修改 generated C
```

## 数字孪生任务规则

如果用户要求建立 `green-joint` 数字孪生，先读：

```text
docs/digital_twin_architecture_plan.md
green_joint_digital_twin/README.md
```

硬性纪律：

- 不要默认重新造模型。
- 重新造模型前必须先通知用户，并说明为什么不能直接复用主线。
- 可以为了简单验证新建临时模型，但必须标成 prototype/temporary/scratch。
- 临时模型验证完成后，必须把结论回归到统一 digital twin 主线。
- 不允许让临时验证模型继续漂在工程里，变成第二套速度环、第二套电流环或第二套 plant。

默认路线：

- 优先复用 `green_joint_current_loop_mbd/` 作为当前控制器 core。
- 优先参考 `motor_current_loop_mbd/`、`motor_speed_current_loop_mbd/` 和 `motor_float_open_loop_mbd/` 的平均 plant 结构。
- `average-inverter/` 是研究 plant 和算法验证来源，不是新的嵌入式交付模板。
- `switching_sampling_study/`、`adc_interrupt_current_loop_test/`、`pwm_deadtime_*_mbd/` 用于后续 PWM/ADC/死区专项增强，不放进 v0。
- 第一版 twin 只做最小闭环和日志对齐，不要一次接入完整 FOC、速度环、死区、齿槽和硬件 adapter。

一句话原则：

```text
先复用现有积木建立可比较的最小 twin，再逐层增加真实硬件细节。
临时模型可以帮助验证假设，但不能替代主线。
```

## Simulink Desktop 安全规则

如果任务涉及 `.slx/.sldd` 重建、模型更新或 Embedded Coder 代码生成，先判断当前是否可能有 MATLAB Desktop 打开同名模型。

默认规则：

- 优先使用 `matlab -batch` 跑 build、smoke test 和 codegen。
- 不要在 Desktop 打开模型时运行会删除/重建同名 `.slx/.sldd` 的脚本。
- 不要把 `open_system(...)`、`arrangeSystem(...)`、`SimulationCommand='update'` 放进默认 build 流程。
- 如果只是整理文档、接口或生成器，不要为了确认视觉布局而触发模型更新。
- 如果用户报告“模型更新”卡死，先按 `docs/simulink_hang_troubleshooting.md` 排查进程、线程、文件占用和 crash/log。

给后续 AI 的判断：

```text
batch smoke/codegen 能通过，而 Desktop 卡死：
  优先怀疑 Desktop UI、缓存、文件占用、自动 update/layout。

batch smoke/codegen 也卡死：
  再怀疑模型结构、数据字典、代数环、类型传播或第三方库。
```

## 修改模型前

先确认：

- 模型入口脚本是什么。
- 是否有 `.sldd`。
- 采样时间是多少。
- 输入输出 Bus 是什么。
- 是否已有 smoke test。
- 是否需要代码生成。
- 当前 git worktree 是否有用户未提交修改。

只修改与当前任务相关的文件。遇到已有未提交改动时，默认认为是用户或前序 AI 的工作，不要回滚。

## 新建模块时

按照：

```text
docs/model_development_standard.md
```

创建标准目录。至少包含：

```text
README.md
interface.yaml
build_<module>_model.m
generate_<module>_dictionary.m
run_<module>_smoke_test.m
<module>_interface.sldd
<module>.slx
```

需要 C 交付时，再加：

```text
generate_<module>_code.m
```

## 复用模块时

优先使用：

```text
motor_control_modules/motor_control_lib.slx
```

复制或拖入后检查：

```matlab
get_param(gcb, 'LinkStatus')
get_param(gcb, 'ReferenceBlock')
```

期望：

```text
LinkStatus = resolved
ReferenceBlock = motor_control_lib/<ModuleName>
```

如果只是从旧模型复制了一团内部子系统，那不是团队复用。

## 测试和验证

能运行 MATLAB 时，尽量运行对应测试，例如：

```bash
matlab -batch "run('<module>/run_<module>_smoke_test.m')"
```

如果任务是文档整理，可不运行 MATLAB，但要检查文档链接和 git diff。

仿真结果要记录：

- 运行命令。
- 关键数值。
- 输出图片或 CSV 路径。
- 是否通过。
- 仍然存在的限制。

## 文档继承规则

每次完成实质性工作后：

- 模块 README 记录入口和结果。
- `docs/progress.md` 记录里程碑。
- 发现错误写入 `MBD_ERRATA.md`。
- 改变工程共识写入 `MBD_DEVELOPMENT_NOTES.md`。
- 改变复用方法写入 `docs/reusable_modules_usage.md`。

一句话原则：

```text
模型可以重建，经验不能丢。
```

## Git 注意事项

当前父 repo 的远程 `main` 与 `average-inverter` 子 repo 历史不同。没有用户明确要求时，不要随意把 `matlab-practice` 推到远程 `main`。

已有安全推送方式曾使用：

```bash
git push git@github.com:joelxue123/matlab-simlink-motor.git main:matlab-practice-main
```

是否提交或推送，要听当前用户指令。
