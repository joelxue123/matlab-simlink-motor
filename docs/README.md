# matlab-practice Docs

这个目录是 `matlab-practice/` 的长期协作入口。以后人或 AI 进入本仓库，先读这里，再进入具体模型目录。

目标不是把所有知识都塞进一个文件，而是固定开发秩序：

```text
先看进度 -> 再看规范 -> 再复用模块 -> 最后修改具体模型
```

## 阅读顺序

1. `progress.md`

   - 当前已经完成什么。
   - 每个里程碑在哪里、怎么运行、验证结果是什么。
2. `model_development_standard.md`

   - 新建 MBD 模块时必须遵守的工程规范。
   - 包括目录结构、`.sldd`、Bus、类型、采样时间、代码生成和测试要求。
3. `mbd_interface_contract_standard.md`

   - 用户接口合同的长期纪律。
   - 明确 `interface.yaml/json -> .sldd -> Simulink -> generated C headers` 的流程。
   - 防止用户直接修改 build 脚本深处的 Bus、类型和参数。
4. `reusable_modules_usage.md`

   - 如何复用 `motor_control_modules/` 里的模块。
   - 如何确认复制出来的块仍然是 library link。
   - Simulink 复用、Model Reference、Protected Model、生成 C 复用分别适合什么场景。
5. `current_mbd_landscape.md`

   - 当前工程的 MBD 主线、研究线、旧技术线和产品化断点。
   - 给后续 AI 的目录分层地图和标签规则。
6. `mbd_future_direction.md`

   - 面向后续接手者的 MBD 未来方向摘要。
   - 明确哪些做法继续沿用，哪些要标成 legacy。
7. `digital_twin_architecture_plan.md`

   - `green-joint` 数字孪生的资产盘点和长期路线。
   - 明确哪些 MBD 模块、平均模型、开关级模型、辨识脚本应该如何组合。
   - 后续建立 twin harness 时先读，不要重新从零扫描目录。
8. `mbd_test_state_management_architecture.md`

   - MBD 测试状态统一管理的中长期架构。
   - 明确 Product Controller、Test Harness、Plant、Scenario Library 的边界。
   - 后续不要继续新建孤岛测试模型，先把测试状态收编到统一 harness。
9. `simulink_hang_troubleshooting.md`

   - Simulink Desktop 卡在“模型更新”时的排查入口。
   - 记录 `green_joint_current_loop_mbd` 的卡死根因、危险脚本模式和安全重建规则。
   - 后续 AI 在重建 `.slx/.sldd` 前必须先读。
10. `ai_collaboration_rules.md`

   - 给未来 AI 的协作规则。
   - 防止每次 AI 按自己的想法重建一套风格。

## Current Focus

- `green_joint_current_loop_mbd/`
- `green_joint_digital_twin/`
- `green-joint/docs/current_loop_mbd_replacement_plan.md`
- `docs/digital_twin_architecture_plan.md`
- `docs/mbd_test_state_management_architecture.md`
- `docs/simulink_hang_troubleshooting.md`

## 仍然重要的根目录文档

- `../DOCS.md`：仓库级协同入口和交接记录。
- `../MBD_DEVELOPMENT_NOTES.md`：MBD 学习和工程判断的长笔记。
- `../MBD_ERRATA.md`：已经犯过的错误、修正证据和预防规则。
- `../motor_control_modules/docs/module_contracts.md`：可复用控制模块接口合同。
- `../motor_control_modules/docs/reuse_integration_guide.md`：控制模块复用流程。

## 本仓库的主线

```text
MATLAB/Simulink 仿真
  -> MBD 模块化
  -> interface.yaml/json 用户接口合同
  -> .sldd 接口合同
  -> 可重入 Embedded Coder 代码
  -> 平台适配层接 TI/ST/NXP/AUTOSAR/自研工程
```

控制算法核心保持平台无关。ADC、PWM、编码器、寄存器、HAL、RTOS 调度属于平台适配层，不放进算法模块。

## 每次重要修改后的要求

- 更新 `progress.md`。
- 如果改了建模方法，更新 `model_development_standard.md`。
- 如果改了模块复用方式，更新 `reusable_modules_usage.md`。
- 如果发现以前判断错了，更新 `../MBD_ERRATA.md`。
- 如果产生新的长期工程判断，追加到 `../MBD_DEVELOPMENT_NOTES.md`。
