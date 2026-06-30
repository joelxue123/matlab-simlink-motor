# MBD Future Direction

这是一份给后续接手者的简短结论。

## 结论

未来不是二选一。

```text
图形建模做表达
文本资产做底座
生成代码做交付
```

也就是说：

- `Simulink` 继续负责控制逻辑的可视化表达、审查和仿真。
- `interface.yaml/json` 负责用户可编辑接口合同。
- `.sldd`、类型、参数、Bus、构建脚本负责工具链接口和可复现性。
- `ERT + Reusable function + 可重入接口` 负责最终代码交付。

## 现在应该继续沿用的东西

### `[KEEP]`

- `.sldd`
- `interface.yaml/json -> .sldd` 的接口合同流程
- `Simulink.AliasType`
- `Simulink.NumericType`
- `Simulink.Bus`
- `Simulink.Parameter`
- `fixed-step discrete solver`
- `Rate Transition`
- `ERT`
- `Reusable function`
- 每个模块自己的 `README` 和 smoke test

### `[KEEP]` 的目录

- `motor_*_mbd/`
- `motor_control_modules/`
- `motor_clarke_park_struct/`
- `simple_add_reentrant/`

这些目录已经比较接近真正的 MBD 主线，后续新模块应该优先参考它们。

## 需要保留但不要扩散的东西

### `[RESEARCH]`

- `average-inverter/`
- `switching_sampling_study/`
- `discretization_compare/`
- `zero_response_study/`
- `adc_interrupt_current_loop_test/`

这些目录适合做机理分析、专题验证、波形对比，不适合直接当成新的交付模板。

### `[LEGACY]`

- `grt.tlc` 作为嵌入式交付主线
- `Nonreusable function`
- 核心逻辑长期依赖散落的 `MATLAB Function`
- 不进 `.sldd` 的 base workspace 接口
- 手工修改 generated C

这些可以维护、可以参考，但不应继续扩散成默认做法。

## 对当前团队的建议

1. 新模块优先按 MBD 标准建模，不再从旧式脚本/黑盒函数开始。
2. 图可以手工搭，但接口必须文本化、类型化、可复现。
3. 产品交付链路必须能从模型追到生成代码，再追到固件接入。
4. 研究目录和交付目录要分清，避免把实验实现误当成标准实现。
5. 后续 AI 接手时，先读 `docs/current_mbd_landscape.md`，再动模型。

## 一句话

```text
未来是 MBD 主导，但 MBD 不是“只画图”，而是“图 + 类型合同 + 自动生成 + 可追踪交付”。
```
