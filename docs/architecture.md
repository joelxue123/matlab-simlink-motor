# Average Inverter Architecture

## 目标

这个工程后续要解决的问题不是“再多写几个脚本”，而是把代码组织成两层：

- 公共库层：稳定、复用、尽量少改接口
- 实验层：每个算法验证单独成目录，允许快速试错和持续迭代

## 当前建议结构

### 1. 根目录只保留公共层

根目录以后优先放这些内容：

- 参数与配置
  - `motor_control_params.m`
  - `cogging_load_config.m`
  - `apply_cogging_load_config.m`
- 模型构建
  - `build_average_inverter_foc_model.m`
  - `build_vibration_comp_test.m`
  - `build_modules/`
- 算法函数
  - `algorithms/`
- 少量兼容入口
  - 已经被外部工作流依赖的旧入口可以暂时保留

根目录不要继续新增零散实验入口。

### 2. `studies/` 只放实验入口和实验文档

每个实验一个目录，例如：

- `studies/cogging_position_scan/`
- `studies/cogging_torque_comp/`
- `studies/vibration_comp/`

每个目录至少包含：

- 主入口函数
- README
- outputs

### 3. 输出按实验隔离

实验入口先切到该目录自己的 `outputs/`，再调用公共层。

这样做的好处：

- 不改原始实验逻辑
- 结果文件自动归档到当前实验目录
- 复现实验时不需要清理根目录垃圾文件

## 新增算法验证的规则

新增一个算法验证时，优先按下面顺序判断放哪里：

1. 如果只是某个现有实验的参数变体，放到对应 `studies/<name>/`。
2. 如果是一条新的验证链路，新建一个 `studies/<new_name>/`。
3. 如果是多个实验都会复用的函数，再放回根目录公共层或 `algorithms/` / `build_modules/`。

## 推荐命名

### Study 目录命名

- 用问题域命名，不用一次性结论命名
- 推荐：`cogging_position_scan`
- 不推荐：`best_scan_202605`

### 入口文件命名

- 目录内统一用短入口
- 推荐：`run_study.m`
- 可选：`reproduce_validation.m`
- 不建议继续在根目录扩散 `run_xxx_xxx_xxx.m`

## 渐进迁移建议

这次只建立新骨架，不强行移动所有旧文件。后续迁移建议按这个顺序：

1. 先把高频入口都通过 `studies/` 接起来。
2. 再把实验专属后处理脚本逐个迁入各自 study 目录。
3. 最后再清理根目录里明显只服务单个实验的旧文件。

这样风险最低，因为不会一次性打断现有 Simulink 研究流程。