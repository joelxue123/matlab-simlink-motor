# MBD Interface Contract Standard

本文档定义后续 MBD 开发的接口纪律。目标是让用户、同事和后续 AI 不需要直接修改
`.m` 里的 `Simulink.BusElement`、`Simulink.AliasType`、`Simulink.Parameter`
细节，也不需要手工维护 `.sldd`。

## 核心结论

长期采用：

```text
interface.yaml / interface.json
  -> generator .m
  -> interface.sldd
  -> Simulink model
  -> generated C headers
```

权责分层：

```text
interface.yaml = 用户接口合同
generator .m   = 生成器，不是用户配置入口
interface.sldd = Simulink/Embedded Coder 数据字典产物
.slx           = 算法图和调度表达
*_ert_rtw/*.h  = C 工程交付接口
```

一句话纪律：

```text
用户改 interface.yaml，工具生成 .sldd，模型引用 .sldd，C 工程使用生成头文件。
```

## 当前推荐格式

第一推荐：

```text
YAML
```

原因：

- 适合人读和代码评审。
- 支持注释，方便记录单位、范围、标定含义和设计原因。
- Git diff 比 `.sldd` 清楚。
- 后续 AI 可以先读 YAML，快速理解模块外部合同。

允许：

```text
JSON
```

使用场景：

- 外部工具链只能稳定输出 JSON。
- 需要更严格的机器解析格式。
- 不需要大量注释。

禁止把下面这些作为长期用户接口：

- 直接让用户改 `.sldd`。
- 直接让用户改 build 脚本深处的 Bus/Parameter 代码。
- 在 base workspace 里临时声明接口变量。
- 在多个 block 参数里散落 `'single'`、`'fixdt(...)'`、裸字符串类型。

## 标准目录

新建可交付 MBD 模块时，优先使用：

```text
<module_name>/
  README.md
  interface.yaml
  build_<module_name>_model.m
  generate_<module_name>_dictionary.m
  run_<module_name>_smoke_test.m
  <module_name>_interface.sldd
  <module_name>.slx
```

如果生成器已经沉淀成共享工具，则模块内可以只保留：

```text
<module_name>/
  interface.yaml
  build_<module_name>_model.m
```

共享工具建议放在：

```text
tools/mbd_interface_codegen/
  generate_sldd_from_yaml.m
  validate_interface_contract.m
```

## interface.yaml 最小内容

每个接口文件至少描述：

- 模块名。
- 生成的类型头文件。
- `AliasType` 或 `NumericType`。
- 输入/输出 Bus。
- 可调参数 `Simulink.Parameter`。
- 单位、范围、默认值和说明。
- 参数存储方式。

示例：

```yaml
module: current_filter
dictionary: current_filter_interface.sldd
type_header: current_filter_types.h

aliases:
  - name: T_CurrentFilterCurrent
    base_type: single
    data_scope: Exported
    unit: A
    description: dq current feedback type

  - name: T_CurrentFilterAlpha
    base_type: single
    data_scope: Exported
    unit: "1"
    description: adaptive IIR coefficient

buses:
  - name: current_filter_input_t
    data_scope: Exported
    elements:
      - name: id_raw
        type: T_CurrentFilterCurrent
        unit: A
        description: raw d-axis current feedback
      - name: iq_raw
        type: T_CurrentFilterCurrent
        unit: A
        description: raw q-axis current feedback
      - name: v_mag_norm
        type: T_CurrentFilterAlpha
        unit: "1"
        min: 0
        max: 1
        description: normalized voltage magnitude

  - name: current_filter_output_t
    data_scope: Exported
    elements:
      - name: id_f
        type: T_CurrentFilterCurrent
        unit: A
      - name: iq_f
        type: T_CurrentFilterCurrent
        unit: A
      - name: alpha
        type: T_CurrentFilterAlpha
        unit: "1"

parameters:
  - name: AlphaLowVoltage
    type: T_CurrentFilterAlpha
    value: 0.95
    unit: "1"
    min: 0
    max: 1
    storage: ExportedGlobal
    description: alpha used below the low-voltage threshold

  - name: AlphaHighVoltage
    type: T_CurrentFilterAlpha
    value: 0.8
    unit: "1"
    min: 0
    max: 1
    storage: ExportedGlobal
    description: alpha used above the high-voltage threshold
```

## 字段规则

类型字段：

- `name` 必须是生成 C 中可接受的标识符。
- `base_type` 可使用 `single`、`double`、`boolean`、`int16`、`uint16` 等。
- 定点类型优先描述为结构化字段，不把 `fixdt(...)` 作为长期接口。
- `data_scope` 默认使用 `Exported`，除非明确只是模型内部类型。

Bus 字段：

- 字段名使用物理意义，不使用 `in1`、`tmp`、`data1`。
- 每个元素必须写 `type`。
- 对外接口元素必须写 `unit`。
- 需要给固件或客户看的字段必须写 `description`。
- 不把 platform ADC/PWM/HAL 细节塞进算法 core 的 Bus。

Parameter 字段：

- 可调参数必须进 `parameters`，不要写死在 block 里。
- 每个参数必须写 `value`、`type`、`unit`、`description`。
- 有物理边界时必须写 `min` 和 `max`。
- `storage` 必须明确，例如 `Auto`、`ExportedGlobal`、`ImportedExtern`。
- 标定参数优先使用 `ExportedGlobal` 或项目指定的 calibration storage class。

## 生成器职责

生成器必须做：

- 读取 `interface.yaml` 或 `interface.json`。
- 校验必填字段。
- 校验引用类型是否存在。
- 生成或更新 `.sldd`。
- 创建 `Simulink.AliasType`、`Simulink.NumericType`、`Simulink.Bus`、`Simulink.Parameter`。
- 设置 `DataScope`、`HeaderFile`、storage class。
- 在重复运行时保持可复现。

生成器不应该做：

- 把算法逻辑藏进接口文件。
- 静默忽略未知字段。
- 修改用户没有声明的接口项。
- 手工 patch 生成的 C/H 文件。

## 算法和接口的边界

接口文件只描述合同：

```text
类型
结构体
参数
单位
范围
默认值
导出方式
```

算法仍然放在：

```text
Simulink block diagram
build model script
library subsystem
```

例如：

```text
AlphaLowVoltage = 0.95
AlphaHighVoltage = 0.8
VoltageLowThreshold = 0.8
VoltageHighThreshold = 0.9
```

这些可以是接口参数。

但下面这个公式本身属于算法表达：

```text
alpha = interpolate(v_mag_norm, thresholds, alpha_limits)
```

它应该在 Simulink 图中用基础 block 表达，或在明确可审查的生成脚本中搭建。

## 开发流程

新模块流程：

1. 写 `interface.yaml`。
2. 运行接口生成器，生成 `.sldd`。
3. build 脚本创建或更新 `.slx`，并引用 `.sldd`。
4. 模型端口和内部 block 使用字典里的类型、Bus 和 Parameter。
5. 运行 smoke test。
6. 需要 C 交付时运行 ERT codegen。
7. 检查生成头文件里的 `typedef`、`struct`、函数签名和参数符号。
8. 更新模块 `README.md` 和 `docs/progress.md`。

修改接口流程：

1. 修改 `interface.yaml`。
2. 重新生成 `.sldd`。
3. 重新 build/update 模型。
4. 运行相关测试。
5. 检查生成 C 头文件是否符合预期。
6. 在 README 或 progress 中记录行为变化。

## 过渡规则

已有模块如果仍然在 `.m` 里手写接口，标记为：

```text
[TRANSITION]
```

含义：

- 可以继续维护。
- 新功能优先把用户可调的类型、Bus、Parameter 前移到 `interface.yaml`。
- 不要求一次性重写全部旧模块。
- 只要发生实质接口改动，就优先补 `interface.yaml`。

## 文档更新规则

这份文档是活的纪律文档。以后如果发现更好的长期方式，必须更新这里，而不是只在聊天里形成口头共识。

更新时必须说明：

- 旧规则哪里不够好。
- 新规则解决什么问题。
- 对已有模块是否需要迁移。
- 新模块从哪一天开始采用。

## 当前结论

当前最推荐方式：

```text
YAML as user contract
.sldd as Simulink contract
ERT generated headers as C contract
```

`.m` 脚本继续存在，但角色从“用户修改入口”降级为“生成器和模型构建工具”。
