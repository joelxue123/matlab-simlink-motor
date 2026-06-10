# MBD Errata

这个文件专门记录本仓库 MBD/Simulink/Embedded Coder 学习过程中的错误判断、修正依据和预防规则。

目标不是美化历史，而是让以后的人、我自己、以及其他 AI 不重复踩同一个坑。

## 使用规则

每次发现重要错误，都追加一条勘误。格式：

```text
编号：
日期：
状态：
错误判断：
正确结论：
证据：
影响：
修复：
预防规则：
```

严重程度建议：

```text
High    会导致模型接口、生成代码或仿真物理含义错误。
Medium  会导致工程语义不清、测试不稳或维护困难。
Low     主要是命名、文档或解释层面的偏差。
```

## Errata Index

| 编号 | 日期 | 严重程度 | 主题 | 状态 |
| --- | --- | --- | --- | --- |
| ERR-2026-06-05-001 | 2026-06-05 | High | Average-Value Inverter 输入接口误判 | Fixed |
| ERR-2026-06-05-002 | 2026-06-05 | Medium | 50us -> 25us 边界先用了 ZOH | Fixed |
| ERR-2026-06-05-003 | 2026-06-05 | Low | Rate Transition 测试未处理表达式采样时间 | Fixed |
| ERR-2026-06-05-004 | 2026-06-05 | Medium | 接口改名后 `.sldd` 可能残留旧类型 | Fixed |

## ERR-2026-06-05-001：Average-Value Inverter 输入接口误判

日期：2026-06-05

状态：Fixed

严重程度：High

错误判断：

```text
误以为 Motor Control Blockset 的 Average-Value Inverter 在本工程中接收 [-1, 1] modulation。
于是把新模型接口设计成：

phase_mod_t { a, b, c }
T_ModulationRatio
DqToAbcModStep 输出 2 * v_svpwm / Vdc
```

正确结论：

```text
本仓库参考的 Average Inverter 工作流接收 [0, 1] duty。
接口应为：

phase_duty_t { da, db, dc }
T_DutyRatio
DqToAbcDutyStep 输出 duty = v_svpwm / Vdc + 0.5
```

证据：

```text
average-inverter/speedloop_kf_test.slx
average-inverter/algorithms/dq2abc_fcn.m
```

其中 `dq2abc_fcn.m` 明确写了：

```matlab
da = va / Vdc + 0.5;
db = vb / Vdc + 0.5;
dc = vc / Vdc + 0.5;
```

临时仿真 `speedloop_kf_test` 后，实际 duty 范围为：

```text
all: min 0.199871913, max 0.800128087
```

影响：

```text
接口语义错误会改变施加到 Average Inverter 的等效电压。
scope 上仍可能看到“像电机在转”的波形，但物理含义不对。
更严重的是，错误命名会污染后续 Bus、typedef、README、skill 和生成代码接口。
```

修复：

```text
motor_float_open_loop_mbd/build_motor_float_open_loop_model.m
  DqToAbcModStep -> DqToAbcDutyStep
  phase_mod_t -> phase_duty_t
  T_ModulationRatio -> T_DutyRatio
  duty = v_svpwm / Vdc + 0.5

motor_float_open_loop_mbd/run_open_loop_smoke_test.m
  增加 log_da/log_db/log_dc。
  自动断言 duty 全程位于 [0, 1]。

motor_float_open_loop_mbd/README.md
MBD_DEVELOPMENT_NOTES.md
/home/user/.codex/skills/mbd-simulink-codegen/SKILL.md
  同步修正接口结论。
```

修复后测试结果：

```text
duty = [0.436322, 0.563678]
Open-loop smoke test passed.
```

预防规则：

```text
不要凭模块名字猜接口范围。
凡是连接库模块，必须先查已有可运行模型、官方 block contract 或仿真日志。
Bus/type 名字必须反映真实物理语义：duty 就叫 duty，modulation 就叫 modulation。
新接口必须加范围断言，例如 duty in [0, 1]。
```

## ERR-2026-06-05-002：50us -> 25us 边界先用了 ZOH

日期：2026-06-05

状态：Fixed

严重程度：Medium

错误判断：

```text
一开始为了让 Average Inverter 输入在 25us 更新，使用 Zero-Order Hold。
```

正确结论：

```text
控制算法 50us 到 inverter/plant 25us 是多速率任务边界。
工程化 MBD 中应优先用 Rate Transition 表达任务之间的数据交接。
```

证据：

```text
average-inverter/speedloop_kf_test.slx 中已有 RT_mod_to_plant。
```

修复：

```text
PhaseDuty_RateTransition_25us
Vdc_RateTransition_25us

Integrity = on
Deterministic = on
OutPortSampleTimeOpt = Specify
OutPortSampleTime = simcfg.Ts_plant
```

预防规则：

```text
ZOH 用于数学采样保持。
Rate Transition 用于多速率任务边界。
当问题描述是“控制周期 A 到 plant/PWM/inverter 周期 B”时，优先考虑 Rate Transition。
```

## ERR-2026-06-05-003：Rate Transition 测试未处理表达式采样时间

日期：2026-06-05

状态：Fixed

严重程度：Low

错误判断：

```text
测试里直接对 get_param(block, 'OutPortSampleTime') 做 str2double。
```

实际问题：

```text
Rate Transition 的采样时间参数保存的是表达式 simcfg.Ts_plant，
不是展开后的数字 2.5e-05。
```

修复：

```text
run_open_loop_smoke_test.m 中增加 evaluate_sample_time()。
如果参数文本是数字，直接解析。
如果参数文本是表达式，则 evalin('base', sample_time_text)。
```

预防规则：

```text
测试应该支持表达式化配置。
如果工程规范要求参数集中管理，就不要强迫模型参数都保存成裸数字。
```

## ERR-2026-06-05-004：接口改名后 `.sldd` 可能残留旧类型

日期：2026-06-05

状态：Fixed

严重程度：Medium

错误判断：

```text
接口从 phase_mod_t/T_ModulationRatio 改成 phase_duty_t/T_DutyRatio 后，
如果只是 upsert 新条目，旧条目可能仍残留在 .sldd。
```

正确结论：

```text
接口合同发生破坏性改名时，应重建数据字典，或显式删除旧条目。
```

修复：

```text
customer_interface_config():
  cfg.rebuildDictionary = true;

open_or_create_data_dictionary():
  如果 rebuildDictionary 为 true 且 .sldd 存在，则删除并重新创建。
```

验证：

```text
motor_float_interface.sldd 当前 Design Data 条目：

T_MotorFloat
T_MotorVoltage
T_MotorAngle
T_MotorCurrent
T_MotorSpeed
T_MotorTorque
T_DutyRatio
open_loop_cmd_t
phase_duty_t
dc_bus_t
plant_feedback_t
```

预防规则：

```text
新增字段或小改类型可以 upsert。
破坏性改名必须清理旧合同。
每次接口改名后，读取 .sldd 条目确认没有旧类型残留。
```

## 后续追加模板

```text
## ERR-YYYY-MM-DD-NNN：标题

日期：
状态：
严重程度：

错误判断：

正确结论：

证据：

影响：

修复：

预防规则：
```
