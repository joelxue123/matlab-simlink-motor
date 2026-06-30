# Reusable Modules Usage

本文件说明 `matlab-practice/` 中可复用模块的使用方法。核心目标是让同事和未来 AI 使用同一套模块，而不是从 demo model 里复制一份独立子系统。

## 当前共享模块库

目录：

```text
motor_control_modules/
```

共享数据字典：

```text
motor_control_modules/motor_control_interface.sldd
```

共享 Simulink library：

```text
motor_control_modules/motor_control_lib.slx
```

Library Browser 名称：

```text
Motor Control Modules
```

当前模块：

| Module | Input | Output | Baseline Sample Time | Role |
| --- | --- | --- | --- | --- |
| `SpeedPiStep` | `speed_pi_input_t` | `speed_pi_output_t` | 100us | 速度 PI，输出 `iq_ref` |
| `CurrentPiStep` | `current_pi_input_t` | `current_pi_output_t` | 50us | dq 电流 PI，输出 `vd_ref/vq_ref` |
| `DqToAbcDutyStep` | `open_loop_cmd_t` | `phase_duty_t` | 50us | dq 电压到 `[0, 1]` 三相 duty |
| `MotorClarkeParkStep` | `motor_t` | `motor_dq_t` | 50us | 三相电流到 alpha/beta/dq |
| `OpenLoopCommand` | internal constants | `open_loop_cmd_t` | 50us | 开环调试命令源 |
| `DeadtimeCompensationStep` | `pwm_deadtime_comp_input_t` | `pwm_deadtime_comp_output_t` | 50us | dq 合成相电流极性的死区 duty 补偿 |

## 当前独立 codegen 模块

有些模块暂时不放进 `motor_control_modules/` library，但已经按 MBD/codegen 方式沉淀，可直接作为 C core 交付或后续升级成 library/model reference。

| Module | Directory | Input | Output | Baseline Sample Time | Role |
| --- | --- | --- | --- | --- | --- |
| `DeadtimeSamplingWindowStep` | `pwm_deadtime_sampling_mbd/` | `pwm_phase_duty_t` | `pwm_sampling_status_t` | 50us | 低边采样窗口 valid 判定 |

使用原则：

```text
DeadtimeCompensationStep 已进入 motor_control_lib.slx，是团队共享交付算法。
DeadtimeSamplingWindowStep 是 ADC/current adapter 判定。
average-inverter/switching_sampling_study 是物理验证 harness。
```

## 首次构建

运行：

```bash
matlab -batch "run('motor_control_modules/setup_motor_control_modules.m'); run('motor_control_modules/build_motor_control_interface_dictionary.m'); run('motor_control_modules/build_motor_control_module_library.m')"
```

如果 Library Browser 已经打开，在 MATLAB 中运行：

```matlab
sl_refresh_customizations
```

然后在 Library Browser 中查找：

```text
Motor Control Modules
```

## Simulink 复用流程

推荐流程：

```text
1. 构建 motor_control_interface.sldd。
2. 构建 motor_control_lib.slx。
3. 在目标模型中 attach 所需 .sldd。
4. 从 Motor Control Modules 拖入需要的模块。
5. 在多速率边界加 Rate Transition。
6. 运行模块 smoke test。
7. 再运行集成模型测试。
```

不要从旧 demo model 里打开子系统内部再复制。这样会断开更新关系，形成隐藏 fork。

## 复制粘贴是否可以

可以，但来源必须是：

```text
motor_control_modules/motor_control_lib.slx
```

也就是说，从 library 窗口复制出来可以；从 demo model 或测试 harness 复制子系统内部不可以。

粘贴后选中模块，检查：

```matlab
get_param(gcb, 'LinkStatus')
get_param(gcb, 'ReferenceBlock')
```

期望：

```text
LinkStatus     = resolved
ReferenceBlock = motor_control_lib/<ModuleName>
```

如果：

```text
LinkStatus = none
```

或者没有 `ReferenceBlock`，说明已经变成独立副本，不适合作为团队复用模块。

## 可接受的复用形式

### Simulink Library Block

适合：

- 同事还在搭 Simulink 顶层模型。
- 需要可视化连接模块。
- 模块内部后续还会统一升级。

要求：

- 保持 library link。
- 不在目标模型里随意打断 link 后修改内部。

### Model Reference

适合：

- 模块边界稳定。
- 大模型需要增量编译。
- 多人并行开发。

要求：

- 接口 Bus 和 sample time 明确。
- 数据字典依赖清楚。
- model reference 的配置和 codegen 配置可重复。

### Protected Model

适合：

- 给外部团队使用但不暴露内部实现。
- 算法供应或跨团队交付。

要求：

- 接口文档必须完整。
- 必须提供测试 harness 或示例模型。

### Generated C + Platform Adapter

适合：

- 固件团队只需要嵌入式 C。
- 目标平台可能是 TI、ST、NXP、AUTOSAR 或自研平台。

边界：

```text
generated controller core:
  no registers
  no HAL
  no chip headers

platform adapter:
  ADC
  PWM
  encoder
  timer
  DMA
  interrupt
```

推荐调度形态：

```c
void MotorControl_25usTick(void)
{
    read_platform_inputs();

    if ((tick % 4U) == 0U) {
        run_speed_loop_100us();
    }

    if ((tick % 2U) == 0U) {
        run_current_loop_50us();
        run_dq_to_duty_50us();
    }

    write_platform_pwm(last_phase_duty);
    tick++;
}
```

## 接口合同

模块之间传递物理量，不传寄存器量：

```text
current: A
voltage: V
speed: rad/s
angle: rad
duty: [0, 1]
```

Average-Value Inverter 和 PWM adapter 使用：

```text
da/db/dc in [0, 1]
```

不要把 `[-1, 1]` modulation command 直接当 duty 输入，除非目标 block 的接口文档明确要求。

## 发布检查清单

模块分享给别人前，至少检查：

```text
1. README exists.
2. Smoke test passes.
3. Interface bus and type names are documented.
4. Sample time is documented.
5. Saturation and units are documented.
6. Generated code interface is checked if code delivery is required.
7. No chip register or HAL dependency exists inside the algorithm module.
8. docs/progress.md is updated.
```
