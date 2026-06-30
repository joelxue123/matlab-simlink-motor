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
| ERR-2026-06-10-005 | 2026-06-10 | Medium | 把采样窗口 valid 模块表述成死区物理仿真 | Fixed |
| ERR-2026-06-11-006 | 2026-06-11 | High | 死区补偿 MBD core 与开关验证 harness 极性来源不一致 | Fixed |
| ERR-2026-06-11-007 | 2026-06-11 | High | 在验证 harness 中复制死区补偿算法而不是插入复用模块 | Fixed |
| ERR-2026-06-23-008 | 2026-06-23 | Medium | 把通用 68V 平均模型参数误用到 green-joint 12V twin | Fixed |
| ERR-2026-06-23-009 | 2026-06-23 | Medium | green-joint 电机参数应按线间 2ohm/55uH 统一换算 | Fixed |
| ERR-2026-06-24-010 | 2026-06-24 | Medium | green-joint 速度环周期误按 100us 记录 | Fixed |
| ERR-2026-06-24-011 | 2026-06-24 | High | green-joint 速度环 Kt 误用 17.03Vrms/krpm 推导值 | Fixed |
| ERR-2026-06-25-012 | 2026-06-25 | Medium | 高速电压受限仿真用 50us 显式欧拉导致 NaN | Fixed |
| ERR-2026-06-26-013 | 2026-06-26 | Medium | 误把 SpeedPiStep 的 Ki 调参入口当成离散 Ki | Fixed |
| ERR-2026-06-26-014 | 2026-06-26 | High | 速度环设计未把减速比接入主线等效惯量 | Fixed |
| ERR-2026-06-26-015 | 2026-06-26 | High | V1 速度参考漏接线导致速度环假通过 | Fixed |
| ERR-2026-06-26-016 | 2026-06-26 | Medium | 继承旧 average-inverter 阻尼导致微型电机速度环假饱和 | Fixed |
| ERR-2026-06-26-017 | 2026-06-26 | Medium | 并行运行写同一 `.sldd` 的 V1 仿真导致参数污染 | Fixed |
| ERR-2026-06-29-018 | 2026-06-29 | High | uint8 reset Switch 阈值量化导致 PLL 首次不复位 | Fixed |

## ERR-2026-06-29-018：uint8 reset Switch 阈值量化导致 PLL 首次不复位

日期：2026-06-29

状态：Fixed

严重程度：High

错误判断：

```text
SpeedEstimatorPllStep 中 reset 是 uint8，Switch 使用 `u2 > 0.5`
即可表达 reset != 0。
```

正确结论：

```text
当 Switch 控制输入和阈值参数被生成成 T_SpeedEstimatorReset/uint8 时，
`0.5` 阈值可能被量化成 `1U`。固件首次 reset 只传 `1U`，
生成代码判断 `reset > 1U` 不成立，PLL 初始 theta_hat 不会对齐当前编码器角。
reset 输入如果用 uint8，应使用阈值 `0U` 搭配 `u2 > Threshold`，
或者在模型中显式做 `reset ~= 0` 的布尔比较。
```

证据：

```text
错误生成代码曾出现：
  omega_reset_switch_Threshold = 1U
  theta_reset_switch_Threshold = 1U

当前修复后的生成代码为：
  green-joint/Module/MBD/green_joint_speed_estimator/speed_estimator_pll_model_data.c
    不再包含 omega/theta reset switch threshold 参数

模型构建脚本已同步修正：
  motor_speed_estimator_mbd/build_speed_estimator_pll_model.m
    omega_reset_switch Criteria = 'u2 ~= 0'
    theta_reset_switch Criteria = 'u2 ~= 0'

当前生成代码为：
  if (rtu_speed_estimator_in->reset != 0)
```

影响：

```text
MBD PLL 首次启用时 theta_hat 可能从 EncoderOffsetRad/0 开始，
而不是当前电机机械角，导致速度估算初始误差、速度环瞬态抖动，
严重时会影响闭环切入。
```

修复：

```text
build_speed_estimator_pll_model.m 中两个 reset Switch 改为 `u2 ~= 0`。
重新生成的 SpeedEstimatorPllStep.c 直接判断 `reset != 0`。
固件 adapter 继续把任意非零 reset 归一成 `1U`。
```

预防规则：

```text
MBD 中所有 uint8/int8 控制量不要用 0.5 这类浮点阈值表达布尔语义。
优先显式比较 `reset ~= 0`；如果使用 Switch，则阈值必须能在目标整数类型下
无歧义生成，例如 uint8 reset 使用 `u2 > 0U`。每次 codegen 后必须检查
生成的参数表，而不是只看 Simulink 图。
```

## ERR-2026-06-26-017：并行运行写同一 `.sldd` 的 V1 仿真导致参数污染

日期：2026-06-26

状态：Fixed

严重程度：Medium

错误判断：

```text
可以并行运行两个 green_joint_digital_twin V1 方波仿真，用不同
GJDT_CURRENT_TUNING_CASE 对比参数。
```

正确结论：

```text
run_green_joint_average_motor_square_wave_test.m 会把当前 tuning 同步到
green_joint_average_motor_twin_interface.sldd。两个 MATLAB 会话并行写同一个
.sldd 时，后写入者会污染另一个仿真的参数，导致不同 case 输出完全相同或不可信。
```

证据：

```text
并行运行 1620 variant_default 和 kp1_ki20000 时，两者都输出：
  iq peak ~= +/-4.14 A
  Kp/Ki 显示不同，但波形指标完全相同

串行重跑后：
  variant_default: iq peak ~= +0.3229 / -0.3234 A
  kp1_ki20000:     iq peak ~= +/-4.14 A
```

影响：

```text
会把稳定的 800Hz variant default 误判成严重过冲，或者把危险调参误判成可用。
```

修复：

```text
V1 方波 tuning case 验证改为串行运行。
README 增加规则：会写同一个 .sldd 的 MATLAB/Simulink 验证不要并行跑。
```

预防规则：

```text
任何会修改 .sldd、.slx、生成目录或共享 results 文件的 MATLAB/Simulink 脚本，
默认串行运行。需要并行时必须为每个 worker 使用独立字典、独立模型副本和独立输出目录。
```

## ERR-2026-06-26-016：继承旧 average-inverter 阻尼导致微型电机速度环假饱和

日期：2026-06-26

状态：Fixed

严重程度：Medium

错误判断：

```text
green_joint_digital_twin 可以沿用 average-inverter 示例中的 motor.B = 1e-4 N*m/(rad/s)。
```

正确结论：

```text
对 1615/1620 微型电机，阻尼必须来自辨识或硬件日志。
在没有辨识数据前，green-joint V1 主线默认 motor.B = 0。
```

证据：

```text
速度阶跃 0 -> 4 rad/s joint-side、iq_limit=4A 时：
  motor.B = 1e-4:
    final joint speed = 0.570909 rad/s
    |iq| max = 2.0247 A
    voltage_mag_norm max = 1

  motor.B = 0:
    final joint speed = 4 rad/s
    final error = 0
    |iq_ref| max = 1.2496 A
    voltage_mag_norm max = 0.668392

motor.B = 1e-4 在 104.7rad/s 电机端速度产生约 0.0105N*m 阻尼转矩，
等价吃掉约 2A q 轴电流，导致速度被假阻尼钉住。
```

影响：

```text
速度环会被误判为电压/反电势饱和或 PI 参数过小，诱导错误调大 iq_limit/Kp/Ki。
```

修复：

```text
matlab-practice/green_joint_digital_twin/setup_green_joint_current_loop_twin.m:
  motor.B = 0.0;

文档要求后续 B 必须通过拖动测试、自由衰减或闭环日志辨识填入。
```

预防规则：

```text
Plant 参数不能从通用示例模型无审查继承到具体产品。
阻尼、负载惯量、摩擦、齿槽转矩必须标记数据来源；没有数据时用 0 做基线并记录待辨识。
```

## ERR-2026-06-26-015：V1 速度参考漏接线导致速度环假通过

日期：2026-06-26

状态：Fixed

严重程度：High

错误判断：

```text
V1 average motor twin 已经接入 SpeedPiModelRef，只要 GJDT_UseSpeedLoop=1，
速度阶跃测试就会进入速度环。
```

正确结论：

```text
build_green_joint_average_motor_twin_model.m 创建了 joint_speed_ref_step 和
speed_ref_to_speed_pi，但漏接：
  joint_speed_ref_step/1 -> speed_ref_to_speed_pi/1

因此 SpeedPiStep 一直看到 wm_ref = 0，速度阶跃脚本第一次出现 iq_ref=0、速度=0 的假通过。
```

证据：

```text
诊断仿真输出：
  gjavg_speed_ref: n=1, min=0, max=0
  gjavg_iq_ref:    min=0, max=0
  gjavg_joint_speed: final=0

补线后：
  |iq_ref| max = 1.2496 A
  final joint speed = 4 rad/s
```

影响：

```text
如果不检查内部日志，会把“速度环没工作”误判为“速度环稳定且无输出”。
```

修复：

```text
build_green_joint_average_motor_twin_model.m:
  add_line(model, 'joint_speed_ref_step/1', 'speed_ref_to_speed_pi/1', ...)

同时 speed_input_bus_creator 三根线命名为 wm_ref/wm_meas/iq_limit。
```

预防规则：

```text
新接 Model Reference 后，至少检查一次关键链路信号：
reference、controller output、plant feedback、switch selected output。
不要只看 sim() 是否成功返回。
```

## ERR-2026-06-26-014：速度环设计未把减速比接入主线等效惯量

日期：2026-06-26

状态：Fixed

严重程度：High

错误判断：

```text
速度环可以按 rotor-only J_motor 设计，后续再考虑减速器。
```

正确结论：

```text
green-joint 速度环接口使用减速器输出端速度 rad/s，
但 SpeedPiStep 输出的是电机端 q 轴电流 A。
因此设计从 joint-side speed error 到 motor-side Iq 的 PI 时，应使用：

  J_speed_loop = J_motor * gear_ratio + J_load_output / gear_ratio

不是 raw J_motor，也不是 J_motor * gear_ratio^2。
```

证据：

```text
setup_green_joint_current_loop_twin.m:
  motor.gear_ratio = 183.35
  motor.speed_loop_equiv_inertia_kg_m2 =
      motor.rotor_inertia_kg_m2 * motor.gear_ratio +
      motor.output_load_inertia_kg_m2 / motor.gear_ratio

design_green_joint_speed_loop.m 输出：
  J_motor = 3.4e-8 kg*m^2
  gear_ratio = 183.35
  J_speed_loop = 6.2339e-6 kg*m^2
  Kp_speed = 0.302884591
  Ki_speed = 19.0308001
```

影响：

```text
如果继续用 raw J_motor，Kp/Ki 会小约 183.35 倍。
速度环会表现为 Iq 输出过小、响应严重偏软，后续调参会失去物理量纲意义。
```

修复：

```text
design_green_joint_speed_loop.m 使用 motor.speed_loop_equiv_inertia_kg_m2。
新增 sync_green_joint_speed_loop_twin_parameters.m 写入 speed_pi_interface.sldd。
run_green_joint_average_motor_speed_step_test.m 在 V1 主线模型中验证速度阶跃。
```

预防规则：

```text
每个外环都必须先写清楚：
  feedback 在哪一侧
  command 作用在哪一侧
  中间传动比如何定义
  控制输出的物理单位

速度环/位置环不允许只按变量名 wm_ref 推断单位。
```

## ERR-2026-06-26-013：误把 SpeedPiStep 的 Ki 调参入口当成离散 Ki

日期：2026-06-26

状态：Fixed

严重程度：Medium

错误判断：

```text
因为当前控制器是 MBD 离散生成代码，速度环 Ki 调参值可能需要使用
Ki_discrete = Ki_continuous * Ts_speed。
```

正确结论：

```text
当前 SpeedPiStep 是离散执行、离散状态，但 Ki_speed 参数本身是连续域增益。
生成代码内部已经显式乘以 Ts_speed。

因此对象字典 OBJ_ID_VEL_KI / gj_mbd_speed_ki 应写连续域 Ki_speed，
例如 1615 当前 20Hz 候选为 19.0308001 A/rad，而不是 0.0095154。
```

证据：

```text
green-joint/Module/MBD/green_joint_speed_loop/SpeedPiStep.c:

  localDW->speed_integrator_state_DSTATE +=
      ((rtb_iq_ref - rtb_speed_pre_sat_sum) * speed_pi_model_P.Kaw_speed
       + speed_pi_model_P.Ki_speed * rtb_speed_error)
      * localP->speed_integrator_delta_Gain;

green-joint/Module/Src/green_joint_speed_loop_mbd_adapter.c:

  speed_pi_model_P.SpeedPiStep_e.speed_integrator_delta_Gain =
      tuning->speed_sample_time_s;

matlab-practice/motor_speed_pi_mbd/build_speed_pi_model.m:

  Solver = FixedStepDiscrete
  FixedStep = speed_pi_simcfg.Ts_speed
```

影响：

```text
如果把 0.0095154 写入 gj_mbd_speed_ki，生成代码还会再乘 0.0005，
实际积分作用会比设计值小 2000 倍，速度环会表现为积分极弱、稳态误差消除很慢。
```

修复：

```text
green-joint/docs/speed_loop_mbd_replacement_plan.md 已明确：
  Ki_speed / Kaw_speed 是连续域调参入口；
  SpeedPiStep 代码内部乘 Ts_speed = 0.0005。
```

预防规则：

```text
判断 MBD PI 参数口径时必须查看生成代码的积分器更新式。
只要生成代码中存在 integrator += (... Ki_speed * error ...) * Ts，
调参入口就是连续域 Ki；不要仅凭“模型是离散的”推断 Ki 已经乘过 Ts。
```

## ERR-2026-06-24-011：green-joint 速度环 Kt 误用 17.03Vrms/krpm 推导值

日期：2026-06-24

状态：Fixed

严重程度：High

错误判断：

```text
green-joint 速度环设计沿用了 motor.back_emf_vrms_per_krpm = 17.03 和 pole_pairs = 10，
通过 psi_f 推导：
  Kt = 1.5 * pole_pairs * psi_f = 0.199173548 N*m/A
```

正确结论：

```text
用户提供了更直接的电流/转矩规格：
  rated current = 0.4949 A
  rated torque = 2.56 mN*m
  peak current = 1.4847 A
  peak torque = 7.33 mN*m

据此：
  Kt_rated = 0.00256 / 0.4949 = 0.00517276217 N*m/A
  Kt_peak = 0.00733 / 1.4847 = 0.00493702431 N*m/A

两者相差约 4.66%，当前默认采用额定点 Kt_rated，与 green-joint 硬件参数文档保持一致。
```

证据：

```text
用户明确提供额定电流、额定转矩、峰值电流、峰值转矩。
这些数据比通用 back-emf 示例参数更贴近 green-joint 当前电机规格。
```

影响：

```text
速度环 PI 参数按以下公式设计：
  Kp_speed = 2 * zeta * wc * J / Kt
  Ki_speed = wc^2 * J / Kt

Kt 从 0.199173548 修正到 0.00517276217 后，Kp/Ki 约放大 38.51 倍。
如果继续使用旧 Kt，速度环输出 Iq 会严重偏小，仿真和实机都会表现为速度响应过软。
```

修复：

```text
matlab-practice/green_joint_digital_twin/setup_green_joint_current_loop_twin.m
  记录 rated/peak current/torque
  motor.torque_constant = motor.kt_rated_nm_per_a
  motor.psi_f = motor.torque_constant / (1.5 * motor.pole_pairs)

matlab-practice/green_joint_digital_twin/design_green_joint_speed_loop.m
  torque_constant_nm_per_a = motor.torque_constant

本条只修正 Kt 来源；2026-06-26 的 ERR-2026-06-26-014 又补充了减速比等效惯量。
当前固件 adapter 默认值应以 ERR-2026-06-26-014 后的 gear-aware 参数为准。
```

修复后结果：

```text
green_joint_digital_twin/design_green_joint_speed_loop.m 输出：
  torque constant = 0.00517276217 N*m/A
  delay-limited speed BW = 55.906 Hz
  selected speed bandwidth = 20 Hz
  Kp_speed = 0.302884591 A/(rad/s)
  Ki_speed = 19.0308001 A/rad
  Kaw_speed = 125.663706 1/s
  P current at 4rad/s = 1.21153836 A
```

预防规则：

```text
速度环设计优先使用目标电机的 torque/current 规格或实测 Kt。
只有没有直接 Kt 数据时，才使用 back-emf 反推，并必须注明单位、RMS/peak、相电流/线电流假设。
若规格书电流不是 dq 轴等效相电流，必须先换算再进入 MBD 参数。
```

## ERR-2026-06-25-012：高速电压受限仿真用 50us 显式欧拉导致 NaN

日期：2026-06-25

状态：Fixed

严重程度：Medium

错误判断：

```text
高速电压受限场景可以直接用 50us 显式欧拉更新 dq 电流状态，
因为控制器电流环本身也是 50us。
```

正确结论：

```text
在 12V、接近最高速保护点、反电势接近可用电压的场景下，
电气方程刚性明显增强，50us 显式欧拉容易数值发散。
```

证据：

```text
原脚本在高反电势场景下出现 NaN/Inf。
改为按 V1 平均模型的 5us plant 子步长，并用离散精确状态更新后，
仿真稳定并复现了：
  baseline_no_iq_tracking_aw:
    time_to_iq_ref_below_0p5_s = 10.712 s
    iq_actual_mean_a = 0.042923 A
    voltage_norm_mean = 1.0
```

影响：

```text
如果继续用 50us 显式欧拉，后续所有“高速保护/电压受限/低电流”结论都可能被数值问题污染。
```

修复：

```text
1. 高速脚本增加 GJDT_TsPlant=5us 的 plant 子步长。
2. dq 电流状态改用离散精确更新，而不是 50us 显式欧拉。
3. 对非有限值立即报错，避免静默污染 CSV/图像。
```

预防规则：

```text
凡是接近反电势上限、弱磁边界、或高转速电气仿真的脚本，
必须优先检查 plant 步长和积分格式，不能默认与控制周期相同。
```

## ERR-2026-06-24-010：green-joint 速度环周期误按 100us 记录

日期：2026-06-24

状态：Fixed

严重程度：Medium

错误判断：

```text
早期 speed-loop MBD 文档、基础模块默认值和数字孪生设计脚本按 Ts_speed = 100us
记录 green-joint 速度环周期。
```

正确结论：

```text
green-joint 电流 ISR 为 20kHz，即 50us。
速度环在 Core/Src/main.c 中通过 ISR_Counter % 10 == 0 调用。

因此 green-joint 当前速度环实际周期是：
  Ts_speed = 10 * 50us = 500us
```

证据：

```text
green-joint/Core/Src/main.c:
  CONTROL_MODE_POSITION 和 CONTROL_MODE_VELOCITY 分支都在 ISR_Counter % 10 == 0 时调用 speed_control()。

green-joint/docs/current_loop_test.md:
  当前 ISR 是 20kHz。
```

影响：

```text
100us 会低估速度环离散延时，导致数字孪生中过于乐观地评估速度环带宽和相位裕度。
Kp/Ki 若按固定目标带宽直接计算不变，但 delay-limited speed bandwidth 会被明显高估。
```

修复：

```text
matlab-practice/green_joint_digital_twin/design_green_joint_speed_loop.m
  Ts_speed_s = 500e-6

matlab-practice/motor_speed_pi_mbd/build_speed_pi_model.m
  defaults.simcfg.Ts_speed = 500e-6

重新生成：
  motor_speed_pi_mbd/speed_pi_model.slx
  motor_speed_pi_mbd/speed_pi_interface.sldd
  motor_speed_pi_mbd/speed_pi_model_ert_rtw/*

同步到：
  green-joint/Module/MBD/green_joint_speed_loop/*
```

修复后结果：

```text
green_joint_digital_twin/design_green_joint_speed_loop.m 输出：
  delay-limited speed BW = 55.906 Hz
  selected speed bandwidth = 20 Hz
  Kp_speed = 4.29029462e-05 A/(rad/s)
  Ki_speed = 0.00269567161 A/rad
  Kaw_speed = 125.663706 1/s

注意：以上 Kp/Ki 只反映本条勘误当时的 Ts 修正结果，后续已被
ERR-2026-06-24-011 的 Kt 修正替代。

motor_speed_pi_mbd/run_speed_pi_smoke_test.m 通过：
  final speed error = 0 rad/s
  iq_ref range = [-0.923001, 15] A
```

预防规则：

```text
任何控制环 MBD 设计必须先从目标固件调度点确认实际调用周期。
不能只按 Simulink 模板或理想控制周期填写 Ts。
文档、build script、adapter 默认值和生成代码默认值必须同步。
```

## ERR-2026-06-23-009：green-joint 电机参数应按线间 2ohm/55uH 统一换算

日期：2026-06-23

状态：Fixed

严重程度：Medium

错误判断：

```text
green_joint_digital_twin 早期文档和默认参数曾混入模板/旧估算电机参数。
这会导致电流环 PI 的 Kp/Ki 不是从 green-joint 当前电机物理参数推导。
```

正确结论：

```text
green-joint 当前数字孪生默认使用用户确认的线间参数：
  line-to-line resistance = 2 ohm
  line-to-line inductance = 55 uH

进入 dq 相电气模型前统一换算：
  Rs = Rll / 2 = 1 ohm
  Ld = Lq = Lll / 2 = 27.5 uH
```

证据：

```text
用户确认：线电阻2欧姆，线电感55uh。

setup_green_joint_current_loop_twin.m 已写入：
  line_to_line_resistance_ohm = 2.0
  line_to_line_inductance_h = 55e-6
  GJDT_Rs_Ohm = line_to_line_resistance_ohm / 2.0
  GJDT_Ld_H = line_to_line_inductance_h / 2.0
  GJDT_Lq_H = line_to_line_inductance_h / 2.0
```

影响：

```text
电流环 PI 按物理量纲计算：
  Kp = L_phase * wc
  Ki = R_phase * wc

在 800Hz 带宽下：
  CurDKp = CurQKp ~= 0.13823 V/A
  CurDKi = CurQKi ~= 5026.55 V/(A*s)

如果继续使用旧参数，MBD 仿真会给出错误的带宽、限幅和 anti-windup 结论。
```

修复：

```text
green_joint_digital_twin/setup_green_joint_current_loop_twin.m
  使用 2ohm/55uH 线间参数，并统一换算到相参数。

green_joint_current_loop_mbd/interface.yaml
green_joint_current_loop_mbd/interface.json
  PI 参数更新为 0.13823 / 5026.55。

green_joint_digital_twin/README.md
  删除旧参数残留，明确 line-to-line -> phase 换算规则。
```

修复后结果：

```text
v0 dq plant:
  iq_ref final          = 3 A
  iq final              = 3 A
  iq tracking error     = 2.38419e-07 A
  id final              = 0 A
  vd final              = 0 V
  vq final              = 3 V
  voltage_mag_norm max  = 0.433276

v1 Average-Value Inverter + PMSM:
  iq_ref final          = 3 A
  id final              = 0.000295401 A
  iq final              = 0.557888 A
  vd final              = -0.249145 V
  vq final              = 6.91952 V
  wm final              = 47.9616 rad/s
  voltage_mag_norm max  = 1
```

预防规则：

```text
green-joint 数字孪生只在入口记录线间实测参数，控制器和 dq plant 只使用换算后的相参数。
README、interface.yaml/json、setup 脚本三处必须同步说明参数来源和量纲。
如果后续通过辨识更新 R/L，必须同时记录测量方式：line-to-line、phase 或 dq equivalent。
```

## ERR-2026-06-23-008：把通用 68V 平均模型参数误用到 green-joint 12V twin

日期：2026-06-23

状态：Fixed

严重程度：Medium

错误判断：

```text
建立 green_joint_digital_twin v1 时，沿用了 motor_current_loop_mbd /
motor_float_open_loop_mbd 里的通用平均电压模型示例参数：

inverter.Vdc = 68 V
GJDT_Vbus_V = 68 V
```

正确结论：

```text
green-joint 数字孪生默认母线电压应按 green-joint 12V 系统设置。
通用 motor_current_loop_mbd 的 68V 只是示例，不应直接套到 green-joint。
```

证据：

```text
green-joint/Core/Src/main.c
  Vbus 由 ADC 分压采样得到：
  Vbus = adc_raw * 3.3/4096 * (30+6.8)/6.8
  axis_.vbus_voltage = platform_adc_read_voltage() * VBUS_SCALE

matlab-practice/docs/progress.md 已有开关级验证基线：
  Vdc = 12 V

用户明确指出：参考 green-joint 目录下，电压是 12V。
```

影响：

```text
68V 不会让模型结构错误，但会让 voltage_mag_norm、限幅距离、电压利用率判断失真。
同样的物理 Vdq 指令在 68V 下看起来远离饱和，在 12V 下可能接近电压上限。
这会误导电流环带宽、anti-windup 和滤波策略判断。
```

修复：

```text
green_joint_digital_twin/setup_green_joint_current_loop_twin.m
  GJDT_Vbus_V = single(12.0)
  inverter.Vdc = double(GJDT_Vbus_V)

重新运行：
  run_green_joint_current_loop_twin_smoke_test.m
  run_green_joint_average_motor_twin_smoke_test.m

更新：
  green_joint_digital_twin/README.md
  docs/digital_twin_architecture_plan.md
  docs/progress.md
```

修复后结果：

```text
以下是修正 68V 错误时的历史结果；当前 green-joint R/L 参数已由
ERR-2026-06-23-009 更新为线间 2ohm/55uH。

v0 dq plant:
  iq_ref final          = 3 A
  iq final              = 2.8544 A
  vq final              = 0.610334 V
  voltage_mag_norm max  = 0.577123

v1 Average-Value Inverter + PMSM:
  iq_ref final          = 3 A
  iq final              = 1.09519 A
  vq final              = 4.8516 V
  wm final              = 34.7078 rad/s
  voltage_mag_norm max  = 0.701224
```

预防规则：

```text
新建项目 twin 时，不要直接继承通用示例里的 Vdc、motor、inverter 参数。
必须先从目标项目固件、板级文档、台架记录或用户说明确认母线电压和电机参数。
如果模型来自复用模板，README 必须明确哪些参数是模板值，哪些是目标项目事实。
```

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

## ERR-2026-06-10-005：把采样窗口 valid 模块表述成死区物理仿真

日期：2026-06-10

状态：Fixed

严重程度：Medium

错误判断：

```text
把 pwm_deadtime_sampling_mbd/ 中的 DeadtimeSamplingWindowStep 直接表述成“死区仿真”。
```

正确结论：

```text
DeadtimeSamplingWindowStep 是可生成 C 的采样窗口 valid 判定模块。
它根据 duty、PWM 周期、deadtime 和 ADC settle time 计算低边可采样窗口。
它不是开关 MOS + 电机 plant 的物理死区仿真。

真正观察死区对三相电流纹波、采样偏移和电机端电压的影响，需要：

center-aligned PWM + deadtime gates
  -> Universal Bridge MOSFET/Diodes
  -> switching PMSM / RL motor plant
  -> phase current logging
```

证据：

```text
pwm_deadtime_sampling_mbd/DeadtimeSamplingWindowStep 生成 C 的核心是：
  T_low_ideal = max((1 - duty) * T_pwm, 0)
  T_usable = max(T_low_ideal - 2 * dead_time - adc_settle_time, 0)
  valid = T_usable >= min_valid_window

它没有 MOSFET、二极管、母线、电机绕组、电流状态或反电势。
```

影响：

```text
如果把窗口判定模块当作死区物理仿真，会低估死区对实际相电流、电压误差、
采样点偏移和无感观测器输入的影响。
```

修复：

```text
1. 保留 pwm_deadtime_sampling_mbd/ 作为可生成 C 的 current_valid 判定模块。
2. 在 average-inverter/switching_sampling_study/ 中补充开关级 smoke test：
   run_switching_deadtime_motor_smoke_test.m
3. 该测试构建 Universal Bridge MOSFET/Diodes + SPS PMSM。
4. 文档中明确区分：
   - codegen adapter logic
   - switching physical plant simulation
```

验证：

```text
matlab -batch "cd('average-inverter/switching_sampling_study'); set(0,'DefaultFigureVisible','off'); run_switching_deadtime_motor_smoke_test"

通过结果：
  Vdc = 12 V
  PWM = 20 kHz
  deadtime = 500 ns
  R = 4 ohm
  L = 100 uH
  50us deadtime compensation polarity = -1
  ia/ib/ic pk-pk = [0.2844 1.1993 1.1058] A
  sum current RMS = 0 A
```

预防规则：

```text
包含 deadtime 字样的模块不一定是死区物理仿真。
凡是要回答“死区对电流/电压实际影响”，必须检查模型里是否真的包含开关器件和电机/绕组动态。
可生成 C 的控制/adapter 模块和不可生成 C 的物理 plant harness 要分层记录。
```

## ERR-2026-06-11-006：死区补偿 MBD core 与开关验证 harness 极性来源不一致

日期：2026-06-11

状态：Fixed

严重程度：High

错误判断：

```text
pwm_deadtime_compensation_mbd/ 已经改成用 id/iq/sin_theta_e/cos_theta_e
合成相电流极性后，开关型验证 harness 暂时仍可以继续把 plant 真实 ia/ib/ic
送入 DeadtimeDutyCompensator。
```

正确结论：

```text
如果可交付固件接口使用 dq 合成相电流判极性，开关型 MOS + PMSM 验证也必须使用同一极性来源。
plant 真实 ia/ib/ic 只能作为验证波形、误差分析和极性校准参考，不能再作为补偿算法输入。
```

证据：

```text
旧接线：
  MachineCurrentSelector -> I_A/I_B/I_C_comp_delay -> DeadtimeDutyCompensator

MBD core 接口：
  pwm_deadtime_comp_input_t { da, db, dc, id, iq, sin_theta_e, cos_theta_e }

新开关级验证报告：
  deadtime_comp_current_source = dq_synthesized
  deadtime_comp_id_A = 0
  deadtime_comp_iq_A = 0.2
  deadtime_comp_range = a[-0 -0], b[-0.01 -0], c[-0 0.01]
  Result: PASS
```

影响：

```text
如果验证 harness 和最终固件接口的极性来源不同，会出现“算法单测通过、开关级验证也看似通过，
但两者验证的不是同一个控制策略”的问题。
在小电流、零漂、噪声和高占空比采样偏移附近，这会误导死区补偿策略判断。
```

修复：

```text
第一版修复曾在 average-inverter/switching_sampling_study/build_switching_sampling_study_model.m
中重新搭建 DeadtimeDutyCompensator。

这个做法后来被 ERR-2026-06-11-007 纠正：验证 harness 不应该复制算法。
最终修复是插入团队总库中的复用模块：

motor_control_modules/motor_control_lib.slx/DeadtimeCompensationStep

开关型 harness 本地只保留：
  theta_e_deg -> sin_theta_e/cos_theta_e
  scalar signals -> pwm_deadtime_comp_input_t bus
  pwm_deadtime_comp_output_t bus -> plant/PWM double signals

average-inverter/switching_sampling_study/run_switching_deadtime_motor_smoke_test.m
  增加 deadtimeCompId/deadtimeCompIq 参数入口
  报告 current source、id/iq、补偿范围和 active 计数

README/progress/MBD notes 同步记录。
```

验证：

```text
matlab -batch "cd('average-inverter/switching_sampling_study'); set(0,'DefaultFigureVisible','off'); run_switching_deadtime_motor_smoke_test"

通过结果：
  deadtime_comp_current_source = dq_synthesized
  deadtime_comp_id_A = 0
  deadtime_comp_iq_A = 0.2
  ia/ib/ic pk-pk = [0.2731 1.1862 1.1045] A
  sum current RMS = 0 A
```

预防规则：

```text
MBD core 的输入语义变更后，必须同步检查：
1. 单元功能测试；
2. 生成 C 接口；
3. 集成/开关级验证 harness 的信号来源；
4. README 和 progress 记录。

不要只让算法模块“自己正确”。能交付的 MBD 模块必须和物理验证链路验证的是同一个策略。
```

## ERR-2026-06-11-007：在验证 harness 中复制死区补偿算法而不是插入复用模块

日期：2026-06-11

状态：Fixed

严重程度：High

错误判断：

```text
为了快速把开关型 MOS + PMSM 验证跑通，可以在
average-inverter/switching_sampling_study/build_switching_sampling_study_model.m
里重新搭一个 DeadtimeDutyCompensator 子系统。
```

正确结论：

```text
死区补偿已经是 pwm_deadtime_compensation_mbd/ 中的可交付 MBD core。
开关型验证 harness 应插入同一个复用模块，而不是复制算法内部实现。

验证 harness 可以有 adapter：
  theta_e_deg -> sin_theta_e/cos_theta_e
  scalar signals -> pwm_deadtime_comp_input_t bus
  pwm_deadtime_comp_output_t bus -> plant/PWM double signals

但不能重新实现 gain、sign、active、saturation 等算法逻辑。
```

证据：

```text
已新增：
  pwm_deadtime_compensation_mbd/build_pwm_deadtime_compensation_library.m
  pwm_deadtime_compensation_mbd/pwm_deadtime_compensation_lib.slx
  motor_control_modules/motor_control_lib.slx/DeadtimeCompensationStep

开关型模型当前插入：
  motor_control_lib.slx/DeadtimeCompensationStep

build_switching_sampling_study_model.m 中已不再存在：
  DeadtimeDutyCompensator
  local_add_deadtime_duty_compensator_subsystem
  local_add_deadtime_phase_path
  local_set_deadtime_synth_current_script
```

影响：

```text
如果每个 harness 都复制一份算法，后续 MBD core 改参数、接口、限幅、类型或 active 逻辑时，
验证模型很容易滞后，形成“看似验证通过，实际验证的是旧算法”的问题。
这会破坏模块化复用，也会让同事无法放心拖拽使用模块。
```

修复：

```text
1. 从 pwm_deadtime_compensation_model/DeadtimeCompensationStep 生成模块包内库：
   pwm_deadtime_compensation_lib.slx

2. 将 DeadtimeCompensationStep 加入团队总库：
   motor_control_modules/motor_control_lib.slx

3. 开关型验证模型插入团队总库块：
   motor_control_lib/DeadtimeCompensationStep

4. 只在 harness 中保留 adapter：
   - load .sldd types/buses into base workspace
   - override smoke-test parameters
   - create pwm_deadtime_comp_input_t bus
   - convert output bus fields to double for PWM/Scope/SPS plant

5. README、progress、MBD notes 同步更新为“复用模块插入”。
```

验证：

```text
matlab -batch "cd('average-inverter/switching_sampling_study'); set(0,'DefaultFigureVisible','off'); run_switching_deadtime_motor_smoke_test"

通过结果：
  deadtime_comp_current_source = dq_synthesized
  deadtime_comp_id_A = 0
  deadtime_comp_iq_A = 0.2
  ia/ib/ic pk-pk = [0.2731 1.1862 1.1045] A
  deadtime_comp_range = a[-0 9.04e-06], b[-0.01 -0.01], c[0.01 0.01]
  Result: PASS
```

预防规则：

```text
先问：这个算法有没有已经存在的 MBD core、library 或 Model Reference？
如果有，集成模型必须插入复用模块。
只有信号适配、类型转换、采样保持、plant 边界转换可以在 harness 本地实现。
```

## 后续追加模板

## ERR-2026-06-23-006：电流环调制余量不应写在固件 adapter 中

日期：2026-06-23
状态：已修复
严重程度：中

错误判断：

```text
GreenJointCurrentLoopStep 输出 vd_norm/vq_norm 后，
在 foc.c 里再执行：
  gd = pi.vd_norm * 0.9f
  gq = pi.vq_norm * 0.9f

把 legacy SVGEN 的调制余量当成 firmware adapter 的临时缩放。
```

正确结论：

```text
0.9 这类电压限幅/调制余量属于控制策略，应进入 MBD 模型和 .sldd 参数。
firmware adapter 只能搬运数据、同步参数、适配调用接口，不能隐藏控制律。
```

证据：

```text
MBD 已经定义 vd_norm/vq_norm = vd_cmd/vq_cmd / voltage_limit。
如果固件再乘 0.9，真实送入 foc_forward 的信号语义就不在 interface.yaml/json 中。
后续仿真、生成代码、固件行为会出现边界不一致。
```

影响：

```text
1. 数字孪生看到的是 vd_norm/vq_norm，硬件实际执行的是 0.9 * vd_norm/vq_norm。
2. 后续 AI 或同事读 MBD 接口时会遗漏固件中的隐藏限幅。
3. 电压利用率、饱和退出、带宽判断会被固件缩放污染。
```

修复：

```text
1. interface.yaml/json 增加 VoltageModulationRatio，默认 0.9。
2. MBD output 增加 vd_mod/vq_mod/voltage_mag_mod。
3. Simulink 模型中让真实 PI 电压限幅等于 vbus * VoltageLimitRatio * VoltageModulationRatio。
4. foc.c 直接使用 pi.vd_mod/pi.vq_mod，不再乘 0.9f。
5. codegen verification 检查 generated C 中存在 VoltageModulationRatio 和 vd_mod/vq_mod。
```

追加校正：

```text
上一版修复只把 0.9 放在最终输出换算中：
  vd_mod = vd_norm * VoltageModulationRatio

这虽然去掉了固件隐藏缩放，但 PI anti-windup 仍然按 vbus * 0.577 饱和，
不符合 ODrive 风格的 voltage_limit = 0.9 * vbus * 0.577。

正确做法是：
  physical_pi_limit = vbus * VoltageLimitRatio * VoltageModulationRatio
  vd_norm = vd_cmd / physical_pi_limit
  vd_mod = vd_cmd / (vbus * VoltageLimitRatio)

这样 0.9 参与 PI 饱和与 anti-windup，后级 SVGEN 看到的最大归一化命令仍为 0.9。
```

预防规则：

```text
凡是会改变控制律、限幅、滤波、坐标变换、归一化边界的逻辑，都必须在 MBD 模型或接口合同中声明。
firmware adapter 只允许做结构体映射、单位已明确的参数同步、状态初始化和调用封装。
```

## ERR-2026-06-23-007：生成代码 adapter 不能只依赖文件级静态状态

日期：2026-06-23
状态：已修复
严重程度：中

错误判断：

```text
因为 green-joint 当前是单电机单实例，所以 adapter 中直接使用文件级静态：
  gj_mbd_model
  gj_mbd_dwork
  gj_mbd_u
  gj_mbd_y
  gj_mbd_initialized

短期可运行即可。
```

正确结论：

```text
MBD 生成模块应优先提供显式 context API。
单轴全局 wrapper 可以作为兼容层，但不能成为唯一生产接口。
```

证据：

```text
PI 有积分状态，状态属于算法实例。
如果后续扩展到多轴、双电机、仿真并行调用或测试 harness 并发调用，
文件级静态状态会导致状态串扰，且调参全局量也会被不同实例互相覆盖。
```

影响：

```text
1. 多轴复用时积分状态不独立。
2. 单元测试无法方便创建多个独立实例。
3. 参数同步依赖 generated global，接口边界不够清晰。
```

修复：

```text
1. adapter 增加 GreenJointCurrentLoopMbdContext_t。
2. 增加 GreenJointCurrentLoopMbdContext_Init/Reset/Step。
3. context 内保存 model、dwork、input、output、tuning。
4. 保留 GreenJointCurrentLoopMbd_Step() 作为当前单轴兼容 wrapper。
```

残留风险：

```text
生成代码参数 CurDKp/CurDKi/... 仍是 ExportedGlobal。
context step 会在调用前把 context tuning 同步到 generated globals。
这对当前单轴安全；若未来真正并发多实例，需要把 tunable 参数改成模型参数结构或实例参数。
```

预防规则：

```text
凡是含状态的 MBD 模块，adapter 必须先设计 context。
全局 wrapper 只能作为 legacy/single-axis 兼容层。
```

## ERR-2026-06-29-008：SpeedEstimator MBD 角度 wrap 不能使用 Math Function mod

日期：2026-06-29
状态：已修复
严重程度：中

错误判断：

```text
在 speed_estimator_pll_model 中使用 Simulink Math Function 的 mod 运算做：
  wrap_0_to_2pi
  wrap_pi

认为这只是数学表达方式，不会影响嵌入式实现。
```

正确结论：

```text
生产 MBD 模型中的角度 wrap 应使用手写 loop：
  while angle >= 2*pi: angle -= 2*pi
  while angle < 0:     angle += 2*pi

不要在 STM32 生产路径里用 Math Function mod 做角度 wrap。
```

证据：

```text
Math Function mod 生成的 SpeedEstimatorPllStep.c 中出现：
  rt_modf_snf()
  fmodf()

改为 MATLAB Function block 的 wrap_0_to_2pi_loop 后，生成代码变为 while
加减 6.28318548F，不再包含 fmod/fmodf/rt_modf_snf。
```

影响：

```text
1. fmodf 在 20 kHz ISR 中成本更高，执行时间更难预测。
2. 角度 wrap 是速度估算器每拍必经路径，不能依赖通用浮点取模。
3. 与 green-joint 固件侧 wrap_0_to_2pi_loop 口径不一致。
```

修复：

```text
1. motor_speed_estimator_mbd/build_speed_estimator_pll_model.m:
   wrap_0_to_2pi / wrap_pi 子系统改成 MATLAB Function block。
2. motor_speed_estimator_mbd/speed_estimator_pll_step_fcn.m:
   参考函数改为 wrap_0_2pi_loop。
3. 重新运行 smoke test 和 codegen。
4. 同步生成代码到 green-joint/Module/MBD/green_joint_speed_estimator。
5. 更新 Debug 构建清单，删除旧 rt_nonfinite/rtGetNaN/rtGetInf 残留。
```

预防规则：

```text
MBD 生产代码中的周期角度 wrap 默认使用 loop wrap。
只有非 ISR、离线仿真或明确不进固件的 harness，才允许临时使用 mod。
生成代码检查必须 grep fmod/rt_modf_snf。
```

## ERR-2026-06-29-004：PLL 只按无噪声阶跃选择 600Hz 默认导致实机速度噪声偏大

日期：2026-06-29
状态：已修正
严重程度：中

错误判断：

```text
在 100us 速度环 V1 无噪声平均电压模型里，600Hz PLL 的阶跃超调和估算滞后优于
120/240/360/480Hz，因此把 600Hz 设为 green-joint 主线默认值。
```

正确结论：

```text
PLL 默认值不能只看无噪声阶跃响应。编码器角度噪声进入速度估算的强度主要跟
PllKi = (2*pi*bw)^2 相关。600Hz 在实机上速度噪声明显偏大，应下调默认值。

当前硬件 bring-up 默认：
  pll_bandwidth_hz = 360Hz
  damping = 1.0
  PllKp = 4523.89355
  PllKi = 5116403.0
```

证据：

```text
用户实机反馈：PLL 运行起来，噪声比较大。
理论关系：360Hz 相对 600Hz 的 Ki 比例为 (360/600)^2 = 0.36。
V1 无噪声结果仍显示 360Hz 明显优于旧 120Hz：
  360Hz overshoot ~= 17.73%, estimator |error|max ~= 0.777 rad/s
  120Hz overshoot ~= 42.19%, estimator |error|max ~= 2.215 rad/s
```

影响：

```text
1. 速度反馈噪声会进入速度 PI，造成 iq_ref 抖动。
2. 高 Ki 还会提高 zero-speed snap threshold，低速行为更敏感。
3. 只靠无噪声 V1 阶跃会偏向选择过高 PLL 带宽。
```

修复：

```text
1. motor_speed_estimator_mbd 默认 pll_bandwidth_hz 从 600 改为 360。
2. green-joint 1615/1620 variant 合同同步改为 360Hz。
3. 重新生成 SpeedEstimatorPllStep 代码并同步到固件 MBD 目录。
4. 新增 run_green_joint_speed_estimator_pll_noise_sweep.m 作为噪声敏感性验证。
```

预防规则：

```text
PLL/速度估算器默认值必须同时看：
  无噪声 V1 闭环阶跃
  编码器量化/噪声敏感性
  低速 snap 阈值
  实机速度波形

600Hz 只能作为高响应 A/B 候选，不能在未验证噪声前恢复为默认。
```

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
