# Efficiency Iq Bus Current Study

这个实验基于文件夹里的速度环模型 `speedloop_kf_test`，扫描不同机械转速和目标 `Iq`，统计稳态 `Iq`、母线电流、输入功率、机械输出功率、损耗拆分和效率估算。

面向客户发布时优先看 [CUSTOMER_GUIDE.md](CUSTOMER_GUIDE.md)，并从 `run_customer_example.m` 填参数运行。

## 入口

```matlab
cd studies/efficiency_iq_bus_current
result = run_study;
```

客户模板入口：

```matlab
result = run_customer_example;
```

## 默认扫描

默认参数在 `default_config.m`：

- `speed_rpm_list = [500 1000 2000 2500]`
- `iq_target_a_list = [0.5 1:1:10 12:2:20]`
- 每个工况仿真 `0.35 s`
- 取 `0.25 s` 到仿真结束的稳态窗口求平均

每个工况会设置速度参考，并通过负载转矩让速度环在稳态输出目标附近的 `Iq`。最终表格以仿真的 `mean_iq_a` 为准，而不是只相信目标值。

默认电机参数来自 `motor_control_params.m`。客户发布场景建议不要直接改这个文件，而是在 `cfg.motor` 和 `cfg.inverter` 中覆盖参数：

```matlab
cfg.motor.pole_pairs = 10;
cfg.motor.kv_vrms_per_krpm = 17.03;
cfg.motor.line_to_line_resistance = 0.4267;
cfg.motor.line_to_line_inductance = 0.53e-3;
cfg.motor.J = 2.5e-4;
cfg.motor.B = 1.0e-4;
cfg.inverter.Vdc = 68;
cfg.inverter.drive_efficiency = 0.95;
```

## 常用参数

```matlab
cfg = struct();
cfg.output_tag = 'customer_motor_68V';
cfg.inverter.Vdc = 68;
cfg.inverter.drive_efficiency = 0.95;
cfg.speed_rpm_list = [500 1000 2000 2500];
cfg.iq_target_a_list = [0.5 1:1:10 12:2:20];
cfg.stop_time_s = 0.4;
cfg.eval_start_s = 0.28;
result = run_study(cfg);
```

快速单点验证：

```matlab
cfg = struct( ...
    'speed_rpm_list', 200, ...
    'iq_target_a_list', 1, ...
    'stop_time_s', 0.12, ...
    'eval_start_s', 0.08, ...
    'plot_results', false);
result = run_study(cfg);
```

包含负 `Iq`：

```matlab
cfg = struct('include_negative_iq', true);
result = run_study(cfg);
```

## 输出

默认输出到当前实验目录的 `outputs/`：

- `efficiency_iq_bus_current_summary.csv`
- `efficiency_iq_bus_current_result.mat`
- `efficiency_iq_bus_current.png`
- `speedloop_kf_test.slx`

摘要表里最常用的列：

- `speed_target_rpm`
- `iq_target_a`
- `mean_speed_rpm`
- `mean_iq_a`
- `mean_p_elec_w`
- `mean_p_dc_w`
- `mean_ibus_a`
- `mean_p_mech_shaft_w`
- `mean_p_cu_w`
- `mean_p_fric_w`
- `mean_p_drive_loss_w`
- `mean_modulation_ratio`
- `max_modulation_ratio`
- `motor_efficiency_pct`
- `efficiency_pct`
- `is_valid`

## 计算口径

三相电功率：

```matlab
p_motor = va*ia + vb*ib + vc*ic
```

母线侧功率和母线电流估算：

```matlab
p_dc = p_motor / inverter.drive_efficiency
ibus = p_dc / inverter.Vdc
```

机械输出功率：

```matlab
p_mech = Tload * wm
```

效率：

```matlab
eta_motor = p_mech / p_motor
eta_system = p_mech / p_dc
```

摘要表里的 `efficiency_pct` 是系统效率 `eta_system`，已经包含配置的驱动效率；`motor_efficiency_pct` 是只看电机端的效率。

铜损和粘性摩擦损耗按下面方式估算：

```matlab
p_cu = 3 * Rs * Irms_phase^2
p_fric = B * wm^2
```

因为这里复用的是平均值逆变器模型，`mean_ibus_a` 是由电机端三相功率加驱动效率折算到 DC 母线的电流估算，不包含真实开关器件非线性、死区损耗和母线纹波。高转速大电流点可能电压受限，除了看 `is_valid`，还要看 `max_modulation_ratio` 是否超过 `1`。
