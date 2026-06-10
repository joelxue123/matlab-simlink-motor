# 电机效率/Iq-母线电流仿真操作指南

本文档面向交付使用：客户只需要输入电机、驱动和扫描参数，即可生成不同转速、不同 `Iq` 下的母线电流、效率和调制比结果。

## 1. 运行入口

在 MATLAB 中进入实验目录：

```matlab
cd <交付目录>/studies/efficiency_iq_bus_current
result = run_customer_example;
```

请将 `<交付目录>` 替换为客户本机上的项目路径。

运行结束后会在 `outputs/` 下生成：

- `efficiency_iq_bus_current_<output_tag>_summary.csv`
- `efficiency_iq_bus_current_<output_tag>_result.mat`
- `efficiency_iq_bus_current_<output_tag>.png`

其中 `csv` 适合给客户查看或导入 Excel，`mat` 保存了完整时域数据，`png` 是快速趋势图。

## 2. 客户需要填写的参数

打开 `run_customer_example.m`，只改输入区即可。

### 驱动参数

```matlab
cfg.inverter.Vdc = 68;                 % 母线电压 (V)
cfg.inverter.current_limit = 20;        % 控制器电流限幅 (A)
cfg.inverter.drive_efficiency = 0.95;   % 驱动效率，0.95 表示 95%
```

### 电机参数

```matlab
cfg.motor.pole_pairs = 10;
cfg.motor.kv_vrms_per_krpm = 17.03;            % Kv/反电动势有效值，线线 RMS，单位 Vrms/krpm
cfg.motor.line_to_line_resistance = 0.4267;    % 线电阻/相间电阻，单位 ohm
cfg.motor.line_to_line_inductance = 0.53e-3;   % 线电感/相间电感，单位 H
cfg.motor.J = 2.5e-4;                           % 转动惯量，单位 kg*m^2
cfg.motor.B = 1.0e-4;                           % 粘性摩擦系数，单位 N*m/(rad/s)
cfg.motor.max_speed_rpm = 4900;
```

当前工具按三相星形等效处理：

- `Rs = line_to_line_resistance / 2`
- `Ld = Lq = line_to_line_inductance / 2`
- `kv_vrms_per_krpm` 按“线线 RMS、机械 krpm”换算永磁磁链

如果客户只有相电阻/相电感，需要先换算成相间测量值再填入。

### 扫描参数

```matlab
cfg.speed_rpm_list = [500 1000 2000 2500];
cfg.iq_target_a_list = [0.5 1:1:10 12:2:20];
```

例如只跑 `500 rpm`：

```matlab
cfg.speed_rpm_list = 500;
```

例如最高电流只跑到 `20 A`：

```matlab
cfg.iq_target_a_list = [0.5 1:1:10 12:2:20];
```

## 3. 输出结果怎么看

最常用的是 `summary.csv`。关键列如下：

- `speed_target_rpm`：目标机械转速
- `iq_target_a`：目标 `Iq`
- `mean_speed_rpm`：稳态平均机械转速
- `mean_iq_a`：稳态实际 `Iq`
- `mean_ibus_a`：估算 DC 母线电流
- `mean_p_elec_w`：电机端三相电功率
- `mean_p_dc_w`：按驱动效率折算后的母线侧功率
- `mean_p_mech_shaft_w`：负载侧机械输出功率
- `mean_p_cu_w`：铜损估算
- `mean_p_fric_w`：粘性摩擦损耗估算
- `mean_modulation_ratio`：稳态平均调制比
- `max_modulation_ratio`：稳态窗口内最大调制比
- `efficiency_pct`：系统效率，包含驱动效率
- `motor_efficiency_pct`：只看电机端的效率
- `is_valid`：速度和 `Iq` 跟踪是否在容差内

建议客户先看三项：

1. `is_valid` 是否为 `true`
2. `max_modulation_ratio` 是否小于等于 `1`
3. `mean_iq_a` 是否接近 `iq_target_a`

如果 `max_modulation_ratio > 1`，说明该工况所需电压矢量超过当前线性调制能力，真实驱动上可能需要弱磁、过调制或更高母线电压。

## 4. 计算口径

三相电功率：

```matlab
p_elec = va*ia + vb*ib + vc*ic
```

驱动效率：

```matlab
p_dc = p_elec / cfg.inverter.drive_efficiency
ibus = p_dc / cfg.inverter.Vdc
```

机械输出功率：

```matlab
p_mech = Tload * wm
```

效率：

```matlab
motor_efficiency = p_mech / p_elec
system_efficiency = p_mech / p_dc
```

损耗估算：

```matlab
p_cu = 3 * Rs * Irms_phase^2
p_fric = B * wm^2
```

注意：当前模型主要用于平均逆变器和速度环下的趋势评估，不包含真实开关损耗、死区、器件温升、母线纹波和完整铁损模型。

## 5. 负载和 Iq 的关系

这个实验的目标是扫描 `Iq` 与母线电流/效率的关系。每个工况会设置目标转速，并自动设置负载转矩，让速度环在稳态输出目标 `Iq`：

```matlab
Tload = Kt * Iq_target - B * wm_ref
```

因此这里的 `Iq = 0.5 A` 不是严格空载工况。如果要评估真实空载电流和空载效率，需要单独设置 `Tload = 0` 的仿真模式。

## 6. 常见问题

`Iq` 和母线电流很接近：

在固定转速下，机械功率近似随 `Iq` 增加，母线电流近似为：

```matlab
Ibus ~= Kt * Iq * omega / (drive_efficiency * Vdc)
```

高速、低母线电压时，`Ibus/Iq` 接近 1 是可能的，需要同时检查调制比。

效率偏低：

低速大电流时机械输出功率小，铜损占比高，效率会下降。高速空载或轻载时摩擦、铁损和驱动损耗占比也会让效率下降；当前输出中已列出铜损和粘性摩擦损耗，铁损需要额外模型支持。

`is_valid = false`：

说明速度或 `Iq` 没有在稳态窗口内跟踪到目标。可以增加仿真时间，或降低目标电流/转速后重跑。

调制比超过 1：

说明电压裕量不足。可以降低转速、降低 `Iq`、提高母线电压，或引入弱磁/过调制策略后再评估。
