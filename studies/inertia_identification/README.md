# Inertia Identification Study

这个目录用于继续做平均值逆变器 PMSM 模型的惯量辨识。

当前先落一个可运行的基线链路：

- 复用现有 `speedloop_kf_test` 速度环阶跃模型
- 从 `w_meas` / `w_kf` 的阶跃响应提取等效闭环带宽
- 结合当前速度 PI 参数，回推等效机械惯量 `J`

这不是最终版辨识方案，但它能先把“单独一个 study 目录 + 可复现入口 + 输出物集中落盘”这条链路跑通。

## 入口

```matlab
cd studies/inertia_identification
init_project_paths
result = run_study;
```

## 默认做什么

默认 `run_study` 会：

- 创建 `outputs/`
- 调用 `init_project_paths`
- 运行一组速度环阶跃仿真
- 分别基于 `w_meas` 和 `w_kf` 提取阶跃等效带宽
- 用速度 PI 的 `Kp` / `Ki` 公式回推等效惯量
- 输出 summary 表、波形 csv、mat 文件和一张汇总图

## 当前辨识思路

速度环设计里已经用了下面两条关系：

- `Kp = 2 * zeta * wbw * J / Kt`
- `Ki = wbw^2 * J / Kt`

因此测到闭环等效带宽 `wbw` 后，可以反推：

- `J_from_Kp = Kp * Kt / (2 * zeta * wbw)`
- `J_from_Ki = Ki * Kt / wbw^2`

当前 study 默认把二者取平均，作为一个“等效惯量基线估计值”。

## 常用运行方式

### 1. 默认运行

```matlab
cd studies/inertia_identification
init_project_paths
result = run_study;
```

### 2. 扫一组惯量缩放系数

```matlab
cfg = default_config;
cfg.inertia_scale_list = [0.5 1.0 2.0 4.0];
cfg.plot_results = true;
result = run_study(cfg);
```

### 3. 只用 `w_meas` 做估计

```matlab
cfg = default_config;
cfg.speed_source = 'wm';
cfg.plot_results = false;
result = run_study(cfg);
```

### 4. 修改机械惯量后同步重算速度 PI

```matlab
cfg = default_config;
cfg.inertia_scale_list = [1.0 2.0 4.0];
cfg.redesign_speed_pi = true;
result = run_study(cfg);
```

## 输出位置

默认输出写到：

- `outputs/inertia_identification_result.mat`
- `outputs/inertia_identification_summary.csv`
- `outputs/<case_name>_waveforms.csv`
- `outputs/<case_name>_result.mat`
- `outputs/inertia_identification_summary.png`

## 当前限制

- 这个入口依赖 `build_speedloop_kf_test` 里固定的速度阶跃时刻，目前默认按 `0.02 s` 上升、`0.30 s` 反向。
- 当前识别的是“闭环等效惯量”，更适合做基线比较，不是最终的物理参数辨识结论。
- 还没有接入直接的 `iq -> torque -> acceleration` 开环拟合链路；后续如果要上更严格的辨识方法，建议继续在这个目录扩展，而不是再把脚本散回项目根目录。

## 适合继续往里放什么

- 惯量扫参验证
- 基于 `w_meas` / `w_kf` 的辨识算法对比
- 带负载扰动的等效惯量估计
- 后续的扭矩-加速度拟合版惯量辨识入口

## 不适合放什么

- 与齿槽建表直接相关的脚本
- 纯位置环带宽测试
- 与振动补偿主链路强绑定的验证入口