# Simulation Plan

## 目标

验证灵巧手抓取包裹时，阻抗控制和限流策略是否能降低持续电流和发热，同时保持抓取稳定。

仿真分为四级：

```text
Level 1: 单关节阻抗模型
Level 2: 单手指多关节模型
Level 3: 多指低维协同抓取模型
Level 4: 含温度、滑移、观测器和优化控制的完整模型
```

## Level 1: 单关节模型

动力学：

```text
J q_ddot + b q_dot = tau + tau_ext
```

接触模型可以先用线性弹簧：

```text
tau_ext = -K_obj max(q - q_obj, 0) - D_obj q_dot_contact
```

原始 PD：

```text
tau = Kp_close(q_close - q) - Kd_close q_dot
```

改进阻抗：

```text
tau = sat(K_hold(q_d - q) - D_hold q_dot, tau_max)
```

接触后：

```text
q_d = q_contact + alpha
```

## Level 1 实验组

```text
A: 原始高刚度 PD
B: 误差限幅 PD
C: 接触后低刚度阻抗 + 力矩饱和
D: 低刚度阻抗 + PI 调 alpha
E: 低刚度阻抗 + PI 调 tau_max
```

## Level 2: 单手指多关节

动力学简化为：

```text
M(q) q_ddot + C(q, q_dot) q_dot + g(q) = tau + J_c(q)^T F_c
```

工程仿真可以先用对角惯量：

```text
J_i q_ddot_i + b_i q_dot_i = tau_i + tau_ext_i
```

不要每个关节独立调 `q_d_i`，而是引入协同变量：

```text
q_d = q_shape + S alpha
```

其中：

```text
q_shape: 基础抓取姿态
S: 闭合协同方向
alpha: 整体握紧量
```

## Level 3: 多指协同

每根手指一个握紧变量：

```text
q_d = q_shape + S alpha
alpha = [alpha_1, alpha_2, alpha_3, ...]^T
```

外环可以控制每根手指的目标握持强度：

```text
alpha_dot_i = ki_i(F_ref_i - F_hat_i)
```

也可以控制总握持强度：

```text
alpha_dot = ki(F_ref_total - F_hat_total)
```

## Level 4: 完整研究模型

加入：

```text
1. 扰动观测器 DOB/ESO
2. 物体刚度在线估计
3. 可变刚度阻抗
4. 电流/温度约束
5. CBF-QP 安全层
6. 低维 MPC 优化握持力
```

## MATLAB 仿真建议

先写函数：

```text
simulate_single_joint_case(config)
```

输入：

```text
J, b
Kp, Kd
K_obj, D_obj
q_close, q_obj
tau_max
controller_type
simulation_time
```

输出：

```text
t, q, q_dot, q_d, tau, I, F_hat, alpha, tau_max
```

## Simulink 仿真建议

模块划分：

```text
Plant: 关节动力学
Contact: 接触模型
Controller: 阻抗/PD 控制器
StateMachine: APPROACH/CONTACT/HOLD/SLIP/THERMAL
PIOuterLoop: alpha 或 tau_max 调节
Saturation: 误差、力矩、电流限幅
ThermalModel: 电机温升模型
Logger: 数据记录
```

## Python 分析建议

Python 用于批量读结果和画图：

```text
1. 读取 .mat 或 .csv
2. 计算 I_rms、I_peak、tau_peak、温升
3. 画 q/q_d、tau、I、接触力、温度
4. 生成对比表
```

## 实验矩阵

建议扫参数：

```text
K_hold: 低/中/高
D_hold: 欠阻尼/临界阻尼/过阻尼
tau_max: 小/中/大
K_obj: 软/中/硬
F_ref: 小/中/大
PI bandwidth: 慢/中/快
```

核心观察：

```text
K_hold 对响应速度和外力偏移的影响
D_hold 对振荡和冲击的影响
tau_max 对电流和发热上界的影响
PI bandwidth 对稳定性和力调节速度的影响
```
