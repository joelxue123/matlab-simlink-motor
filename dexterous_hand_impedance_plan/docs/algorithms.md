# Algorithms To Try

## Stage 1: 可落地算法

### 1. 原始 PD 基线

控制律：

```text
tau = Kp_close(q_close - q) - Kd_close q_dot
```

用途：

```text
作为对照组，验证原始策略为何发热。
```

预期：

```text
接触后位置误差长期存在，tau 和 I 持续偏大。
```

### 2. 误差限幅 PD

控制律：

```text
e = sat(q_d - q, e_max)
tau = Kp e - Kd q_dot
```

证明点：

```text
|tau| <= Kp e_max + Kd |q_dot|
```

优点：

```text
实现最简单，能快速限制硬顶力矩。
```

### 3. 力矩/电流饱和

控制律：

```text
tau = sat(tau_pd, tau_max)
```

电流上界：

```text
|I| <= tau_max / Kt
```

热功率上界：

```text
P_heat <= R(tau_max / Kt)^2
```

### 4. 接触后低刚度阻抗

接触前：

```text
K = K_close
D = D_close
q_d = q_close
```

接触后：

```text
K = K_hold
D = D_hold
q_d = q_contact + alpha
tau_max = tau_hold_max
```

其中：

```text
K_hold < K_close
tau_hold_max < tau_close_max
```

### 5. 状态机增益调度

状态：

```text
APPROACH
CONTACT
SQUEEZE
HOLD
SLIP_RECOVERY
THERMAL_LIMIT
RELEASE
```

每个状态调度：

```text
K(s), D(s), tau_max(s), q_d(s), F_ref(s)
```

切换要有：

```text
滞回
最小驻留时间
参数平滑过渡
```

### 6. PI 调整体握紧变量 alpha

多关节不要独立调每个 `q_d_i`，使用低维协同：

```text
q_d = q_shape + S alpha
```

PI 外环：

```text
alpha = sat(kp_f e_F + ki_f integral(e_F), alpha_min, alpha_max)
e_F = F_ref - F_hat
```

或者只用积分慢调：

```text
alpha_dot = sat(ki_f e_F, alpha_dot_max)
```

### 7. PI 调力矩上限 tau_max

控制律：

```text
tau_max_dot = sat(ki_f(F_ref - F_hat), tau_dot_max)
tau_max = sat(tau_max, tau_min, tau_limit)
```

优点：

```text
直接控制电流和发热上界。
```

缺点：

```text
如果 tau_max 太低，可能抓不稳。
```

### 8. 滑移触发补偿

逻辑：

```text
if slip_detected:
    F_ref = F_ref + Delta_F_fast
    tau_max = tau_max + Delta_tau_fast
else if stable_for_T:
    F_ref = F_ref - Delta_F_slow
    tau_max = tau_max - Delta_tau_slow
```

目标：

```text
抓稳时慢慢降电流，滑移时快速补力。
```

## Stage 2: 研究算法

### 1. 扰动观测器 DOB

估计外部接触力矩：

```text
tau_ext_hat = Q(s)[tau_motor - J q_ddot_hat - b q_dot]
```

用途：

```text
接触检测
抓握力估计
滑移风险判断
自适应阻抗调节
```

### 2. 扩张状态观测器 ESO

把摩擦、接触力、建模误差合并为总扰动：

```text
d = tau_ext - tau_friction + model_error
```

ESO 估计：

```text
d_hat
```

用途：

```text
无力传感器情况下估计接触扰动。
```

### 3. 自适应可变阻抗

控制律：

```text
tau = sat(K(t)(q_d - q) - D(t)q_dot, tau_max)
```

刚度调节：

```text
K_dot = gamma * adaptation_signal
K = sat(K, K_min, K_max)
```

阻尼联动：

```text
D = 2 zeta sqrt(J K) - b
```

### 4. 物体刚度在线估计

估计：

```text
K_obj_hat = Delta F_hat / Delta x
```

调节：

```text
soft object -> lower K and tau_max
hard contact -> higher D and lower closing speed
slip risk -> increase F_ref or K temporarily
```

### 5. Passivity Observer/Controller

监控能量：

```text
E(t) = integral(tau_ext * q_dot) dt
```

当系统注入过多能量时增加阻尼：

```text
D = D + D_passivity
```

适合接触稳定证明。

### 6. CBF-QP 安全层

名义控制：

```text
tau_nom = impedance + PI
```

QP 修正：

```text
min ||tau - tau_nom||^2

s.t.
|tau| <= tau_limit
|I| <= I_limit
T_motor <= T_limit
q_min <= q <= q_max
slip_risk <= slip_limit
```

优势：

```text
可以把安全约束写成数学证明。
```

### 7. 低维 MPC

控制变量：

```text
alpha
tau_max
F_ref
```

代价函数：

```text
min sum(lambda_I I^2 + lambda_slip slip_risk + lambda_e e_F^2)
```

约束：

```text
I <= I_limit
tau <= tau_limit
F_grip >= F_required
T_motor <= T_limit
```

建议：

```text
不要一开始做全关节 MPC，先做低维抓握力 MPC。
```

### 8. Safe RL 高层调参

RL 不直接输出力矩，只输出高层参数：

```text
K_hold
D_hold
tau_max
F_ref
alpha_ref
slip_margin
```

底层仍然保留：

```text
阻抗控制
饱和
CBF/QP 安全层
```

## 推荐尝试顺序

```text
1. 原始 PD 基线
2. 误差限幅 + 力矩饱和
3. 接触后低刚度阻抗
4. PI 调 alpha
5. PI 调 tau_max
6. 滑移触发补偿
7. DOB/ESO 接触力估计
8. 自适应可变阻抗
9. CBF-QP 安全层
10. 低维 MPC
```
