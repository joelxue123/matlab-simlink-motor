# Stability And Bandwidth Proof Notes

## 1. 单关节闭环模型

关节动力学：

```text
J q_ddot + b q_dot = tau + tau_ext
```

阻抗/PD 控制：

```text
tau = K(q_d - q) - D q_dot
```

闭环：

```text
J q_ddot + (b + D)q_dot + Kq = Kq_d + tau_ext
```

## 2. 位置跟踪传递函数

无外力时：

```text
Q(s) / Q_d(s) = K / (J s^2 + (b + D)s + K)
```

定义：

```text
omega_n = sqrt(K / J)
zeta = (b + D) / (2 sqrt(JK))
```

结论：

```text
K 增大 -> omega_n 增大 -> 响应更快
D 增大 -> zeta 增大 -> 振荡减小
只增大 K 但 D 不变 -> zeta 下降 -> 更容易超调和振荡
```

如果希望固定阻尼比：

```text
D = 2 zeta sqrt(JK) - b
```

建议：

```text
接近阶段: zeta = 0.7 ~ 1.0
接触/保持: zeta = 1.0 ~ 1.5
```

## 3. 外力到位置偏差传递函数

有外力时：

```text
Q(s) = G_q(s) Q_d(s) + G_f(s) Tau_ext(s)
```

其中：

```text
G_q(s) = K / (J s^2 + (b + D)s + K)
G_f(s) = 1 / (J s^2 + (b + D)s + K)
```

恒定外力矩：

```text
Tau_ext(s) = Tau_0 / s
```

稳态偏差：

```text
q_ss - q_d = Tau_0 / K
```

结论：

```text
K 大: 外力偏差小，但接触力和电流更大
K 小: 外力偏差大，更柔顺，发热更低
D: 决定外力突变后的振荡和衰减
J: 决定外力突变瞬间的加速度
```

## 4. 突然外力响应

若：

```text
tau_ext(t) = Tau_0 u(t)
```

则：

```text
E(s) = Tau_0 / [s(Js^2 + (b + D)s + K)]
```

初始加速度：

```text
q_ddot(0+) = Tau_0 / J
```

最终偏移：

```text
e_ss = Tau_0 / K
```

## 5. Lyapunov 稳定性

令：

```text
e = q - q_d
```

当 `q_d` 固定时：

```text
J e_ddot + (b + D)e_dot + K e = tau_ext
```

取能量函数：

```text
V = 1/2 J e_dot^2 + 1/2 K e^2
```

求导：

```text
V_dot = -(b + D)e_dot^2 + e_dot tau_ext
```

无外力时：

```text
V_dot = -(b + D)e_dot^2 <= 0
```

因此只要：

```text
J > 0
K > 0
b + D > 0
```

闭环稳定。

有外力时：

```text
V_dot = e_dot tau_ext - (b + D)e_dot^2
```

说明系统从外力矩 `tau_ext` 到速度 `e_dot` 是被动的。

## 6. 饱和与发热证明

饱和控制：

```text
tau = sat(tau_nom, tau_max)
```

则：

```text
|tau| <= tau_max
```

电流上界：

```text
|I| <= tau_max / Kt
```

热功率：

```text
P_heat = I^2 R
```

因此：

```text
P_heat <= R(tau_max / Kt)^2
```

这说明降低 `tau_max` 能直接降低发热上界。

若保持力矩从 `tau_old` 降为 `tau_new`：

```text
P_new / P_old ~= (tau_new / tau_old)^2
```

## 7. PI 外环收敛证明

使用低维握紧变量：

```text
q_d = q_shape + S alpha
```

假设抓握力估计满足局部单调：

```text
F_hat = phi(alpha)
phi'(alpha) > 0
```

积分调节：

```text
alpha_dot = ki(F_ref - F_hat)
```

令：

```text
e_F = F_ref - F_hat
```

则：

```text
e_F_dot = -phi'(alpha) alpha_dot
        = -ki phi'(alpha)e_F
```

只要：

```text
ki > 0
phi'(alpha) > 0
```

则：

```text
e_F -> 0
```

有噪声、摩擦、饱和时，结论变为：

```text
e_F 收敛到有界邻域
```

## 8. 内外环带宽设计

内环阻抗带宽：

```text
omega_impedance ~= sqrt(K / J)
```

外环 PI 带宽应明显低于内环：

```text
omega_PI <= 0.1 ~ 0.2 omega_impedance
```

原因：

```text
内环负责快速稳定关节动力学
外环只慢速调握紧量或力矩上限
外环太快会引起力估计滞后、过补偿和接触振荡
```

## 9. 切换系统稳定性

状态机切换要满足：

```text
滞回
最小驻留时间
K/D/tau_max 平滑过渡
```

参数平滑：

```text
K_dot = (K_target - K) / T_K
D_dot = (D_target - D) / T_D
```

证明思路：

```text
每个模式下内环稳定
切换不瞬时注入过多能量
切换频率受驻留时间限制
因此整体切换系统稳定
```

## 10. 最终可写成的定理

若满足：

```text
J > 0
K_min <= K(t) <= K_max
D(t) + b >= D_min > 0
|tau| <= tau_max
PI 输出 alpha/tau_max 有界且变化率受限
F_hat 对 alpha 局部单调
状态切换有滞回和最小驻留时间
```

则闭环系统满足：

```text
关节位置和速度有界
力矩和电流有界
热功率有上界
抓握力误差收敛到有界邻域
接触过程保持被动/稳定
```
