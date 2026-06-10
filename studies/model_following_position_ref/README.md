# Model-Following Position Reference

这个实验复现更接近工业伺服的模型跟踪位置指令生成：

```text
位置阶跃 x_cmd
  -> 离散二阶参考模型
  -> 加速度/速度限幅
  -> pos_ref、vel_ref、acc_ref
  -> 位置环 + 速度前馈
```

参考模型不使用连续微分块，而是在 MATLAB Function block 里维护状态：

```text
pos_dot = vel
vel_dot = wn^2 * (pos_cmd - pos) - 2*zeta*wn*vel
acc_ref = saturate(vel_dot)
vel_ref = saturate(vel_ref + acc_ref * Ts)
pos_ref = pos_ref + vel_ref * Ts
```

简化轴代理使用：

```text
speed_cmd = Kp_pos * (pos_cmd - theta) + Kff * vel_ref
speed loop proxy: w_dot = (speed_cmd - w) / tau
theta_dot = w
```

模型里同时对比三种情况：

- raw position step
- model-followed position only
- model-followed position + velocity feedforward

## 运行

```matlab
cd /home/user/study/matlab-practice/average-inverter/studies/model_following_position_ref
result = run_study;
```

打开模型和 Scope：

```matlab
result = run_study(struct('open_model', true));
```

调整参考模型和限幅：

```matlab
result = run_study(struct('ref_bandwidth_hz', 8));
result = run_study(struct('max_acc_ref_rad_s2', 5000, 'max_vel_ref_rad_s', 120));
result = run_study(struct('axis_vel_ff_gain', 0.8));
```

## 输出

运行后会生成：

- `model_following_position_ref.slx`
- `outputs/model_following_position_ref_result.mat`
- `outputs/model_following_position_ref_signals.csv`
- `outputs/model_following_position_ref.png`

## 观察点

- `acc_unsat` 和 `acc_ref`：能看到加速度限幅前后的差异
- `vel_ref`：不是微分得到的，而是参考模型状态
- `axis_mf_noff` vs `axis_mf_vff`：能看到速度前馈对跟随误差的影响
- `speed_cmd_mf_vff`：位置 P 输出叠加速度前馈后的速度指令
- `max_acc_ref_rad_s2` 越小，指令越柔和，但定位时间越长
- `axis_vel_ff_gain` 越接近合适值，模型跟踪位置的误差越小
