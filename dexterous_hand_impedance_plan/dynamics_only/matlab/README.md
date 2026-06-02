# Single Joint Dynamics Control Lab

这是一个最小 MATLAB 控制算法实验台，用于单关节电机动力学仿真。

## Plant

```text
J*qddot + b*qdot = tau - tau_load
```

其中：

```text
q: position
qdot: velocity
tau: motor torque command after delay/saturation
tau_load: external load torque
```

## Controllers

```text
PID
DOB + PD
Impedance
```

## Tests

```text
position_step
load_step
delay_scan
```

## Run

From the repository root:

```matlab
run('dexterous_hand_impedance_plan/dynamics_only/matlab/run_study.m')
```

Results are saved to:

```text
dexterous_hand_impedance_plan/dynamics_only/results/
```
