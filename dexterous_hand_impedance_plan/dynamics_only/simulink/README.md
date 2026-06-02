# Simulink Single Joint Dynamics Model

这个目录用于生成和运行单关节动力学 Simulink 模型。

## Build Model

From the repository root:

```matlab
run('dexterous_hand_impedance_plan/dynamics_only/simulink/build_single_joint_dynamics_model.m')
```

This creates:

```text
single_joint_dynamics_control.slx
```

## Model Structure

```text
q_ref Step
tau_load Step
q/qdot/qddot feedback
    ↓
Controller MATLAB Function
    ↓
Torque Saturation
    ↓
Command Delay
    ↓
Discrete Plant: J*qddot + b*qdot = tau - tau_load
    ↓
Outports and Scopes
```

Controller modes:

```text
1: PID
2: DOB + PD
3: Impedance
```

## Generate C Code

After building the model:

```matlab
run('dexterous_hand_impedance_plan/dynamics_only/simulink/generate_c_code.m')
```

The script configures the model for fixed-step discrete simulation and calls:

```matlab
slbuild('single_joint_dynamics_control')
```

If Simulink Coder is installed, C code will be generated in the MATLAB build output folder.

## Controller Block Code Generation

The `Controller` block should be configured as a reusable function for embedded deployment:

```text
Function packaging: Reusable function
Function name options: User specified
Function name: usr_pid
File name options: User specified
File name: usr_pid
```

This is now also recorded in `build_single_joint_dynamics_model.m`, so rebuilding the model preserves this setting.

To generate code for only this block:

```matlab
run('dexterous_hand_impedance_plan/dynamics_only/simulink/generate_controller_block_c_code.m')
```

## Struct/Bus I/O Controller

For maintainability, a controller-only model with struct-style I/O is also available:

```matlab
run('dexterous_hand_impedance_plan/dynamics_only/simulink/build_controller_struct_model.m')
run('dexterous_hand_impedance_plan/dynamics_only/simulink/generate_controller_struct_c_code.m')
```

The interface uses:

```text
ControllerInputBus:
  q_ref
  q
  qdot
  qddot
  tau_prev
  mode

ControllerOutputBus:
  tau_cmd
  tau_load_hat
```

The subsystem-build output is expected at:

```text
ControllerStruct_grt_rtw/ControllerStruct.c
ControllerStruct_grt_rtw/ControllerStruct.h
```

## Generated C SIL Compare

生成 `.c/.h` 之后，还需要验证生成 C 和 Simulink 模型行为一致。

当前目录提供一个 host-side generated-C-in-the-loop 检查：

```matlab
run('dexterous_hand_impedance_plan/dynamics_only/simulink/run_generated_c_sil_compare.m')
```

这个脚本会：

```text
1. 编译 Controller_ert_rtw/Controller.c 为 controller_ert_mex
2. 生成一组固定点 raw 输入序列
3. 把 raw 输入转换成 Simulink 物理量输入
4. 跑 single_joint_controller_only.slx，得到 MIL 输出
5. 跑 generated C MEX，得到 C 输出
6. 把两边输出转换到 raw LSB 后逐点比较
```

通过标准：

```text
tau_cmd 差异 <= 1 raw LSB
tau_load_hat 差异 <= 1 raw LSB
```

结果输出：

```text
dexterous_hand_impedance_plan/dynamics_only/results/sil/generated_c_sil_compare.mat
dexterous_hand_impedance_plan/dynamics_only/results/sil/generated_c_sil_compare.csv
dexterous_hand_impedance_plan/dynamics_only/results/sil/generated_c_sil_compare.png
```

注意：这一步不是替代官方 Simulink SIL/PIL Manager，而是补充验证最终要移植的 generated C 接口确实可被外部程序调用，并且和模型输出一致。
