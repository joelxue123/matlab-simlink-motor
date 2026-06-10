# Simulink Local-Q PI Module

This module keeps code generation inside Simulink.

The design goal is:

- The module boundary receives and returns already-quantized fixed-point signals.
- The PI gains use a local gain type, `T_pi_gain`.
- The integrator and internal `P + I` sum use a local accumulator type, `T_pi_integrator`.
- The generated `.c` remains owned by Simulink Coder, so module development stays in the Simulink diagram.

Build the model:

```matlab
run('build_fixed_point_pi_local_q_model.m')
```

Generate C:

```matlab
run('generate_fixed_point_pi_local_q_code.m')
```

Default type split:

```text
input/output boundary: sfix16_En14
kp, ki_dt:             sfix32_En20
integrator/internal:   sfix32_En24
```

`Ki_dt_local` already includes sample time:

```text
Ki_dt_local = Ki_local * Ts
```

The generated model removes the root-level `ref_to_q14` and `feedback_to_q14`
conversion blocks from the earlier test harness. That makes the reusable PI
subsystem depend only on its interface type and local PI numeric policy.

The generated reusable subsystem function has a module-style interface:

```c
void fixed_point_FixedPointPI_LocalQ(
    const FixedPointPI_LocalQ_Input *rtu_pi_in,
    FixedPointPI_LocalQ_Output *rty_pi_out,
    DW_FixedPointPI_LocalQ_fixed__T *localDW);
```

The generated source is:

```text
fixed_point_pi_local_q_ert_rtw/fixed_point_pi_local_q.c
```
