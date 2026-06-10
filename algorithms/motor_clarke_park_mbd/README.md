# Motor Clarke/Park MBD Module

This directory is a reusable MBD module package for `average-inverter`.

It intentionally follows the modern embedded-code workflow:

```text
.sldd + Simulink.NumericType / AliasType / Bus
+ reusable/reentrant Simulink model
+ functional test harness
+ optional integration harness
```

Do not replace this module with loose `algorithms/*.m` MATLAB Function scripts
when the goal is embedded C delivery. MATLAB Function scripts are useful for
quick simulation, but this package is the project-level reusable MBD contract.

## Interface

Input Bus:

```text
motor_t {
  T_MotorCurrent ia;
  T_MotorCurrent ib;
  T_MotorCurrent ic;
  T_MotorAngle   theta_e;
}
```

Output Bus:

```text
motor_dq_t {
  T_MotorCurrent i_alpha;
  T_MotorCurrent i_beta;
  T_MotorCurrent id;
  T_MotorCurrent iq;
}
```

Current type:

```text
T_MotorCurrent = sfix16_En12
```

This is defined in `motor_interface.sldd` through `Simulink.NumericType`, with
exported typedefs in `motor_types.h`.

## Files

- `build_motor_clarke_park_model.m` creates/updates `motor_interface.sldd` and
  builds the reusable Simulink model.
- `run_motor_clarke_park_function_test.m` builds and runs a visual functional
  test harness.
- `build_motor_open_loop_integration_model.m` shows how an upstream module can
  connect through the shared `motor_t` Bus contract.
- `generate_motor_clarke_park_code.m` runs Embedded Coder code generation.

Generated files such as `.slx`, `.sldd`, `.slxc`, `slprj/`, and `*_ert_rtw/`
are outputs of these scripts.

## Run

From repository root:

```bash
matlab -batch "run('average-inverter/algorithms/motor_clarke_park_mbd/run_motor_clarke_park_function_test.m')"
```

To generate the integration harness:

```bash
matlab -batch "run('average-inverter/algorithms/motor_clarke_park_mbd/build_motor_open_loop_integration_model.m')"
```

To generate embedded C:

```bash
matlab -batch "run('average-inverter/algorithms/motor_clarke_park_mbd/generate_motor_clarke_park_code.m')"
```

## Reuse Rule

The reusable algorithm boundary is the Bus contract, not individual ad hoc
scalar wires:

```text
upstream module -> motor_t -> MotorClarkeParkStep -> motor_dq_t
```

If an upstream module produces scalar `ia`, `ib`, `ic`, and `theta_e`, use an
adapter subsystem or Bus Creator to assemble `motor_t`. If it produces another
structure, map that structure to `motor_t` in an adapter. Keep
`MotorClarkeParkStep` stable.
