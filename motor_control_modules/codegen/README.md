# Reusable Code Generation

This folder is reserved for controller-only code generation scripts.

Current generated-code candidates:

```text
SpeedPiStep
CurrentPiStep
DqToAbcDutyStep
MotorClarkeParkStep
```

Available milestone codegen scripts today:

```text
motor_speed_pi_mbd/generate_speed_pi_code.m
motor_current_pi_mbd/generate_current_pi_code.m
motor_clarke_park_struct/generate_motor_clarke_park_code.m
motor_current_loop_mbd/generate_current_loop_code.m
```

Next cleanup target:

```text
Create a controller-only codegen script for the full reusable module set,
without Average-Value Inverter or PMSM plant code.
```

Code generation rules:

```text
1. Do not hand-edit generated C.
2. Do not include platform registers in generated controller code.
3. Verify generated headers and type headers after every codegen run.
4. Keep generated functions reusable/reentrant where possible.
```
