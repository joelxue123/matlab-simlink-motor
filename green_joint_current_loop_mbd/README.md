# Green Joint Current PI MBD

This module is the first production-oriented MBD extraction point for the
`green-joint` current loop.

The scope is intentionally narrow:

```text
id_ref / iq_ref / id_fbk / iq_fbk / vbus
  -> GreenJointCurrentLoopStep
  -> vd_cmd / vq_cmd / vd_norm / vq_norm / vd_mod / vq_mod
```

This follows the safer current-loop split used by controllers such as ODrive:
the current controller generates a physical `Vdq` command, then downstream
modulation code converts the normalized voltage vector into phase voltages,
SVPWM duty, sector, and timer writes.

## Why Narrow Scope

Replacing Clarke/Park, filtering, PI, inverse Park, SVPWM, sector logic, and PWM
register writes in one step is too risky. This module replaces only the current
PI and Vd-priority voltage allocation. The existing firmware can keep its
proven current feedback path and modulation path while we validate the MBD PI.

## Physical Units

The interface uses physical units first:

```text
current: A
voltage: V
sample time: s
normalized voltage vector: unit circle [-1, 1]
```

PI gains are physical:

```text
Kp = L * wc      [V/A]
Ki = R * wc      [V/(A*s)]
```

## MBD Boundary

Input:

```text
green_joint_current_loop_input_t {
  id_ref
  iq_ref
  id_fbk
  iq_fbk
  vbus
}
```

Output:

```text
green_joint_current_loop_output_t {
  vd_cmd
  vq_cmd
  voltage_mag
  vd_norm
  vq_norm
  voltage_mag_norm
  vd_mod
  vq_mod
  voltage_mag_mod
}
```

`vd_cmd/vq_cmd` are physical volts after Vd-priority voltage allocation.

`vd_norm/vq_norm` are normalized by the usable current-loop voltage limit:

```text
voltage_limit = max(vbus * VoltageLimitRatio * VoltageModulationRatio, VoltageEpsilon)
```

`VoltageLimitRatio` is the SVPWM linear base limit, normally about `1/sqrt(3)`.
`VoltageModulationRatio` is the usable headroom, currently `0.9`.

So with a 12 V bus:

```text
voltage_limit = 12 * 0.577 * 0.9 = 6.2316 V
```

This limit participates in PI saturation and anti-windup. That matches the
ODrive-style meaning: the current controller itself cannot request more than
the usable modulation voltage.

`vd_mod/vq_mod` are the final normalized voltage commands relative to the
SVPWM linear base limit for the existing firmware inverse Park/SVGEN path:

```text
vd_mod = vd_cmd / (vbus * VoltageLimitRatio)
vq_mod = vq_cmd / (vbus * VoltageLimitRatio)
voltage_mag_mod = voltage_mag / (vbus * VoltageLimitRatio)
```

Because `vd_cmd/vq_cmd` were already limited by `VoltageModulationRatio`, the
maximum magnitude of `vd_mod/vq_mod` is naturally `0.9`.

The normalized vector remains available for diagnostics and future modulation
modules:

```text
unit-circle Vdq -> inverse Park / modulation / SVPWM
```

## Not In This Module

- ADC sampling
- Clarke/Park
- adaptive current filtering
- inverse Park
- SVPWM
- sector output
- PWM/TIM register writes
- fault/state machine/protocol

## Implementation

The generated model uses Simulink primitive blocks, not a MATLAB Function block
and not a black-box PID Controller block.

Implemented algorithm:

```text
id_err = id_ref - id_fbk
iq_err = iq_ref - iq_fbk

vd_pre = CurDKp * id_err + d_integrator
vq_pre = CurQKp * iq_err + q_integrator

vd_cmd = clamp(vd_pre, -voltage_limit, +voltage_limit)

vq_limit = sqrt(max(voltage_limit^2 - vd_cmd^2, 0))
vq_cmd = clamp(vq_pre, -vq_limit, +vq_limit)

voltage_mag = sqrt(vd_cmd^2 + vq_cmd^2)

d_integrator += (CurDKi * id_err + PiCorrectionGain * (vd_cmd - vd_pre)) * Ts
q_integrator += (CurQKi * iq_err + PiCorrectionGain * (vq_cmd - vq_pre)) * Ts

voltage_limit = max(vbus * VoltageLimitRatio * VoltageModulationRatio, VoltageEpsilon)
modulation_base = max(vbus * VoltageLimitRatio, VoltageEpsilon)

vd_norm = vd_cmd / voltage_limit
vq_norm = vq_cmd / voltage_limit
voltage_mag_norm = voltage_mag / voltage_limit

vd_mod = vd_cmd / modulation_base
vq_mod = vq_cmd / modulation_base
voltage_mag_mod = voltage_mag / modulation_base
```

## Generated State

Because PI has integrator states, generated code includes:

```c
GreenJointCurrentLoopStep_Init(...)
DW_GreenJointCurrentLoopStep_T
```

The firmware adapter must allocate and initialize this state once, then pass it
to every 50 us current-loop call.

## Verification

```bash
matlab -batch "run('matlab-practice/green_joint_current_loop_mbd/run_green_joint_current_loop_smoke_test.m')"
matlab -batch "run('matlab-practice/green_joint_current_loop_mbd/generate_green_joint_current_loop_code.m')"
```

`run_green_joint_current_loop_smoke_test.m` also checks that the generated
interface defaults still match `green-joint/Module/Config/green_joint_1615_config.json`.
This prevents `interface.json`, `.sldd`, and firmware variant defaults from
quietly drifting apart.

`generate_green_joint_current_loop_code.m` also runs:

```text
verify_green_joint_current_pi_codegen()
```

This guard checks that generated C still uses Vd-priority allocation and
back-calculation anti-windup.

Current status:

```text
Smoke test passed.
Smoke test verifies green_joint_1615 variant current-loop defaults.
ERT code generation passed.
Generated interface has no duty/sector/Clarke/Park fields.
Voltage allocation is Vd-priority: Vq only uses the voltage remaining after Vd.
```

## Simulink Desktop Warning

This module rebuilds generated `.slx` and `.sldd` artifacts from scripts. Do not
run rebuild scripts while MATLAB Desktop has the same model or dictionary open.

Preferred flow:

```bash
matlab -batch "run('matlab-practice/green_joint_current_loop_mbd/run_green_joint_current_loop_smoke_test.m')"
matlab -batch "run('matlab-practice/green_joint_current_loop_mbd/generate_green_joint_current_loop_code.m')"
```

If Simulink hangs around model update, read:

```text
../docs/simulink_hang_troubleshooting.md
```
