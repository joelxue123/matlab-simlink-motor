# 12V Coreless Motor Small-Inertia Identification

Date: 2026-06-10

This note records the first literature-driven plan for identifying parameters of
a small 12V coreless motor. It is intended to be the MATLAB/Simulink simulation
entry. Hardware execution details should stay linked to the HPM board workflow.

Related hardware plan:

```text
/home/user/study/AI+MOTOR/HPM6E00EVK-RevC/INERTIA_IDENTIFICATION_PLAN.md
```

## Scope

The immediate target is a small-inertia 12V coreless motor. If the actual motor
is brushed coreless DC, use the PMDC model. If it is brushless coreless BLDC or
PMSM, the electrical model changes, but the mechanical identification ideas are
the same.

The most useful first model is:

```text
v = R*i + L*di/dt + Ke*w
Te = Kt*i
Te - Tload - B*w - Tc*sign(w) - Tbias = J*dw/dt
```

Where:

```text
R      armature resistance
L      armature inductance
Ke     back-EMF constant
Kt     torque constant
J      equivalent inertia
B      viscous friction
Tc     Coulomb friction
Tbias  static bias / gravity / offset torque
w      mechanical speed
```

For a DC motor in SI units, `Kt` and `Ke` are numerically consistent after unit
conversion. For real experiments, treat them as separate identified values until
the data quality is proven.

## Literature Takeaways

The literature does not point to one magic formula. It points to a workflow:

1. identify electrical parameters first;
2. use controlled excitation for mechanical parameters;
3. estimate inertia together with friction, not alone;
4. validate by predicting a different experiment than the one used for fitting.

Useful references:

- DC motor dynamic identification with step and sinusoidal inputs:
  <https://pmc.ncbi.nlm.nih.gov/articles/PMC12788085/>
- DC motor parameter identification from speed step response:
  <https://www.hindawi.com/journals/mpe/2012/189757/>
- PMDC parameter extraction review:
  <https://www.mdpi.com/2079-9292/12/12/2559>
- PMSM parameter identification review, useful when the motor/drive is BLDC or
  PMSM rather than brushed DC:
  <https://www.mdpi.com/2076-0825/10/7/143>
- MathWorks PMSM mechanical parameter estimator, useful as an engineering
  reference for estimating inertia and damping in an MBD workflow:
  <https://www.mathworks.com/help/mcb/ref/pmsmmechanicalparameterestimator.html>
- Manufacturer background on coreless motors and their low-inertia behavior:
  <https://www.portescap.com/en/products/brush-dc-motors/coreless-dc-motors>

## Why 12V Coreless Small Inertia Is Difficult

Small inertia is not just a smaller number. It changes the measurement problem.

```text
J small -> acceleration large -> useful transient is short
short transient -> sampling delay and timestamp error become visible
speed differentiation -> noise amplification
encoder quantization -> acceleration estimate becomes unstable
friction / brush torque / driver deadband -> comparable to inertia torque
12V bus -> voltage saturation appears quickly at speed because of back-EMF
low winding inductance -> current changes fast; slow current sampling misses it
```

Therefore, avoid this naive method as the primary estimator:

```text
J = Te / diff(speed)
```

It is too sensitive to speed noise, time-base error, friction, and current-loop
delay. It can be used only as a rough sanity check after better filtering.

## Parameters To Identify

Identify the parameters in this order:

```text
1. R
2. L
3. Ke
4. Kt
5. B and Tc
6. J
7. driver delay, voltage loss, current limit, deadband
```

Do not start with `J`. If `Te = Kt*i` is wrong, every inertia result will be
wrong.

## Recommended Methods

### R and L

Use a locked-rotor or very-low-speed current step, with current sampled much
faster than the expected electrical time constant:

```text
i(t) = V/R * (1 - exp(-t*R/L))
tau_e = L/R
```

Rules:

```text
use small voltage/current
make pulse short
monitor temperature
avoid long locked-rotor heating
```

For tiny coreless motors, `L` may be very small, so a slow 1 kHz log may not be
enough to estimate `L`. In that case, rely on datasheet `L` first and identify
the slower mechanical parameters from motion data.

### Ke

Use no-load spin data:

```text
Ke ~= (V - R*i) / w
```

Prefer a sweep over several steady speeds, not one point. Avoid points where the
driver is saturated or current measurement is poor.

### Kt

Best options:

```text
torque sensor + current
known load torque + current
datasheet cross-check
Kt ~= Ke after SI unit conversion
```

Without current measurement, `Kt` cannot be separated cleanly from driver gain.

### B and Tc

Use coast-down:

```text
0 = J*dw/dt + B*w + Tc*sign(w) + Tbias
```

Coast-down alone cannot determine absolute `J` unless `J` is already known. It
is still useful for learning friction shape and for checking whether friction is
large compared with inertia torque.

### J

Primary method:

```text
bidirectional torque pulse
  + position-based acceleration fitting
  + least-squares mechanical model
```

For each short window, fit position instead of differentiating speed:

```text
theta(t) = theta0 + w0*t + 0.5*a*t^2
```

Then estimate:

```text
Te = J*a + B*w + Tc*sign(w) + Tbias
```

Use positive and negative torque pulses. This helps cancel bias torque, driver
offset, and asymmetric friction.

If the estimated `J` changes strongly with pulse amplitude, the experiment is
not yet identifying pure inertia. Likely causes:

```text
current loop saturation
voltage saturation
friction not modeled
bad timestamp
encoder quantization
too much filtering delay
```

### Added-Inertia Method

For very small motors, add a known inertia:

```text
J_total = J_motor + J_known
J_motor = J_total - J_known
```

This is often the most practical way to improve accuracy. The known inertia
makes the transient slower and easier to measure. The tradeoff is that the
coupling and alignment must be good.

## 12V Test Rules

Start with a conservative experiment:

```text
bus voltage: 12V
initial current: 10% to 20% of rated current
short pulse: 20 ms to 100 ms, adjusted by observed speed rise
repeat positive and negative pulses
stop before high speed and obvious back-EMF saturation
record current, voltage or duty, position, speed, error flags, temperature
```

The current command or torque command is more valuable than voltage command. If
only voltage/duty command is available, the identification must include the
electrical equation and driver model, which is harder and less reliable.

## MATLAB Estimator Skeleton

For HPM merged CSV data, use the command sequence as the time base:

```matlab
T = readtable("merged_result.csv");
t = T.cmd_seq * 0.001;
```

Use valid samples only:

```matlab
valid = T.error_code == 0 & isfinite(T.position_rad);

if ismember("feedback_valid", T.Properties.VariableNames)
    valid = valid & T.feedback_valid == 1;
end
```

Fit acceleration from position inside one pulse:

```matlab
tp = t(idx) - t(idx(1));
theta = T.position_rad(idx);

P = [ones(size(tp)), tp, 0.5*tp.^2];
c = P \ theta;
a = c(3);
```

Build the mechanical least-squares model:

```matlab
Phi = [a_vec(:), w_vec(:), sign(w_vec(:)), ones(numel(w_vec), 1)];
y = Te_vec(:);

p = Phi \ y;
J     = p(1);
B     = p(2);
Tc    = p(3);
Tbias = p(4);
```

Report at least:

```text
J mean and standard deviation
B, Tc, Tbias
fit residual
number of valid windows
current/torque command range
bus voltage
sampling time
filter settings
```

## Simulation-First MBD Plan

Build the work in this order:

```text
1. MATLAB script: synthesize 12V coreless motor data with known R/L/Ke/Kt/J/B/Tc.
2. MATLAB estimator: recover parameters from synthetic noisy data.
3. Simulink plant: motor + driver limits + sensor quantization.
4. Simulink test harness: torque pulse and coast-down experiments.
5. Hardware CSV importer: same estimator reads HPM merged_result_csv.
6. Report generator: compare true/sim/identified parameters.
```

Use `.sldd` later for reusable parameter contracts:

```text
coreless_motor_param_t
driver_param_t
identification_test_config_t
```

This keeps the simulation model, estimator, and generated C work aligned with
the MBD style already used in this repository.

## Current MATLAB Sandbox

The first executable milestone is a synthetic-data estimator. It does not need
hardware and does not generate C. It verifies the algorithm before building a
Simulink plant.

Files:

```text
coreless_motor_12v_config.m
synthesize_coreless_motor_12v_data.m
identify_coreless_motor_mechanics.m
run_coreless_motor_identification_demo.m
plot_coreless_motor_identification_results.m
results/README.md
```

Run:

```bash
matlab -batch "run('identification/coreless_motor_12v_identification/run_coreless_motor_identification_demo.m')"
```

The demo generates:

```text
results/synthetic_coreless_motor_12v_data.csv
results/identification_windows.csv
results/identification_report.txt
results/coreless_identification_timeseries.png
results/coreless_identification_windows.png
```

Important implementation decisions:

```text
use measured current i_meas_A, not voltage_cmd, for mechanical identification
simulate current-loop lag and current deadband
fit acceleration from position over pulse plateaus
discard edge samples after each current pulse transition
include zero-current coast segments to help friction estimation
split each segment into short fitting windows to improve speed coverage
estimate J/B/Tc/Tbias by least squares
```

Current validation result:

```text
Command:
  matlab -batch "run('identification/coreless_motor_12v_identification/run_coreless_motor_identification_demo.m')"

Result on 2026-06-10:
  window_count      = 60
  J true / estimate = 1.2e-06 / 1.21400e-06 kg*m^2
  J error           = 1.17%
  B true / estimate = 2.0e-06 / 3.00958e-06 Nm/(rad/s)
  Tc true / estimate = 1.5e-04 / 1.03505e-04 Nm
```

Known limitation:

```text
The current synthetic pulse/coast experiment identifies J well, but B/Tc are
still weaker than J. Do not treat the friction estimates as final. The next
experiment should add a dedicated long coast-down or multi-speed low-current
test with wider speed coverage.
```

This is intentionally the first layer only. The next layer should reuse the
same config and estimator around a Simulink plant/test harness.

## Current Decision

Most valuable first path:

```text
electrical parameter sanity check
  -> simulation-first estimator
  -> bidirectional torque pulse with position fitting
  -> friction-aware least squares
  -> added known inertia if the raw motor is too fast
```

This is better than immediately hand-writing embedded identification code. The
first deliverable should be a MATLAB/Simulink identification sandbox. Embedded C
comes later after the estimator is stable.
