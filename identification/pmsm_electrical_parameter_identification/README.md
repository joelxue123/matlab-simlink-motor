# PMSM Electrical Parameter Identification

Date: 2026-06-10

This sandbox estimates PMSM electrical parameters and encoder alignment terms
for later sensorless/sensored cooperation.

## Goals

Identify:

```text
Rs
Ld
Lq
psi_f / flux linkage
encoder electrical offset
encoder residual angle harmonics
```

Future use:

```text
sensorless observer initialization
sensorless/sensored angle comparison
encoder offset alignment
encoder nonlinearity compensation
field weakening and MTPA map quality checks
```

## Model

Steady dq voltage model:

```text
vd = Rs*id + Ld*did/dt - we*Lq*iq
vq = Rs*iq + Lq*diq/dt + we*(Ld*id + psi_f)
```

Standstill step tests:

```text
we = 0
vd = Rs*id + Ld*did/dt
vq = Rs*iq + Lq*diq/dt
```

Flux linkage spin test:

```text
psi_f ~= (vq - Rs*iq - Lq*diq/dt) / we - Ld*id
```

Encoder alignment:

```text
theta_encoder = theta_true + offset + nonlinearity
theta_sensorless ~= theta_true
offset = circular_mean(theta_encoder - theta_sensorless)
```

## Run

```bash
matlab -batch "run('identification/pmsm_electrical_parameter_identification/run_pmsm_electrical_id_demo.m')"
```

Interactive MATLAB usage:

```matlab
cfg = pmsm_electrical_id_config();
data = synthesize_pmsm_electrical_id_data(cfg);
result = identify_pmsm_electrical_params(data, cfg)

% Convenience form: generate default synthetic data and identify directly.
result = identify_pmsm_electrical_params()
```

Build the Simulink waveform model:

```bash
matlab -batch "run('identification/pmsm_electrical_parameter_identification/build_pmsm_electrical_id_waveform_model.m')"
```

Then open:

```text
pmsm_electrical_id_waveform_model.slx
```

The model contains three waveform inspection subsystems:

```text
Standstill_RL_Step_Test
Flux_Linkage_Spin_Test
Encoder_Alignment_Test
```

Generated artifacts:

```text
results/electrical_step_data.csv
results/flux_spin_data.csv
results/encoder_angle_data.csv
results/pmsm_electrical_id_report.txt
results/pmsm_electrical_id_summary.png
results/pmsm_electrical_id_test_conditions.txt
pmsm_electrical_id_waveform_model.slx
```

## Current Scope

This first version uses synthetic data. It proves the estimator path before
connecting real bench CSV. It does not generate C and does not build Simulink
models yet.

Next MBD milestone:

```text
Simulink plant + test harness
  -> Data Dictionary contracts
  -> estimator module
  -> sensorless/sensored angle comparison module
```
