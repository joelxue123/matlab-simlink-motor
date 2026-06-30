# Motor Performance Characterization

Date: 2026-06-10

This folder collects motor performance checks that are broader than one control
module or one identification experiment.

## Current Focus: Current Sensor Harmonic Regression

Current sensing errors create repeatable signatures in dq currents:

```text
phase current offset / zero drift  -> 1x electrical frequency in id/iq
phase current gain mismatch        -> 2x electrical frequency in id/iq
random current noise               -> broadband noise
PWM sampling ripple                -> PWM frequency and sidebands
deadtime voltage error             -> often visible as 6x electrical torque ripple
```

Electrical frequency and mechanical frequency are related by:

```text
f_e = pole_pairs * f_m
```

So:

```text
1x electrical = pole_pairs x mechanical
2x electrical = 2 * pole_pairs x mechanical
```

## Regression Test

Run:

```bash
matlab -batch "run('motor_performance_characterization/run_current_sensor_harmonic_regression_test.m')"
```

Generated artifacts:

```text
results/current_sensor_harmonic_metrics.csv
results/current_sensor_harmonic_regression_report.txt
results/current_sensor_harmonic_regression.png
results/current_sensor_fault_dq_timeseries.png
```

The test injects these faults into ideal balanced three-phase currents:

```text
nominal
offset_fault
gain_mismatch_fault
noise_fault
pwm_ripple_fault
```

The automatic pass criteria are intentionally focused:

```text
offset_fault must have dominant 1x electrical ripple
gain_mismatch_fault must have dominant 2x electrical ripple
nominal must have negligible 1x and 2x ripple
```

Noise and PWM cases are recorded for inspection, but their exact spectrum
depends on sampling and PWM choices.

## Why This Matters

Before blaming the motor for torque ripple, check the measurement chain:

```text
current offset
current gain mismatch
ADC noise
PWM sampling point
deadtime compensation
encoder angle quality
```

This regression is the first guardrail for that workflow.

## Current-Torque Curve

Run:

```bash
matlab -batch "run('motor_performance_characterization/run_current_torque_curve.m')"
```

Generated artifacts:

```text
results/current_torque_curve_data.csv
results/current_torque_curve_report.txt
results/current_torque_curve.png
```

This script stores the measured hanging-mass points, derives:

```text
phase current RMS
input power
mechanical output power
efficiency
```

and fits a linear current-torque relation. The current conversion assumption is:

```text
phase_current_rms_A = line_current_peak_A / sqrt(2)
```
