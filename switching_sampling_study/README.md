# Switching Sampling Study

This folder is a dedicated workspace for studying switch-level PWM behavior,
zero-vector allocation, and current sampling windows.

The goal is different from the existing average-inverter models:

- Focus on switching sequence, not just averaged phase voltage
- Compare different `V0/V7` allocations under the same voltage reference
- Observe how sampling windows change inside one PWM period
- Prepare for later ADC trigger and current reconstruction studies

## Files

- `switching_sampling_study_config.m`: base parameters for PWM, DC bus, modulation, and sampling assumptions
- `build_switching_sampling_study_model.m`: creates a Simulink scaffold model for switch-level sampling research
- `run_switching_sampling_study.m`: loads config and builds the model
- `run_triangle_carrier_study.m`: compares different `V0/V7` allocations using a center-aligned triangular carrier and reports low-side window metrics
- `run_mcu_sampling_window_study.m`: reproduces the MCU sector-based duty clamp and compares PWM/sampling windows before and after the extra common-mode shift
- `run_rl_sampling_impact_study.m`: drives a simple three-phase RL load with the PWM states and compares true phase current with the sampled-current points
- `run_rotating_rl_sampling_study.m`: rotates the electrical angle across whole cycles and compares sampled current with per-period average current over the full trajectory
- `run_saddle_common_mode_study.m`: plots three-phase saddle-wave references and common-mode voltage under different `V0/V7` allocations
- `run_symmetric_v0v7_study.m`: simplified entry point that fixes `V0/V7 = 50%/50%` and studies only the symmetric case

## What the first-stage model contains

- Symmetric `V0/V7 = 50%/50%` duty generator
- Center-aligned triangular carrier
- Three carrier comparators for A/B/C phases
- Low-side gate reconstruction by complementary logic
- Analytical window metrics for each phase:
	- ideal low-side window
	- low-side window after dead time
	- effective window after settling time
- Scope and `To Workspace` logging for carrier, duties, low-side gates, and window metrics

This first-stage model is intentionally simpler than a full switching bridge.
It is meant to build timing intuition first, before adding dead-time, ADC
trigger logic, and the actual power stage.

## Usage

In MATLAB:

```matlab
cd switching_sampling_study
run_switching_sampling_study
```

This builds and opens `switching_sampling_study_model.slx`.

To study center-aligned triangular-carrier PWM first:

```matlab
cd switching_sampling_study
run_triangle_carrier_study
```

To simplify the first stage and study only symmetric zero-vector allocation:

```matlab
cd switching_sampling_study
run_symmetric_v0v7_study
```

To focus on saddle-wave modulation and common-mode voltage:

```matlab
cd switching_sampling_study
run_saddle_common_mode_study
```

To reproduce the MCU sector-based sampling-window clamp:

```matlab
cd switching_sampling_study
run_mcu_sampling_window_study
```

To see whether the MCU shift changes sampled current on a simple RL load:

```matlab
cd switching_sampling_study
run_rl_sampling_impact_study
```

To rotate through a whole electrical cycle and inspect sampled current over angle:

```matlab
cd switching_sampling_study
run_rotating_rl_sampling_study
```

Override the electrical angle or modulation ratio:

```matlab
run_triangle_carrier_study('thetaEDeg', 20, 'modulationRatio', 0.85)
```

## Recommended next steps

1. Insert dead-time and compute effective low-side valid windows.
2. Add ADC trigger logic and mark valid sampling instants.
3. Replace the logic-level PWM stage with `Universal Bridge + powergui + PMSM`.
4. After the symmetric case is understood, re-enable biased `V0` or `V7` allocation for comparison.

## Window metric definitions

For the current first-stage model, the phase low-side window is approximated as:

```matlab
T_low_ideal = (1 - duty) * T_pwm
T_low_dead  = max(T_low_ideal - 2 * dead_time, 0)
T_valid     = max(T_low_dead - settle_time, 0)
```

This is still a timing-level approximation. The next step is to add explicit
ADC trigger placement and then move to `Universal Bridge + PMSM` for full
switch-level validation.