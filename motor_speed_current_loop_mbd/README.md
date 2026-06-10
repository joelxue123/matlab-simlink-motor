# Motor Speed + Current Loop MBD

This directory contains the first full float FOC integration milestone:

```text
SpeedPiStep
  -> CurrentPiStep
  -> DqToAbcDutyStep
  -> Average-Value Inverter
  -> Surface Mount PMSM
  -> MotorClarkeParkStep feedback
```

The plant and inverter are simulation harness blocks. The reusable
controller-facing subsystems remain the embedded-code candidates.

## Run

From repository root:

```bash
matlab -batch "run('motor_speed_current_loop_mbd/run_speed_current_loop_smoke_test.m')"
```

The build script creates:

```text
motor_speed_current_loop_interface.sldd
motor_speed_current_loop_model.slx
```

## Architecture

```text
wm_ref step
  -> speed_pi_input_t
  -> SpeedPiStep
  -> speed_pi_output_t.iq_ref
  -> Rate Transition 100us to 50us
  -> current_pi_input_t
  -> CurrentPiStep
  -> current_pi_output_t.vd_ref/vq_ref
  -> DqToAbcDutyStep
  -> phase_duty_t [0, 1]
  -> Rate Transition 50us to 25us
  -> Average-Value Inverter
  -> Surface Mount PMSM
  -> plant_feedback_t
  -> Rate Transition 25us to controller rates
  -> MotorClarkeParkStep
  -> id_meas / iq_meas feedback
```

## Sample Times

```text
Plant / Average Inverter: 25 us
Current loop:             50 us
Speed loop:              100 us
```

Important Rate Transition blocks:

- `PhaseDuty_RateTransition_25us`
- `Vdc_RateTransition_25us`
- `ia_feedback_rt_50us`
- `theta_e_feedback_rt_50us`
- `wm_feedback_rt_100us`
- `IqRef_RateTransition_50us`

## Current Smoke Test Result

```text
wm_ref final   = 41.8879 rad/s
wm_meas final  = 41.8883 rad/s
speed error    = -0.000385284 rad/s
iq_ref range   = [-0.949188, 15] A
iq_meas range  = [-0.879125, 14.6738] A
id_meas range  = [-0.169437, 0.371049] A
vd_ref range   = [-1.2048, 0] V
vq_ref range   = [0, 20.7848] V
duty range     = [0.235291, 0.764709]
iq_limit       = 15 A
Speed-current-loop smoke test passed.
```

## Design Notes

- Use `.sldd + Simulink.AliasType('single') + Simulink.Bus` for module
  contracts.
- Keep Average-Value Inverter duty in `[0, 1]`.
- Keep speed-loop output as `iq_ref`; do not merge speed PI and current PI into
  one giant subsystem.
- Cross every different-rate boundary with explicit Rate Transition blocks.
- The default test duration is `0.060s` so the smoke test remains practical.
- `ParameterPrecisionLossMsg` is set to `none` for this integration model
  because exact single representation warnings for constants such as `1/3`,
  `sqrt(3)/2`, and sample times are expected in this float-first milestone.
