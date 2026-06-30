# Motor Current Filter MBD

This directory keeps the legacy green-joint dq current-feedback filter as a small MBD experiment.

## Scope

Legacy milestone:

```text
raw_id/raw_iq
  -> CurrentFilterStep
  -> id_f/iq_f
```

This module models the dq-domain feedback low-pass that used to be in `green-joint`.
As of 2026-06-29, the firmware mainline disables this filter and feeds raw Park
`motor_i_d / motor_i_q` directly into the current PI.

Validation status:

- smoke test passed
- ERT code generation passed
- generated reusable function interface confirmed

Out of scope for this first milestone:

- abc-sector blending inside `platform_adc`
- full current PI controller
- inverter/plant integration

## Why this module still exists

The firmware no longer applies this adaptive dq low-pass after Park transform. The
MBD module is kept as a reusable/legacy asset so that, if filtering is restored,
it is restored through an explicit model instead of ad-hoc C code in `main.c`.

- the filter contract is visible
- the parameters live in `.sldd`
- the code can be regenerated later
- the same logic can be tested independently

## Files

- `build_current_filter_model.m` builds `current_filter_model.slx` and `current_filter_interface.sldd`
- `run_current_filter_smoke_test.m` builds and simulates the model, then checks `alpha`, `id_f`, and `iq_f`

## Run

From repository root:

```bash
matlab -batch "run('matlab-practice/motor_current_filter_mbd/run_current_filter_smoke_test.m')"
```

## Planned interface

```text
current_filter_input_t {
  id_raw
  iq_raw
  v_mag_norm
}

current_filter_output_t {
  id_f
  iq_f
  alpha
}
```

## Notes

- The first version is float-first.
- `alpha` is adaptive and derived from `v_mag_norm`.
- The module should stay reusable and reentrant.
