# Motor Clarke/Park Struct Example

This example shows why `.sldd` is useful for struct-style embedded C
interfaces.

Algorithm:

```text
motor_t { ia, ib, ic, theta_e } -> motor_dq_t { i_alpha, i_beta, id, iq }
```

The model performs Clarke + Park transform:

```text
i_alpha = 2/3 * (ia - 0.5*ib - 0.5*ic)
i_beta  = 1/sqrt(3) * (ib - ic)
id      = i_alpha*cos(theta_e) + i_beta*sin(theta_e)
iq      = -i_alpha*sin(theta_e) + i_beta*cos(theta_e)
```

## Files

- `build_motor_clarke_park_model.m` creates the `.sldd` and Simulink model.
- `generate_motor_clarke_park_code.m` runs `slbuild`.
- `run_motor_clarke_park_function_test.m` creates and runs a visual Simulink
  functional test harness.
- `build_motor_open_loop_integration_model.m` creates a top-level model that
  connects an open-loop input module to `MotorClarkeParkStep` through `motor_t`.
- `motor_interface.sldd` is generated/updated by the build script.

## Run

From the repository root:

```bash
matlab -batch "run('motor_clarke_park_struct/generate_motor_clarke_park_code.m')"
```

Generated files are written to:

```text
motor_clarke_park_struct/motor_clarke_park_model_ert_rtw/
```

## Functional Test

Run:

```bash
matlab -batch "run('motor_clarke_park_struct/run_motor_clarke_park_function_test.m')"
```

The test builds a temporary harness, feeds four known `motor_t` inputs, and
compares `i_alpha`, `i_beta`, `id`, and `iq` against a MATLAB reference
calculation. It also saves a visual harness that can be opened in Simulink:

```text
motor_clarke_park_struct/motor_clarke_park_function_test_harness.slx
```

The harness contains:

- scalar Constant blocks for `ia`, `ib`, `ic`, and `theta_e`
- `motor_bus_creator` to assemble the `motor_t` input bus
- `MotorClarkeParkStep` subsystem under test
- `dq_selector` to split `motor_dq_t`
- Display blocks for `i_alpha`, `i_beta`, `id`, and `iq`
- To Workspace blocks for automated comparison

Important Bus Creator rule:

```text
The signal names entering Bus Creator must match the Bus element names.
```

For `motor_t`, the input signal names must be:

```text
ia
ib
ic
theta_e
```

If they remain the defaults `signal1`, `signal2`, etc., Simulink reports a bus
mismatch because `signal1` does not match `motor_t.ia`.

Current result:

```text
Maximum error: 0.000244
Motor Clarke/Park functional test passed.
Saved visual test harness:
  motor_clarke_park_struct/motor_clarke_park_function_test_harness.slx
```

## Integration Example

Run:

```bash
matlab -batch "run('motor_clarke_park_struct/build_motor_open_loop_integration_model.m')"
```

This creates:

```text
motor_clarke_park_struct/motor_open_loop_integration_model.slx
```

The top-level connection is:

```text
OpenLoopMotorInputStep -> motor_t -> MotorClarkeParkStep -> motor_dq_t
```

The important part is the interface contract:

- `OpenLoopMotorInputStep` output port is `Bus: motor_t`.
- `MotorClarkeParkStep` input port is `Bus: motor_t`.
- Both modules use the same `motor_interface.sldd`.
- The signal line between modules carries the whole `motor_t` struct.

If an upstream module naturally outputs `ia`, `ib`, `ic`, and `theta_e` as
separate scalar signals, add a small adapter layer that uses Bus Creator to
assemble `motor_t`. If an upstream module outputs a different structure, add an
adapter that maps that structure into `motor_t`. Avoid forcing unrelated modules
to know each other's internal signal names.

## Customer Interface

Change scalar interface types in `customer_interface_config()`:

```matlab
cfg.currentTypeKind = 'fixed';
cfg.currentSignedness = 'Signed';
cfg.currentWordLength = 16;
cfg.currentFractionLength = 12;
cfg.angleBaseType = 'single';
```

`T_MotorCurrent` is now a `Simulink.NumericType` fixed-point type:

```text
sfix16_En12
```

This means:

- signed 16-bit storage
- 12 fractional bits
- LSB = `2^-12 = 0.000244140625`
- approximate range = `[-8, 7.999755859375]`

The angle stays `single` for now because a full fixed-point angle path usually
needs normalized angle units and a sin/cos lookup table design.

The model keeps using stable type and struct names:

```matlab
cfg.currentTypeName = 'T_MotorCurrent';
cfg.angleTypeName = 'T_MotorAngle';
cfg.inputBusName = 'motor_t';
cfg.outputBusName = 'motor_dq_t';
```

The script stores these in `motor_interface.sldd`:

- `T_MotorCurrent` as `Simulink.NumericType`
- `T_MotorAngle` as `Simulink.AliasType`
- `motor_t` as `Simulink.Bus`
- `motor_dq_t` as `Simulink.Bus`

Expected generated C shape:

```c
typedef int16_T T_MotorCurrent;
typedef real32_T T_MotorAngle;

typedef struct {
  T_MotorCurrent ia;
  T_MotorCurrent ib;
  T_MotorCurrent ic;
  T_MotorAngle theta_e;
} motor_t;

typedef struct {
  T_MotorCurrent i_alpha;
  T_MotorCurrent i_beta;
  T_MotorCurrent id;
  T_MotorCurrent iq;
} motor_dq_t;
```

The reusable algorithm interface should look like:

```c
extern void MotorClarkeParkStep(const motor_t *rtu_motor_in,
                                motor_dq_t *rty_dq_out);
```

This is a good embedded C shape: the input struct is read through a `const`
pointer, and the output struct is written through an output pointer.

## Why `.sldd` Helps Here

Without a data dictionary, the Bus and scalar type definitions would usually be
scattered across initialization scripts or the base workspace. With `.sldd`:

- `motor_t` and `motor_dq_t` live with the project.
- `T_MotorCurrent` and `T_MotorAngle` are reused by every struct field.
- Changing current precision from `single` to fixed-point is done at the type
  definition source.
- Generated C keeps stable names even when base types change.
- Fixed-point storage, word length, fraction length, exported typedef name, and
  generated header file are controlled by the dictionary entry.

The diagram intentionally stays focused on the algorithm. It does not need a
Data Type Conversion block at every operation, and it does not need visible
`Q12` labels throughout the model. The fixed-point contract comes from
`T_MotorCurrent` in the data dictionary, and Embedded Coder carries that
contract into generated C code as integer storage, scaling, shifts, and
saturation.

Note: MATLAB may warn that constants such as `2/3`, `1/sqrt(3)`, and some test
inputs lose precision when represented as `sfix16_En12`. That is expected for
this fixed-point demonstration. If this were production motor-control code, the
next design step would be range analysis, accumulator sizing, rounding choices,
overflow policy, and a fixed-point angle/sin/cos strategy.
