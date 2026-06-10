# Reentrant Simulink Add Example

This example builds a tiny embedded-C Simulink model:

```text
z = x + y
```

The generated code is configured for ERT and reusable/reentrant C interfaces.
It uses the modern Simulink type workflow:

```text
Simulink Data Dictionary -> Simulink.AliasType -> generated typedefs
```

## Files

- `build_simple_add_reentrant_model.m` creates `reentrant_add_model.slx`.
- `add_interface.sldd` is created by the build script and stores interface types.
- `generate_simple_add_reentrant_code.m` generates ERT C code.

## Run

From the repository root:

```bash
matlab -batch "run('simple_add_reentrant/generate_simple_add_reentrant_code.m')"
```

Generated files are written to:

```text
simple_add_reentrant/reentrant_add_model_ert_rtw/
```

## Customer Interface Types

Customers can change the generated C interface types in
`customer_interface_config()` inside `build_simple_add_reentrant_model.m`.
The model uses type names, while the data dictionary stores their definitions:

```matlab
cfg.inputTypeName = 'T_AddIn';
cfg.outputTypeName = 'T_AddOut';
cfg.accumulatorTypeName = 'T_AddAcc';

cfg.inputBaseType = 'single';
cfg.outputBaseType = 'single';
cfg.accumulatorBaseType = 'single';
```

The script creates these entries in `add_interface.sldd` as `Simulink.AliasType`
objects and exports them to `add_types.h`.

Common base type choices:

```matlab
cfg.inputBaseType = 'double';        % C base type: real_T
cfg.inputBaseType = 'single';        % C base type: real32_T
cfg.inputBaseType = 'int16';         % C base type: int16_T
cfg.inputBaseType = 'uint16';        % C base type: uint16_T
cfg.inputBaseType = 'fixdt(1,16,8)'; % signed fixed-point, 16-bit word, 8 fraction bits
```

For fixed-point or integer interfaces, set `accumulatorBaseType` wide enough for
`x + y`, and keep `saturateOnIntegerOverflow = 'on'` when saturated arithmetic
is required.

Key generated files:

- `AddStep.h`
- `AddStep.c`
- `add_types.h`
- `reentrant_add_model.h`
- `reentrant_add_model.c`
- `rtwtypes.h`

## Generated Interfaces

The atomic reusable subsystem generates a pure algorithm function:

```c
extern T_AddOut AddStep(T_AddIn rtu_x, T_AddIn rtu_y);
```

The top model also generates a multi-instance model step function:

```c
extern void reentrant_add_model_step(
    RT_MODEL_reentrant_add_model_T *const reentrant_add_model_M,
    T_AddIn reentrant_add_model_U_x,
    T_AddIn reentrant_add_model_U_y,
    T_AddOut *reentrant_add_model_Y_z);
```

The generated `add_types.h` contains typedefs similar to:

```c
typedef real32_T T_AddIn;
typedef real32_T T_AddOut;
typedef real32_T T_AddAcc;
```

For a simple stateless embedded module, calling `AddStep()` is usually enough.
For a larger model with states, use the model object and `*_step()` interface so
each instance owns its state.

## Embedded Integration Sketch

```c
#include "AddStep.h"

T_AddOut control_tick(T_AddIn x, T_AddIn y)
{
    return AddStep(x, y);
}
```

Multi-instance style:

```c
#include "reentrant_add_model.h"

static RT_MODEL_reentrant_add_model_T add_model;
static T_AddIn x;
static T_AddIn y;
static T_AddOut z;

void app_init(void)
{
    reentrant_add_model_initialize(&add_model, &x, &y, &z);
}

void app_step(void)
{
    reentrant_add_model_step(&add_model, x, y, &z);
}
```
