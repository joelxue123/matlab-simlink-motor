# Three Discretization Comparison

This folder builds a standalone Simulink model to compare three discrete implementations of the first-order system

```math
\dot{x} = -a x + b u
```

against the continuous-time reference.

Included methods:

- Forward Euler
- Backward Euler
- Exact ZOH discretization

## Files

- `build_discretization_compare_model.m`: creates `discretization_compare_model.slx`
- `run_discretization_compare.m`: builds, simulates, plots, and reports RMSE

## Default formulas

Continuous model:

```math
\dot{x} = -a x + b u, \quad y = x
```

Forward Euler:

```math
x[k+1] = (1-aT)x[k] + bT u[k]
```

Backward Euler:

```math
x[k+1] = \frac{1}{1+aT}x[k] + \frac{bT}{1+aT}u[k]
```

Exact ZOH:

```math
x[k+1] = e^{-aT}x[k] + \frac{b}{a}(1-e^{-aT})u[k]
```

When `a = 0`, the ZOH input term reduces to `bT`.

## Usage

In MATLAB:

```matlab
cd discretization_compare
run_discretization_compare
```

Override parameters:

```matlab
run_discretization_compare('a', 120, 'b', 1, 'Ts', 5e-4, 'StopTime', 0.08)
```

## What the model contains

- One continuous State-Space block as the reference
- One Zero-Order Hold block to sample the input for all discrete branches
- Three Discrete State-Space blocks, one per discretization method
- A Scope and To Workspace logging blocks for comparison

This keeps the comparison entirely in standard Simulink blocks, with no custom S-functions.