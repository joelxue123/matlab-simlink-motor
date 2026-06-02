# Zero Response Study

This folder contains a compact MATLAB study for showing how transfer-function
zeros change transient response while the dominant poles stay fixed.

## Entry point

In MATLAB:

```matlab
cd zero_response_study
run_zero_transient_study
```

If you want a minimal example where the closed-loop poles stay identical but
the overshoot changes only because of zeros:

```matlab
cd zero_response_study
run_same_poles_zero_example
```

The script compares:

- no zero
- several left-half-plane zeros
- one right-half-plane zero

All cases are normalized to the same DC gain, so the comparison stays focused
on transient shape instead of static gain.

## Useful overrides

```matlab
run_zero_transient_study('wnRadS', 60, 'zeta', 0.5)
run_zero_transient_study('lhpZeroRadS', [300 120 40], 'rhpZeroRadS', 15)
run_zero_transient_study('includeRhpZero', false, 'tEndS', 0.3)
```

For the minimal same-poles example:

```matlab
run_same_poles_zero_example('zeta', 0.6, 'leftZeroNearRadS', 15)
```

## What to watch

- LHP zero farther left: effect becomes weaker, response approaches the no-zero case
- LHP zero closer to the origin: faster initial rise, usually more overshoot
- RHP zero: inverse response, output first moves opposite to the final direction