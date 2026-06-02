# Z Stability Demo

This folder contains a minimal MATLAB demo for validating discrete-time stability analysis in the z-domain.

Files:

- `analyze_z_stability.m`: computes poles, zeros, unit-circle stability, frequency response, phase margin, and gain margin.
- `plot_z_analysis.m`: plots unit-circle pole-zero map and frequency response.
- `run_z_stability_demo.m`: runs three examples: stable, marginal, and unstable.
- `compare_s_z_models.m`: compares a continuous-time `s` model and a discrete-time `z` model side by side.

Usage in MATLAB:

```matlab
cd z_stability_demo
run_z_stability_demo
```

Compare `s` and `z` models:

```matlab
cd z_stability_demo
compare_s_z_models
```

Coefficient convention:

- `num = [b0 b1 ...]` means `b0 + b1 z^-1 + ...`
- `den = [1 a1 ...]` means `1 + a1 z^-1 + ...`

Example:

```matlab
Ts = 1/10000;
result = analyze_z_stability([1 -1], [1 -0.95], Ts);
plot_z_analysis(result, 'Example: (z-1)/(z-0.95)');
```

The comparison script uses the example

```matlab
Hz(z) = (1 - z^-1) / (1 - 0.95 z^-1)
```

and the equivalent continuous washout approximation

```matlab
Hs(s) = (tau s) / (1 + tau s),  tau = -Ts / log(0.95)
```

to compare Bode curves, step response, and impulse response.