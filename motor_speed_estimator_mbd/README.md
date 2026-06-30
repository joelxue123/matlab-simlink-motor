# Motor Speed Estimator MBD

This module is the first PLL-based MBD version of the green-joint speed
estimator.

It implements a PLL/tracking observer from motor mechanical angle to:

```text
joint_speed_est_rad_s
motor_speed_mech_rad_s
motor_speed_elec_rad_s
theta_hat_rad
```

The production target has replaced the firmware-only finite-difference plus
IIR speed path. The model is intentionally small so it can become a
library/model-reference block in the main green-joint digital twin.

## Architecture

This follows the ODrive encoder observer shape:

```text
theta_hat[k+1|k] = wrap(theta_hat[k] + Ts * omega_hat[k])
theta_err        = wrap_pi(theta_meas[k] - theta_hat[k+1|k])
theta_hat[k+1]   = wrap(theta_hat[k+1|k] + Ts * PllKp * theta_err)
omega_hat[k+1]   = omega_hat[k] + Ts * PllKi * theta_err
```

It is not a finite-difference speed plus low-pass filter. The observer uses
angle residual feedback to update both position and speed estimates.

ODrive's public v3.x firmware implements the observer in encoder-count units:
position is predicted from velocity, the wrapped CPR error corrects both
position and velocity, and very low estimated velocity is snapped to zero to
avoid standstill jitter. This MBD module keeps that structure but exposes a
green-joint contract in motor mechanical radians:

```text
input:  motor_angle_rad
state:  theta_hat_rad, omega_hat_rad_s
output: joint_speed_est_rad_s = omega_hat_rad_s * InvGearRatio
```

Do not fork this estimator inside speed-loop or position-loop harnesses. Add
future output-encoder fusion here, then reference this module from the digital
twin.

Angle wrapping is intentionally implemented with hand-written loop logic:

```text
while angle >= 2*pi: angle -= 2*pi
while angle < 0:     angle += 2*pi
```

Do not use Simulink `Math Function` with `mod` for production angle wrapping
in this module. It generates `fmodf`/`rt_modf_snf` on STM32, which is slower
and less predictable than the bounded loop used by the firmware
`wrap_0_to_2pi_loop` convention.

## Commands

```bash
matlab -batch "run('matlab-practice/motor_speed_estimator_mbd/run_speed_estimator_pll_smoke_test.m')"
matlab -batch "run('matlab-practice/motor_speed_estimator_mbd/generate_speed_estimator_pll_code.m')"
```

## Current Defaults

```text
sample_time_s = 50 us
pll_bandwidth_hz = 360 Hz
pll_damping = 1.0
gear_ratio = 183.35
pole_pairs = 2
```

The PLL gains are:

```text
PllKp = 2 * damping * 2*pi*bandwidth
PllKi = (2*pi*bandwidth)^2
```

This is a green-joint tuning convention. ODrive's public v3.x code uses its
own `config_.bandwidth` convention, so do not copy ODrive numeric bandwidth
values into this model without unit conversion and simulation.

The current 360 Hz default is the hardware-noise bring-up value. The previous
600 Hz candidate looked good in the noiseless V1 average-motor speed-step
sweep, but hardware feedback showed visibly larger velocity noise. Since
`PllKi = (2*pi*bw)^2`, reducing 600 Hz to 360 Hz cuts the angle-noise-to-speed
gain to about 36% while keeping much less lag than the old 120 Hz reference.

Keep 600 Hz as a high-response A/B candidate only after checking encoder noise
and low-speed snap behavior on hardware.

The generated dictionary also stores:

```text
SpeedEstimatorSampleTime = 50e-6
GearRatio                = 183.35
InvGearRatio             = 1/GearRatio
ZeroSpeedThresholdRadS   = ODrive-style low-speed snap threshold
```

`InvGearRatio` is intentional: the firmware adapter should use multiplication
instead of dividing in the 20 kHz ISR.

Firmware integration lives in:

```text
green-joint/Module/MBD/green_joint_speed_estimator/
green-joint/Module/Src/green_joint_speed_estimator_mbd_adapter.c
green-joint/Core/Src/main.c
```

Current green-joint mainline always uses this estimator. The legacy diff + IIR
path has been removed from the firmware mainline; keep it only as historical
analysis or offline baseline material.

Reset contract:

```text
input reset uses uint8, where 0 = run and nonzero = align theta_hat to the
current motor_angle_rad and clear omega_hat.
```

Do not model this reset as a uint8 Switch with a `0.5` threshold. Embedded Coder
can quantize that threshold to `1U`, so a firmware reset value of `1U` would not
trigger. The current build script uses `u2 ~= 0` for the reset switches, so the
generated C directly tests `reset != 0` and no reset threshold parameter is
generated.
