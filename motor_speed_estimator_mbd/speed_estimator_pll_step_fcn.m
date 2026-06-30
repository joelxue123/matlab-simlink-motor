function [joint_speed_est_rad_s, motor_speed_mech_rad_s, motor_speed_elec_rad_s, theta_hat_rad] = speed_estimator_pll_step_fcn(motor_angle_rad, reset)
%#codegen
persistent theta_hat omega_hat initialized

if isempty(initialized) || reset ~= uint8(0)
    theta_hat = single(motor_angle_rad);
    omega_hat = single(0.0);
    initialized = true;
end

err = wrap_pi(single(motor_angle_rad) - theta_hat);
theta_pred = wrap_0_2pi(theta_hat + single(SpeedEstimatorSampleTime) * ...
    omega_hat);
err = wrap_pi(single(motor_angle_rad) - theta_pred);
theta_hat = wrap_0_2pi(theta_pred + single(SpeedEstimatorSampleTime) * ...
    single(PllKp) * err);
omega_hat = omega_hat + single(SpeedEstimatorSampleTime) * ...
    single(PllKi) * err;
if abs(omega_hat) < single(ZeroSpeedThresholdRadS)
    omega_hat = single(0.0);
end

motor_speed_mech_rad_s = single(omega_hat);
joint_speed_est_rad_s = single(omega_hat * single(InvGearRatio));
motor_speed_elec_rad_s = single(omega_hat * single(PolePairs));
theta_hat_rad = single(theta_hat);
end

function y = wrap_pi(x)
pi_v = single(3.1415926535897932385);
y = wrap_0_2pi_loop(single(x) + pi_v) - pi_v;
end

function y = wrap_0_2pi(x)
y = wrap_0_2pi_loop(x);
end

function y = wrap_0_2pi_loop(x)
two_pi = single(6.2831853071795864769);
y = single(x);
while y >= two_pi
    y = y - two_pi;
end
while y < single(0.0)
    y = y + two_pi;
end
end
