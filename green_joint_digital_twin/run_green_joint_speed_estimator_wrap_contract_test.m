%% Validate SpeedEstimator angle wrap contract for green-joint digital twin
%
% Contract:
%   theta_meas/theta_pred/theta_hat stay in [0, 2*pi)
%   theta_err uses the shortest signed path in [-pi, pi)
%
% This is a V1-side contract test for the MBD SpeedEstimator. The production
% MBD module still owns code generation; this script validates the system
% semantics that digital-twin scenarios depend on.

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
results_dir = fullfile(script_dir, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

cfg.sample_time_s = 50e-6;
cfg.pll_bandwidth_hz = 360.0;
cfg.damping = 1.0;
cfg.stop_time_s = 0.010;
cfg.motor_speed_rad_s = 900.0;
cfg.initial_positive_crossing_rad = 2 * pi - 0.02;
cfg.initial_negative_crossing_rad = 0.02;
cfg.angle_tolerance_rad = 1e-7;
cfg.error_tolerance_rad = 1e-6;
cfg.speed_tolerance_rad_s = 1e-3;

omega_n = 2 * pi * cfg.pll_bandwidth_hz;
cfg.pll_kp = 2 * cfg.damping * omega_n;
cfg.pll_ki = omega_n ^ 2;

case_rows = validate_static_wrap_cases(cfg);
dynamic_rows = [
    simulate_boundary_case(cfg, 'positive_0_to_2pi_crossing', ...
        cfg.initial_positive_crossing_rad, cfg.motor_speed_rad_s);
    simulate_boundary_case(cfg, 'negative_2pi_to_0_crossing', ...
        cfg.initial_negative_crossing_rad, -cfg.motor_speed_rad_s)];

case_table = struct2table(case_rows);
dynamic_table = struct2table(dynamic_rows);

case_file = fullfile(results_dir, ...
    'green_joint_speed_estimator_wrap_static_cases.csv');
dynamic_file = fullfile(results_dir, ...
    'green_joint_speed_estimator_wrap_dynamic_crossing.csv');
writetable(case_table, case_file);
writetable(dynamic_table, dynamic_file);

fprintf('\nGreen-joint SpeedEstimator wrap contract test\n');
fprintf('  sample time = %.9g s\n', cfg.sample_time_s);
fprintf('  PLL bw      = %.6g Hz\n', cfg.pll_bandwidth_hz);
fprintf('  static csv  = %s\n', case_file);
fprintf('  dynamic csv = %s\n\n', dynamic_file);
disp(case_table(:, {'case_name', 'theta_meas_rad', ...
    'theta_pred_rad', 'theta_err_rad', 'expected_err_rad', 'pass'}));
disp(dynamic_table(:, {'scenario', 'wrap_crossing_count', ...
    'theta_hat_min_rad', 'theta_hat_max_rad', ...
    'theta_err_min_rad', 'theta_err_max_rad', ...
    'max_abs_speed_error_rad_s', 'pass'}));

if any(~case_table.pass) || any(~dynamic_table.pass)
    error('SpeedEstimator wrap contract test failed.');
end

fprintf('SpeedEstimator wrap contract test passed.\n');

function rows = validate_static_wrap_cases(cfg)
two_pi = 2 * pi;
specs = {
    'meas_after_zero_pred_before_2pi', 0.01, two_pi - 0.01, 0.02;
    'meas_before_2pi_pred_after_zero', two_pi - 0.01, 0.01, -0.02;
    'exact_positive_pi_maps_to_neg_pi', pi, 0.0, -pi;
    'exact_negative_pi_maps_to_neg_pi', 0.0, pi, -pi};

rows = repmat(struct( ...
    'case_name', '', ...
    'theta_meas_rad', 0.0, ...
    'theta_pred_rad', 0.0, ...
    'theta_err_rad', 0.0, ...
    'expected_err_rad', 0.0, ...
    'abs_error_rad', 0.0, ...
    'pass', false), size(specs, 1), 1);

for i = 1:size(specs, 1)
    theta_meas = wrap_0_to_2pi_loop(specs{i, 2});
    theta_pred = wrap_0_to_2pi_loop(specs{i, 3});
    theta_err = wrap_pi_loop(theta_meas - theta_pred);
    expected = specs{i, 4};
    abs_error = abs(theta_err - expected);

    rows(i).case_name = specs{i, 1};
    rows(i).theta_meas_rad = theta_meas;
    rows(i).theta_pred_rad = theta_pred;
    rows(i).theta_err_rad = theta_err;
    rows(i).expected_err_rad = expected;
    rows(i).abs_error_rad = abs_error;
    rows(i).pass = abs_error <= cfg.error_tolerance_rad && ...
        is_angle_0_to_2pi(theta_meas, cfg.angle_tolerance_rad) && ...
        is_angle_0_to_2pi(theta_pred, cfg.angle_tolerance_rad) && ...
        is_angle_neg_pi_to_pi(theta_err, cfg.angle_tolerance_rad);
end
end

function row = simulate_boundary_case(cfg, scenario, initial_theta_rad, omega_true_rad_s)
sample_count = floor(cfg.stop_time_s / cfg.sample_time_s) + 1;
theta_true = initial_theta_rad;
theta_hat = wrap_0_to_2pi_loop(theta_true);
omega_hat = omega_true_rad_s;

theta_meas_log = zeros(sample_count, 1);
theta_pred_log = zeros(sample_count, 1);
theta_hat_log = zeros(sample_count, 1);
theta_err_log = zeros(sample_count, 1);
omega_hat_log = zeros(sample_count, 1);

for k = 1:sample_count
    theta_true = theta_true + omega_true_rad_s * cfg.sample_time_s;
    theta_meas = wrap_0_to_2pi_loop(theta_true);
    theta_pred = wrap_0_to_2pi_loop(theta_hat + ...
        cfg.sample_time_s * omega_hat);
    theta_err = wrap_pi_loop(theta_meas - theta_pred);
    theta_hat = wrap_0_to_2pi_loop(theta_pred + ...
        cfg.sample_time_s * cfg.pll_kp * theta_err);
    omega_hat = omega_hat + cfg.sample_time_s * cfg.pll_ki * theta_err;

    theta_meas_log(k) = theta_meas;
    theta_pred_log(k) = theta_pred;
    theta_hat_log(k) = theta_hat;
    theta_err_log(k) = theta_err;
    omega_hat_log(k) = omega_hat;
end

wrap_crossing_count = sum(abs(diff(theta_meas_log)) > pi);
max_abs_speed_error = max(abs(omega_hat_log - omega_true_rad_s));
max_abs_theta_err = max(abs(theta_err_log));
theta_hat_ok = all(arrayfun(@(x) ...
    is_angle_0_to_2pi(x, cfg.angle_tolerance_rad), theta_hat_log));
theta_pred_ok = all(arrayfun(@(x) ...
    is_angle_0_to_2pi(x, cfg.angle_tolerance_rad), theta_pred_log));
theta_meas_ok = all(arrayfun(@(x) ...
    is_angle_0_to_2pi(x, cfg.angle_tolerance_rad), theta_meas_log));
theta_err_ok = all(arrayfun(@(x) ...
    is_angle_neg_pi_to_pi(x, cfg.angle_tolerance_rad), theta_err_log));

row = struct( ...
    'scenario', scenario, ...
    'omega_true_rad_s', omega_true_rad_s, ...
    'wrap_crossing_count', wrap_crossing_count, ...
    'theta_meas_min_rad', min(theta_meas_log), ...
    'theta_meas_max_rad', max(theta_meas_log), ...
    'theta_pred_min_rad', min(theta_pred_log), ...
    'theta_pred_max_rad', max(theta_pred_log), ...
    'theta_hat_min_rad', min(theta_hat_log), ...
    'theta_hat_max_rad', max(theta_hat_log), ...
    'theta_err_min_rad', min(theta_err_log), ...
    'theta_err_max_rad', max(theta_err_log), ...
    'max_abs_theta_err_rad', max_abs_theta_err, ...
    'max_abs_speed_error_rad_s', max_abs_speed_error, ...
    'theta_meas_range_pass', theta_meas_ok, ...
    'theta_pred_range_pass', theta_pred_ok, ...
    'theta_hat_range_pass', theta_hat_ok, ...
    'theta_err_range_pass', theta_err_ok, ...
    'pass', theta_meas_ok && theta_pred_ok && theta_hat_ok && ...
        theta_err_ok && wrap_crossing_count >= 1 && ...
        max_abs_theta_err <= cfg.error_tolerance_rad && ...
        max_abs_speed_error <= cfg.speed_tolerance_rad_s);
end

function y = wrap_pi_loop(x)
pi_v = pi;
y = wrap_0_to_2pi_loop(x + pi_v) - pi_v;
end

function y = wrap_0_to_2pi_loop(x)
two_pi = 2 * pi;
y = x;
while y >= two_pi
    y = y - two_pi;
end
while y < 0
    y = y + two_pi;
end
end

function ok = is_angle_0_to_2pi(x, tolerance)
ok = x >= -tolerance && x < 2 * pi + tolerance;
end

function ok = is_angle_neg_pi_to_pi(x, tolerance)
ok = x >= -pi - tolerance && x < pi + tolerance;
end
