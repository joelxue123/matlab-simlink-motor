%% Sweep PLL speed-estimator sample interval for green-joint
%
% This is a numeric V1-side study for the estimator candidate. It compares
% 50/100/200/400 us PLL update intervals before firmware replacement.

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
results_dir = fullfile(script_dir, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

cfg.sample_time_s_list = [50e-6 100e-6 200e-6 400e-6];
cfg.pll_bandwidth_hz_list = [120 240 360 480 600];
cfg.damping = 1.0;
cfg.gear_ratio = 183.35;
cfg.encoder_counts = 65536;
cfg.stop_time_s = 0.050;
cfg.speed_step_time_s = 0.005;
cfg.motor_speed_before_rad_s = 0.0;
cfg.motor_speed_after_rad_s = 900.0;
cfg.noise_std_count = 0.0;
cfg.measurement_phase_frequency_hz = 20.0;

rows = struct([]);
for sample_time_s = cfg.sample_time_s_list
    for pll_bandwidth_hz = cfg.pll_bandwidth_hz_list
        result = simulate_pll_case(cfg, sample_time_s, pll_bandwidth_hz);
        rows = [rows; result]; %#ok<AGROW>
    end
end

summary_table = struct2table(rows);
summary_file = fullfile(results_dir, ...
    'green_joint_speed_estimator_pll_interval_sweep.csv');
writetable(summary_table, summary_file);

plot_file = fullfile(results_dir, ...
    'green_joint_speed_estimator_pll_interval_sweep.png');
plot_sweep(summary_table, plot_file);

fprintf('\nGreen-joint PLL speed-estimator interval sweep\n');
fprintf('  speed step      = %.6g -> %.6g motor rad/s\n', ...
    cfg.motor_speed_before_rad_s, cfg.motor_speed_after_rad_s);
fprintf('  measurement f   = %.6g Hz\n', cfg.measurement_phase_frequency_hz);
fprintf('  summary         = %s\n', summary_file);
fprintf('  plot            = %s\n\n', plot_file);
disp(summary_table(:, {'sample_time_us', 'pll_bandwidth_hz', ...
    'rise_time_ms', 'settling_time_ms', 'overshoot_pct', ...
    'phase_at_20hz_deg'}));

function result = simulate_pll_case(cfg, sample_time_s, pll_bandwidth_hz)
t = (0:sample_time_s:cfg.stop_time_s).';
omega_true = cfg.motor_speed_before_rad_s * ones(size(t));
omega_true(t >= cfg.speed_step_time_s) = cfg.motor_speed_after_rad_s;
theta_true = cumsum(omega_true) * sample_time_s;
theta_meas = mod(theta_true, 2 * pi);
if cfg.noise_std_count > 0
    theta_meas = theta_meas + randn(size(theta_meas)) * ...
        (2 * pi / cfg.encoder_counts) * cfg.noise_std_count;
    theta_meas = mod(theta_meas, 2 * pi);
end

omega_n = 2 * pi * pll_bandwidth_hz;
kp = 2 * cfg.damping * omega_n;
ki = omega_n ^ 2;

theta_hat = theta_meas(1);
omega_hat = 0;
omega_est = zeros(size(t));
for k = 1:numel(t)
    theta_pred = wrap_0_2pi(theta_hat + sample_time_s * omega_hat);
    err = wrap_pi(theta_meas(k) - theta_pred);
    theta_hat = wrap_0_2pi(theta_pred + sample_time_s * kp * err);
    omega_hat = omega_hat + sample_time_s * ki * err;
    omega_est(k) = omega_hat;
end

final_ref = cfg.motor_speed_after_rad_s;
post_step = t >= cfg.speed_step_time_s;
rise_time_s = first_crossing_time(t, omega_est, cfg.speed_step_time_s, ...
    0.9 * final_ref);
settling_time_s = settling_time(t, omega_est, final_ref, ...
    cfg.speed_step_time_s, 0.05 * final_ref);
overshoot_pct = max(0, max(omega_est(post_step)) - final_ref) / final_ref * 100;
steady = t >= (cfg.stop_time_s - 0.010);
speed_noise_std_rad_s = std(omega_est(steady) - omega_true(steady));

phase_at_20hz_deg = pll_measurement_phase_deg(sample_time_s, ...
    pll_bandwidth_hz, cfg.damping, cfg.measurement_phase_frequency_hz);

result = struct( ...
    'sample_time_us', sample_time_s * 1e6, ...
    'pll_bandwidth_hz', pll_bandwidth_hz, ...
    'pll_kp', kp, ...
    'pll_ki', ki, ...
    'rise_time_ms', rise_time_s * 1e3, ...
    'settling_time_ms', settling_time_s * 1e3, ...
    'overshoot_pct', overshoot_pct, ...
    'steady_speed_noise_std_rad_s', speed_noise_std_rad_s, ...
    'phase_at_20hz_deg', phase_at_20hz_deg, ...
    'final_motor_speed_rad_s', mean(omega_est(steady)));
end

function phase_deg = pll_measurement_phase_deg(sample_time_s, bandwidth_hz, damping, frequency_hz)
omega_n = 2 * pi * bandwidth_hz;
kp = 2 * damping * omega_n;
ki = omega_n ^ 2;
omega = 2 * pi * frequency_hz;
s = 1j * omega;
continuous_pll = (kp * s + ki) ./ (s .^ 2 + kp * s + ki);
zoh_delay = exp(-s * sample_time_s * 0.5);
phase_deg = angle(continuous_pll .* zoh_delay) * 180 / pi;
end

function y = wrap_pi(x)
y = mod(x + pi, 2 * pi) - pi;
end

function y = wrap_0_2pi(x)
y = mod(x, 2 * pi);
if y < 0
    y = y + 2 * pi;
end
end

function result = first_crossing_time(time, values, start_time, threshold)
idx = find(time >= start_time & values >= threshold, 1);
if isempty(idx)
    result = NaN;
else
    result = time(idx) - start_time;
end
end

function result = settling_time(time, values, final_value, start_time, band)
post = find(time >= start_time);
result = NaN;
for i = post(:).'
    if all(abs(values(i:end) - final_value) <= band)
        result = time(i) - start_time;
        return;
    end
end
end

function plot_sweep(summary_table, plot_file)
figure_handle = figure('Visible', 'off');
hold on;
sample_times = unique(summary_table.sample_time_us);
for i = 1:numel(sample_times)
    rows = summary_table.sample_time_us == sample_times(i);
    plot(summary_table.pll_bandwidth_hz(rows), ...
        summary_table.phase_at_20hz_deg(rows), '-o', ...
        'DisplayName', sprintf('%.0f us', sample_times(i)));
end
grid on;
xlabel('PLL bandwidth (Hz)');
ylabel('Estimator phase at 20 Hz (deg)');
title('green-joint PLL speed-estimator interval sweep');
legend('Location', 'southwest');
saveas(figure_handle, plot_file);
close(figure_handle);
end
