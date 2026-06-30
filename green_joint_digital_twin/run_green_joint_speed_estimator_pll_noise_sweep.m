%% Sweep PLL speed-estimator noise sensitivity for green-joint
%
% This numeric study complements the V1 closed-loop step test. The hardware
% feedback showed the 600 Hz PLL candidate is noisy, so this script compares
% steady-state speed noise caused by encoder angle quantization/jitter.

clear;
clc;
rng(1);

script_dir = fileparts(mfilename('fullpath'));
results_dir = fullfile(script_dir, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

cfg.sample_time_s = 50e-6;
cfg.pll_bandwidth_hz_list = [120 240 360 480 600 720];
cfg.damping = 1.0;
cfg.gear_ratio = 183.35;
cfg.encoder_counts = 65536;
cfg.stop_time_s = 0.120;
cfg.settle_time_s = 0.030;
cfg.motor_speed_rad_s = 400.0;
cfg.noise_std_count_list = [0 0.25 0.5 1.0 2.0];

rows = struct([]);
for noise_std_count = cfg.noise_std_count_list
    for pll_bandwidth_hz = cfg.pll_bandwidth_hz_list
        result = simulate_pll_noise_case(cfg, pll_bandwidth_hz, noise_std_count);
        rows = [rows; result]; %#ok<AGROW>
    end
end

summary_table = struct2table(rows);
summary_file = fullfile(results_dir, ...
    'green_joint_speed_estimator_pll_noise_sweep.csv');
writetable(summary_table, summary_file);

plot_file = fullfile(results_dir, ...
    'green_joint_speed_estimator_pll_noise_sweep.png');
plot_noise_sweep(summary_table, plot_file);

fprintf('\nGreen-joint PLL speed-estimator noise sweep\n');
fprintf('  motor speed     = %.6g rad/s\n', cfg.motor_speed_rad_s);
fprintf('  sample time     = %.6g us\n', cfg.sample_time_s * 1e6);
fprintf('  summary         = %s\n', summary_file);
fprintf('  plot            = %s\n\n', plot_file);
disp(summary_table(:, {'pll_bandwidth_hz', 'noise_std_count', ...
    'motor_speed_noise_std_rad_s', 'joint_speed_noise_std_rad_s', ...
    'noise_ratio_vs_600hz'}));

function result = simulate_pll_noise_case(cfg, pll_bandwidth_hz, noise_std_count)
t = (0:cfg.sample_time_s:cfg.stop_time_s).';
theta_true = cfg.motor_speed_rad_s * t;
theta_meas = mod(theta_true, 2 * pi);

if noise_std_count > 0
    theta_noise = randn(size(theta_meas)) * ...
        (2 * pi / cfg.encoder_counts) * noise_std_count;
    theta_meas = mod(theta_meas + theta_noise, 2 * pi);
end

omega_n = 2 * pi * pll_bandwidth_hz;
kp = 2 * cfg.damping * omega_n;
ki = omega_n ^ 2;

theta_hat = theta_meas(1);
omega_hat = cfg.motor_speed_rad_s;
omega_est = zeros(size(t));

for k = 1:numel(t)
    theta_pred = wrap_0_2pi(theta_hat + cfg.sample_time_s * omega_hat);
    err = wrap_pi(theta_meas(k) - theta_pred);
    theta_hat = wrap_0_2pi(theta_pred + cfg.sample_time_s * kp * err);
    omega_hat = omega_hat + cfg.sample_time_s * ki * err;
    omega_est(k) = omega_hat;
end

steady = t >= cfg.settle_time_s;
motor_noise = std(omega_est(steady) - cfg.motor_speed_rad_s);
joint_noise = motor_noise / cfg.gear_ratio;
noise_ratio_vs_600hz = (pll_bandwidth_hz / 600.0) ^ 2;

result = struct( ...
    'sample_time_us', cfg.sample_time_s * 1e6, ...
    'pll_bandwidth_hz', pll_bandwidth_hz, ...
    'pll_kp', kp, ...
    'pll_ki', ki, ...
    'noise_std_count', noise_std_count, ...
    'motor_speed_noise_std_rad_s', motor_noise, ...
    'joint_speed_noise_std_rad_s', joint_noise, ...
    'noise_ratio_vs_600hz', noise_ratio_vs_600hz);
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

function plot_noise_sweep(summary_table, plot_file)
figure_handle = figure('Visible', 'off');
hold on;
noise_levels = unique(summary_table.noise_std_count);
for i = 1:numel(noise_levels)
    rows = summary_table.noise_std_count == noise_levels(i);
    plot(summary_table.pll_bandwidth_hz(rows), ...
        summary_table.motor_speed_noise_std_rad_s(rows), '-o', ...
        'DisplayName', sprintf('%.2g count std', noise_levels(i)));
end
grid on;
xlabel('PLL bandwidth (Hz)');
ylabel('Motor speed noise std (rad/s)');
title('green-joint PLL speed-estimator noise sweep');
legend('Location', 'northwest');
saveas(figure_handle, plot_file);
close(figure_handle);
end
