%% Validate that Kp=1, Ki=20000 must not be used as the 1620 current-loop default
%
% Kp=1/Ki=20000 came from 1625 current-loop tuning around 2000 Hz bandwidth.
% This script checks it against the 1620 electrical plant with explicit
% controller timing delay. The goal is not to tune a final hardware value, but
% to prevent the 1620 variant contract from silently inheriting a 1625 value.

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

run(fullfile(script_dir, 'setup_green_joint_current_loop_twin.m'));
GJDT_MotorType = '1620';
GJDT_ModuleConfig = load_green_joint_module_config( ...
    fullfile(fileparts(fileparts(script_dir)), 'green-joint'), GJDT_MotorType);

cfg.ts_ctrl = GJDT_Ts;
cfg.ts_plant = 1e-6;
cfg.stop_time = 0.010;
cfg.vbus = double(GJDT_Vbus_V);
cfg.rs = GJDT_ModuleConfig.phase_resistance_ohm;
cfg.lq = GJDT_ModuleConfig.phase_inductance_h;
cfg.voltage_limit = cfg.vbus ...
    * GJDT_ModuleConfig.current_loop.voltage_limit_ratio ...
    * GJDT_ModuleConfig.current_loop.voltage_modulation_ratio;
cfg.pi_correction_gain = GJDT_ModuleConfig.current_loop.pi_correction_gain;
cfg.square_period = 0.001;
cfg.square_amplitude = 0.3;
cfg.step_final = 1.5;
cfg.step_time = 0.001;
cfg.measure_start = 0.002;
cfg.feedback_alpha = 0.95;

wc_800 = 2 * pi * 800;
wc_2000 = 2 * pi * 2000;
cases = [ ...
    struct('name', "1625_borrowed_kp1_ki20000", ...
        'kp', 1.0, 'ki', 20000.0, ...
        'note', "BANNED for 1620 default: 1625-derived tuning"); ...
    struct('name', "1620_physical_800hz", ...
        'kp', cfg.lq * wc_800, 'ki', cfg.rs * wc_800, ...
        'note', "Recommended 1620 safe contract candidate"); ...
    struct('name', "1620_physical_2000hz", ...
        'kp', cfg.lq * wc_2000, 'ki', cfg.rs * wc_2000, ...
        'note', "Aggressive 1620 candidate, delay-sensitive")];

delay_ticks = [0 1 2];
rows = [];
for i = 1:numel(cases)
    for j = 1:numel(delay_ticks)
        square_result = simulate_square(cfg, cases(i), delay_ticks(j));
        step_result = simulate_step(cfg, cases(i), delay_ticks(j));
        rows = [rows; { ...
            char(cases(i).name), delay_ticks(j), cases(i).kp, cases(i).ki, ...
            square_result.stable, square_result.rmse_a, ...
            square_result.max_abs_error_a, square_result.iq_peak_pos_a, ...
            square_result.iq_peak_neg_a, square_result.voltage_norm_max, ...
            step_result.stable, step_result.overshoot_ratio, ...
            step_result.settling_time_ms, step_result.final_iq_a, ...
            char(cases(i).note)}]; %#ok<AGROW>
    end
end

summary = cell2table(rows, 'VariableNames', { ...
    'case_name', 'feedback_delay_ticks', 'kp', 'ki', ...
    'square_stable', 'square_rmse_a', 'square_max_abs_error_a', ...
    'square_iq_peak_pos_a', 'square_iq_peak_neg_a', ...
    'square_voltage_norm_max', 'step_stable', 'step_overshoot_ratio', ...
    'step_settling_time_ms', 'step_final_iq_a', 'note'});

csv_file = fullfile(script_dir, 'results', ...
    'green_joint_1620_current_loop_kp1_ki20000_validation.csv');
if ~exist(fileparts(csv_file), 'dir')
    mkdir(fileparts(csv_file));
end
writetable(summary, csv_file);

plot_file = fullfile(script_dir, 'results', ...
    'green_joint_1620_current_loop_kp1_ki20000_validation.png');
plot_square_comparison(cfg, cases, plot_file);

borrowed_delay1 = summary(strcmp(summary.case_name, ...
    '1625_borrowed_kp1_ki20000') & summary.feedback_delay_ticks == 1, :);
if borrowed_delay1.square_stable || borrowed_delay1.step_stable
    error('Expected Kp=1/Ki=20000 to fail the 1620 delay-sensitive validation.');
end

safe_delay1 = summary(strcmp(summary.case_name, ...
    '1620_physical_800hz') & summary.feedback_delay_ticks == 1, :);
if ~safe_delay1.square_stable || ~safe_delay1.step_stable
    error('Expected 1620 800 Hz physical candidate to pass with 1 tick delay.');
end

fprintf('\nGreen-joint 1620 current-loop validation:\n');
fprintf('  phase R/L       = %.9g ohm / %.9g H\n', cfg.rs, cfg.lq);
fprintf('  Vbus / Vlimit   = %.9g V / %.9g V\n', cfg.vbus, cfg.voltage_limit);
fprintf('  Ts ctrl/plant   = %.9g us / %.9g us\n', ...
    cfg.ts_ctrl * 1e6, cfg.ts_plant * 1e6);
fprintf('  conclusion      = Kp=1, Ki=20000 is 1625-derived and banned for 1620 default\n');
fprintf('  recommended     = Kp %.9g, Ki %.9g (1620 800 Hz physical candidate)\n', ...
    cfg.lq * wc_800, cfg.rs * wc_800);
fprintf('  csv             = %s\n', csv_file);
fprintf('  plot            = %s\n', plot_file);
disp(summary(:, {'case_name', 'feedback_delay_ticks', 'kp', 'ki', ...
    'square_stable', 'square_rmse_a', 'square_iq_peak_pos_a', ...
    'square_iq_peak_neg_a', 'square_voltage_norm_max', ...
    'step_stable', 'step_overshoot_ratio', 'step_settling_time_ms'}));

function result = simulate_square(cfg, case_cfg, feedback_delay_ticks)
t = (0:cfg.ts_plant:cfg.stop_time)';
ref = square_ref(t, cfg.square_period, cfg.square_amplitude);
[iq, vq] = simulate_current_loop(cfg, case_cfg.kp, case_cfg.ki, ref, ...
    feedback_delay_ticks);

measure = t >= cfg.measure_start;
err = ref(measure) - iq(measure);
result.rmse_a = sqrt(mean(err.^2));
result.max_abs_error_a = max(abs(err));
result.iq_peak_pos_a = max(iq(measure));
result.iq_peak_neg_a = min(iq(measure));
result.voltage_norm_max = max(abs(vq)) / cfg.voltage_limit;
result.stable = result.voltage_norm_max < 0.95 ...
    && result.iq_peak_pos_a < 1.0 ...
    && result.iq_peak_neg_a > -1.0;
end

function result = simulate_step(cfg, case_cfg, feedback_delay_ticks)
t = (0:cfg.ts_plant:0.006)';
ref = zeros(size(t));
ref(t >= cfg.step_time) = cfg.step_final;
[iq, vq] = simulate_current_loop(cfg, case_cfg.kp, case_cfg.ki, ref, ...
    feedback_delay_ticks);

post = t >= cfg.step_time;
result.overshoot_ratio = (max(iq(post)) - cfg.step_final) / cfg.step_final;
result.final_iq_a = iq(end);
result.voltage_norm_max = max(abs(vq)) / cfg.voltage_limit;
result.settling_time_ms = first_settle_time_ms(t, iq, cfg.step_final, ...
    cfg.step_time, 0.02 * cfg.step_final);
result.stable = result.voltage_norm_max < 0.95 ...
    && result.overshoot_ratio < 0.5 ...
    && ~isnan(result.settling_time_ms) ...
    && abs(result.final_iq_a - cfg.step_final) < 0.05;
end

function [iq_fbk, vq] = simulate_current_loop(cfg, kp, ki, ref, feedback_delay_ticks)
t = (0:cfg.ts_plant:(numel(ref) - 1) * cfg.ts_plant)';
iq = zeros(size(t));
iq_fbk = zeros(size(t));
vq = zeros(size(t));
integrator = 0.0;
last_vq = 0.0;

ctrl_steps = round(cfg.ts_ctrl / cfg.ts_plant);
feedback_delay_samples = feedback_delay_ticks * ctrl_steps;
for k = 2:numel(t)
    if mod(k - 2, ctrl_steps) == 0
        feedback_index = max(1, k - 1 - feedback_delay_samples);
        err = ref(k - 1) - iq_fbk(feedback_index);
        pre_sat = kp * err + integrator;
        cmd = min(max(pre_sat, -cfg.voltage_limit), cfg.voltage_limit);
        integrator = integrator + ...
            (ki * err + cfg.pi_correction_gain * (cmd - pre_sat)) ...
            * cfg.ts_ctrl;
        last_vq = cmd;
    end

    vq(k) = last_vq;
    iq(k) = iq(k - 1) ...
        + cfg.ts_plant * (vq(k) - cfg.rs * iq(k - 1)) / cfg.lq;
    iq_fbk(k) = iq_fbk(k - 1) ...
        + cfg.feedback_alpha * (iq(k) - iq_fbk(k - 1));
end
end

function ref = square_ref(t, period, amplitude)
phase = mod(t, period);
ref = -amplitude * ones(size(t));
ref(phase >= period / 2) = amplitude;
end

function settle_ms = first_settle_time_ms(t, y, target, step_time, band)
settle_ms = NaN;
start_index = find(t >= step_time, 1, 'first');
for i = start_index:numel(t)
    if all(abs(y(i:end) - target) <= band)
        settle_ms = (t(i) - step_time) * 1e3;
        return;
    end
end
end

function plot_square_comparison(cfg, cases, plot_file)
figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1200 760]);
t = (0:cfg.ts_plant:cfg.stop_time)';
ref = square_ref(t, cfg.square_period, cfg.square_amplitude);
plot(t * 1e3, ref, 'k-', 'LineWidth', 1.0);
hold on;
for i = 1:numel(cases)
    [iq, ~] = simulate_current_loop(cfg, cases(i).kp, cases(i).ki, ref, 1);
    plot(t * 1e3, iq, 'LineWidth', 1.2);
end
grid on;
xlabel('Time (ms)');
ylabel('Iq feedback (A)');
title('1620 current-loop validation, 1 tick feedback delay, 1 kHz square');
legend(['Iq ref', cellstr(string({cases.name}))], ...
    'Interpreter', 'none', 'Location', 'best');
xlim([2 cfg.stop_time * 1e3]);
exportgraphics(gcf, plot_file, 'Resolution', 160);
close(gcf);
end

function cfg = load_green_joint_module_config(fw_dir, motor_type)
config_file = fullfile(fw_dir, 'Module', 'Config', ...
    ['green_joint_' char(motor_type) '_config.json']);
if ~exist(config_file, 'file')
    error('Missing green-joint module config: %s', config_file);
end
cfg = jsondecode(fileread(config_file));
end
