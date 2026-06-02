function result = run_cogging_torque_comp_study(cogging)
% Study cogging-torque compensation using the existing angle-synchronous FF pipeline.
%
% Optional input struct fields:
%   harmonic1, harmonic2, amp1, amp2, phase1_deg, phase2_deg,
%   table_points, phase_advance_deg, ff_output_limit, ff_enable_time,
%   learn_start_time, test_stop_time, speed_ref_scale.

if nargin < 1
    cogging = struct();
end

motor_control_params;
[cogging, motor] = local_fill_defaults(cogging, motor, control);
control = local_apply_cogging_settings(control, motor, cogging);

assignin('base', 'motor', motor);
assignin('base', 'inverter', inverter);
assignin('base', 'simcfg', simcfg);

baseline_control = control;
baseline_control.vib.mode = 'none';
baseline_control.vib.enable_learning = 0;
baseline_control.vib.enable_ff = 0;
assignin('base', 'control', baseline_control);

build_vibration_comp_test;
set_param('vibration_comp_test', 'InitFcn', '');
out_base = sim('vibration_comp_test', 'ReturnWorkspaceOutputs', 'on');

table_info = local_learn_ff_table_from_logs(out_base, baseline_control);
control.vib.ff_table = table_info.ff_table;
assignin('base', 'control', control);

control.vib.mode = 'offline';
control.vib.enable_learning = 0;
control.vib.enable_ff = 1;
assignin('base', 'control', control);

build_vibration_comp_test;
set_param('vibration_comp_test', 'InitFcn', '');
out_comp = sim('vibration_comp_test', 'ReturnWorkspaceOutputs', 'on');

metrics = local_evaluate_results(out_base, out_comp, control, table_info);
local_plot_results(out_base, out_comp, control, metrics);

result = struct();
result.config = cogging;
result.table_info = table_info;
result.metrics = metrics;
assignin('base', 'cogging_comp_result', result);
end

function [cogging, motor] = local_fill_defaults(cogging, motor, control)
defaults = struct();
load_defaults = cogging_load_config(cogging);
defaults.load_base_torque = load_defaults.load_base_torque;
defaults.harmonic1 = load_defaults.harmonic1;
defaults.harmonic2 = load_defaults.harmonic2;
defaults.amp1 = load_defaults.amp1;
defaults.amp2 = load_defaults.amp2;
defaults.phase1_deg = load_defaults.phase1_deg;
defaults.phase2_deg = load_defaults.phase2_deg;
defaults.table_points = 180;
defaults.phase_advance_deg = 0;
defaults.ff_output_limit = 0.25 * control.iq_ref_limit;
defaults.ff_enable_time = 0.55;
defaults.learn_start_time = 0.15;
defaults.test_stop_time = 1.0;
defaults.speed_ref_scale = 1.0;

names = fieldnames(defaults);
for idx = 1:numel(names)
    name = names{idx};
    if ~isfield(cogging, name) || isempty(cogging.(name))
        cogging.(name) = defaults.(name);
    end
end

if cogging.speed_ref_scale <= 0
    cogging.speed_ref_scale = 1.0;
end

motor.speed_ref_mech_rad_s = motor.speed_ref_mech_rad_s * cogging.speed_ref_scale;
assignin('base', 'motor', motor);
end

function control = local_apply_cogging_settings(control, motor, cogging)
control.vib.mode = 'none';
control.vib.enable_learning = 0;
control.vib.enable_ff = 0;
control.vib.table_points = min(360, max(24, round(cogging.table_points)));
control.vib.phase_advance_deg = cogging.phase_advance_deg;
control.vib.output_limit = max(0, cogging.ff_output_limit);
control.vib.learn_start_time = max(0, cogging.learn_start_time);
control.vib.ff_enable_time = max(control.vib.learn_start_time + 0.20, cogging.ff_enable_time);
control.vib.test_stop_time = max(control.vib.ff_enable_time + 0.20, cogging.test_stop_time);
control.vib.min_speed_abs = 0.80 * motor.speed_ref_mech_rad_s;
control.vib.speed_err_threshold = 6.0;
control.vib.ff_table = zeros(360, 1);
control.vib.ff_table_file = 'cogging_ff_table.mat';

control = apply_cogging_load_config(control, cogging);
end

function table_info = local_learn_ff_table_from_logs(sim_out, control)
theta_ts = sim_out.get('log_vib_theta');
iqbase_ts = sim_out.get('log_vib_iqbase');
wm_ts = sim_out.get('log_vib_wm');

window_start = max(control.vib.learn_start_time + 0.25, 0.40);
window_end = control.vib.test_stop_time;
mask = theta_ts.Time(:) >= window_start & theta_ts.Time(:) <= window_end;

theta = theta_ts.Data(mask);
iqbase = interp1(iqbase_ts.Time(:), iqbase_ts.Data(:), theta_ts.Time(mask), 'linear', 'extrap');
wm = interp1(wm_ts.Time(:), wm_ts.Data(:), theta_ts.Time(mask), 'linear', 'extrap');
if isempty(theta) || isempty(iqbase)
    error('No data in offline learning window.');
end

points = control.vib.table_points;
bin_edges = linspace(0, 2 * pi, points + 1);
theta_wrap = mod(theta(:), 2 * pi);
iqbase = iqbase(:);
learn_signal = iqbase - mean(iqbase);

active_table = zeros(points, 1);
counts = zeros(points, 1);
for idx = 1:points
    if idx < points
        in_bin = theta_wrap >= bin_edges(idx) & theta_wrap < bin_edges(idx + 1);
    else
        in_bin = theta_wrap >= bin_edges(idx) & theta_wrap <= bin_edges(idx + 1);
    end
    if any(in_bin)
        active_table(idx) = mean(learn_signal(in_bin));
        counts(idx) = sum(in_bin);
    end
end

valid = counts > 0;
if nnz(valid) < 2
    error('Insufficient angular coverage to build offline FF table.');
end

bin_centers = (bin_edges(1:end-1) + bin_edges(2:end)) / 2;
valid_x = bin_centers(valid).';
valid_y = active_table(valid);
valid_x_ext = [valid_x(end) - 2 * pi; valid_x; valid_x(1) + 2 * pi];
valid_y_ext = [valid_y(end); valid_y; valid_y(1)];
active_table = interp1(valid_x_ext, valid_y_ext, bin_centers(:), 'linear');

ff_table = zeros(360, 1);
ff_table(1:points) = active_table(:);
save(control.vib.ff_table_file, 'ff_table', 'points', 'window_start', 'window_end');

table_info = struct();
table_info.points = points;
table_info.window_start = window_start;
table_info.window_end = window_end;
table_info.ff_table_file = control.vib.ff_table_file;
table_info.ff_table = ff_table;
table_info.learn_signal_rms = rms(learn_signal);
table_info.speed_mean = mean(wm);
end

function metrics = local_evaluate_results(out_base, out_comp, control, table_info)
wm_base = out_base.get('log_vib_wm');
wm_comp = out_comp.get('log_vib_wm');
iqff_comp = out_comp.get('log_vib_iqff');
learn_comp = out_comp.get('log_vib_learn');

window_start = max([control.vib.learn_start_time + 0.25, control.vib.ff_enable_time + 0.10, 0.40]);
window_end = control.vib.test_stop_time;

[base_std, base_pp] = local_ripple_metrics(wm_base, window_start, window_end);
[comp_std, comp_pp] = local_ripple_metrics(wm_comp, window_start, window_end);

metrics = struct();
metrics.window_start = window_start;
metrics.window_end = window_end;
metrics.base_ripple_std = base_std;
metrics.base_ripple_pp = base_pp;
metrics.comp_ripple_std = comp_std;
metrics.comp_ripple_pp = comp_pp;
metrics.std_reduction_pct = 100 * (base_std - comp_std) / max(base_std, eps);
metrics.pp_reduction_pct = 100 * (base_pp - comp_pp) / max(base_pp, eps);
metrics.iqff_rms = local_rms_window(iqff_comp, window_start, window_end);
metrics.learn_active_mean = local_mean_window(learn_comp, window_start, window_end);
metrics.ff_table_file = table_info.ff_table_file;

fprintf('\n=== Cogging torque compensation study ===\n');
fprintf('Cogging harmonic 1   : %g\n', control.vib.load_harmonic1);
fprintf('Cogging harmonic 2   : %g\n', control.vib.load_harmonic2);
fprintf('Lookup table points  : %d\n', control.vib.table_points);
fprintf('Window               : [%.3f, %.3f] s\n', window_start, window_end);
fprintf('Baseline ripple std  : %.6f rad/s\n', metrics.base_ripple_std);
fprintf('Comp ripple std      : %.6f rad/s\n', metrics.comp_ripple_std);
fprintf('Std reduction        : %.2f %%\n', metrics.std_reduction_pct);
fprintf('Baseline ripple p-p  : %.6f rad/s\n', metrics.base_ripple_pp);
fprintf('Comp ripple p-p      : %.6f rad/s\n', metrics.comp_ripple_pp);
fprintf('P-P reduction        : %.2f %%\n', metrics.pp_reduction_pct);
fprintf('Iq_ff RMS            : %.6f A\n', metrics.iqff_rms);
fprintf('Saved table file     : %s\n', metrics.ff_table_file);
end

function local_plot_results(out_base, out_comp, control, metrics)
wref_base = out_base.get('log_vib_wref');
wm_base = out_base.get('log_vib_wm');
tload_base = out_base.get('log_vib_tload');

wref_comp = out_comp.get('log_vib_wref');
wm_comp = out_comp.get('log_vib_wm');
iqff_comp = out_comp.get('log_vib_iqff');

figure('Name', 'Cogging Torque Compensation Comparison', 'Color', 'w');

subplot(3,1,1);
plot(wref_base.Time, wref_base.Data, 'k--', 'LineWidth', 1.0); hold on;
plot(wm_base.Time, wm_base.Data, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.0);
plot(wm_comp.Time, wm_comp.Data, 'Color', [0 0.45 0.74], 'LineWidth', 1.0);
xline(metrics.window_start, ':', 'Window Start');
xline(metrics.window_end, ':', 'Window End');
xline(control.vib.ff_enable_time, '--', 'FF On');
grid on;
legend('w_{ref}', 'w_{meas} baseline', 'w_{meas} offline FF', 'Location', 'best');
title('Speed Comparison');
ylabel('rad/s');

subplot(3,1,2);
plot(tload_base.Time, tload_base.Data, 'Color', [0.49 0.18 0.56], 'LineWidth', 1.0); hold on;
plot(iqff_comp.Time, iqff_comp.Data, 'Color', [0.47 0.67 0.19], 'LineWidth', 1.0);
xline(metrics.window_start, ':', 'Window Start');
xline(metrics.window_end, ':', 'Window End');
xline(control.vib.ff_enable_time, '--', 'FF On');
grid on;
legend('T_{cogging}', 'Iq_{ff}', 'Location', 'best');
title('Cogging Disturbance and Feedforward');
ylabel('N*m / A');

subplot(3,1,3);
wm_base_i = interp1(wm_base.Time(:), wm_base.Data(:), wm_comp.Time(:), 'linear', 'extrap');
plot(wm_comp.Time, wm_base_i - mean(wm_base_i), 'Color', [0.85 0.33 0.10], 'LineWidth', 1.0); hold on;
plot(wm_comp.Time, wm_comp.Data - mean(wm_comp.Data), 'Color', [0 0.45 0.74], 'LineWidth', 1.0);
xline(metrics.window_start, ':', 'Window Start');
xline(metrics.window_end, ':', 'Window End');
xline(control.vib.ff_enable_time, '--', 'FF On');
grid on;
legend('Baseline ripple', 'Offline FF ripple', 'Location', 'best');
title('Centered Speed Ripple');
xlabel('Time (s)');
ylabel('rad/s');

assignin('base', 'cogging_metrics', metrics);
assignin('base', 'cogging_plot_wref', wref_comp);
end

function [ripple_std, ripple_pp] = local_ripple_metrics(ts, t0, t1)
mask = ts.Time >= t0 & ts.Time <= t1;
y = ts.Data(mask);
if isempty(y)
    ripple_std = NaN;
    ripple_pp = NaN;
    return;
end
y = y(:) - mean(y(:));
ripple_std = std(y);
ripple_pp = max(y) - min(y);
end

function y_rms = local_rms_window(ts, t0, t1)
mask = ts.Time >= t0 & ts.Time <= t1;
y = ts.Data(mask);
if isempty(y)
    y_rms = NaN;
else
    y_rms = rms(y(:));
end
end

function y_mean = local_mean_window(ts, t0, t1)
mask = ts.Time >= t0 & ts.Time <= t1;
y = ts.Data(mask);
if isempty(y)
    y_mean = NaN;
else
    y_mean = mean(y(:));
end
end