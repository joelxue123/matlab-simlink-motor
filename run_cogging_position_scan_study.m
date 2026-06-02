function result = run_cogging_position_scan_study(cfg)
% Build a cogging feedforward table from position-control scanning and validate it.

if nargin < 1
    cfg = struct();
end

motor_control_params;
cfg = local_fill_defaults(cfg, motor, control);

forward = local_run_scan_pass(cfg.theta_forward, cfg, motor, inverter, control, simcfg, 'forward');
backward = local_run_scan_pass(cfg.theta_backward, cfg, motor, inverter, control, simcfg, 'backward');

[scan_result, ff_table] = local_build_ff_table(forward, backward, cfg);
save(cfg.ff_table_file, 'ff_table');

validation = local_validate_table(ff_table, cfg, motor, inverter, control, simcfg);
local_plot_scan_results(scan_result, validation, cfg);

result = struct();
result.config = cfg;
result.forward = forward;
result.backward = backward;
result.scan = scan_result;
result.validation = validation;
result.ff_table = ff_table;
result.ff_table_file = cfg.ff_table_file;

assignin('base', 'cogging_scan_result', result);
assignin('base', 'cogging_scan_ff_table', ff_table);

fprintf('\n=== Cogging position scan study ===\n');
fprintf('Scan points          : %d\n', cfg.scan_points);
fprintf('Hold time            : %.4f s\n', cfg.hold_time);
fprintf('Settle time          : %.4f s\n', cfg.settle_time);
fprintf('Average time         : %.4f s\n', cfg.avg_time);
fprintf('FF table file        : %s\n', cfg.ff_table_file);
fprintf('Valid forward points : %d / %d\n', nnz(forward.valid_mask), cfg.scan_points);
fprintf('Valid backward points: %d / %d\n', nnz(backward.valid_mask), cfg.scan_points);
fprintf('Validation std red.  : %.2f %%\n', validation.metrics.std_reduction_pct);
fprintf('Validation p-p red.  : %.2f %%\n', validation.metrics.pp_reduction_pct);
end

function cfg = local_fill_defaults(cfg, motor, control)
defaults = struct();
defaults.scan_points = 180;
defaults.start_time = 0.05;
defaults.hold_time = 0.05;
defaults.settle_time = 0.03;
defaults.avg_time = 0.015;
defaults.position_bandwidth_scale = 0.30;
defaults.position_output_limit_scale = 0.25;
defaults.pid_pos_Ki = 0;
defaults.plot_results = true;
defaults.ff_table_file = 'cogging_scan_ff_table.mat';
defaults.use_iq_meas = false;
defaults.steady_speed_tol = 1.5;
defaults.position_err_tol = deg2rad(0.50);
defaults.iq_std_tol = 0.15;
load_defaults = cogging_load_config(cfg);
defaults.load_base_torque = load_defaults.load_base_torque;
defaults.harmonic1 = load_defaults.harmonic1;
defaults.harmonic2 = load_defaults.harmonic2;
defaults.amp1 = load_defaults.amp1;
defaults.amp2 = load_defaults.amp2;
defaults.phase1_deg = load_defaults.phase1_deg;
defaults.phase2_deg = load_defaults.phase2_deg;
defaults.ff_output_limit = 0.25 * control.iq_ref_limit;
defaults.ff_enable_time = 0.55;
defaults.learn_start_time = 0.15;
defaults.test_stop_time = 1.0;

names = fieldnames(defaults);
for idx = 1:numel(names)
    name = names{idx};
    if ~isfield(cfg, name) || isempty(cfg.(name))
        cfg.(name) = defaults.(name);
    end
end

cfg.scan_points = min(360, max(24, round(cfg.scan_points)));
cfg.hold_time = max(cfg.hold_time, 5 * motor.J);
cfg.avg_time = min(max(cfg.avg_time, 0.002), 0.75 * cfg.hold_time);
cfg.settle_time = min(max(cfg.settle_time, 0), cfg.hold_time - cfg.avg_time);

theta_grid = linspace(0, 2 * pi, cfg.scan_points + 1);
theta_grid(end) = [];
cfg.theta_forward = theta_grid(:);
cfg.theta_backward = flipud(theta_grid(:));
cfg.scan_stop_time = cfg.start_time + cfg.scan_points * cfg.hold_time + 0.02;
cfg.position_bandwidth_hz = control.pos_bandwidth_hz * cfg.position_bandwidth_scale;
cfg.position_output_limit = motor.speed_ref_mech_rad_s * cfg.position_output_limit_scale;
end

function pass = local_run_scan_pass(theta_table, cfg, motor, inverter, control, simcfg, label)
control.pos_ref_mode = 'scan_table';
control.pos_use_planner = false;
control.pos_scan.start_time = cfg.start_time;
control.pos_scan.hold_time = cfg.hold_time;
control.pos_scan.points = cfg.scan_points;
control.pos_scan.theta_table = zeros(360, 1);
control.pos_scan.theta_table(1:cfg.scan_points) = theta_table(:);

control.pos_bandwidth_hz = cfg.position_bandwidth_hz;
control.pos_bandwidth_rad_s = 2 * pi * control.pos_bandwidth_hz;
control.pi_pos.Kp = control.pos_bandwidth_rad_s;
control.pi_pos.output_limit = cfg.position_output_limit;
control.pid_pos.Kp = 2 * control.pid_pos.damping * control.pos_bandwidth_rad_s;
control.pid_pos.Ki_cont = cfg.pid_pos_Ki;
control.pid_pos.Ki = cfg.pid_pos_Ki;
control.pid_pos.output_limit = cfg.position_output_limit;

control.use_periodic_load = true;
control = apply_cogging_load_config(control, cfg);

simcfg.stop_time = cfg.scan_stop_time;

assignin('base', 'motor', motor);
assignin('base', 'inverter', inverter);
assignin('base', 'control', control);
assignin('base', 'simcfg', simcfg);

build_average_inverter_foc_model;
set_param('average_inverter_foc', 'InitFcn', '');
sim_out = sim('average_inverter_foc', 'ReturnWorkspaceOutputs', 'on');
if bdIsLoaded('average_inverter_foc')
    close_system('average_inverter_foc', 0);
end

pass = local_extract_scan_pass(sim_out, theta_table, cfg, label);
end

function pass = local_extract_scan_pass(sim_out, theta_table, cfg, label)
pos_ref_ts = sim_out.get('log_pos_ref');
pos_ts = sim_out.get('log_pos');
wkf_ts = sim_out.get('log_wkf');
wref_ts = sim_out.get('log_wref');
iqref_ts = sim_out.get('log_iq_ref');
iqmeas_ts = sim_out.get('log_iq_meas');

t_ref = pos_ref_ts.Time(:);
pos_ref = pos_ref_ts.Data(:);
pos_meas = interp1(pos_ts.Time(:), pos_ts.Data(:), t_ref, 'linear', 'extrap');
w_kf = interp1(wkf_ts.Time(:), wkf_ts.Data(:), t_ref, 'linear', 'extrap');
w_ref = interp1(wref_ts.Time(:), wref_ts.Data(:), t_ref, 'linear', 'extrap');
iq_ref = interp1(iqref_ts.Time(:), iqref_ts.Data(:), t_ref, 'linear', 'extrap');
iq_meas = interp1(iqmeas_ts.Time(:), iqmeas_ts.Data(:), t_ref, 'linear', 'extrap');

theta_samples = zeros(cfg.scan_points, 1);
iq_ref_samples = zeros(cfg.scan_points, 1);
iq_meas_samples = zeros(cfg.scan_points, 1);
w_samples = zeros(cfg.scan_points, 1);
pos_err_samples = zeros(cfg.scan_points, 1);
iq_std_samples = zeros(cfg.scan_points, 1);
valid_mask = false(cfg.scan_points, 1);

for idx = 1:cfg.scan_points
    t0 = cfg.start_time + (idx - 1) * cfg.hold_time;
    t2 = t0 + cfg.hold_time;
    t1 = max(t0 + cfg.settle_time, t2 - cfg.avg_time);
    mask = t_ref >= t1 & t_ref <= t2;
    if ~any(mask)
        continue;
    end

    theta_samples(idx) = mean(pos_meas(mask));
    iq_ref_samples(idx) = mean(iq_ref(mask));
    iq_meas_samples(idx) = mean(iq_meas(mask));
    w_samples(idx) = mean(abs(w_kf(mask)));
    pos_err_samples(idx) = mean(abs(pos_ref(mask) - pos_meas(mask)));
    iq_std_samples(idx) = std(iq_ref(mask));

    valid_mask(idx) = w_samples(idx) <= cfg.steady_speed_tol && ...
        pos_err_samples(idx) <= cfg.position_err_tol && ...
        iq_std_samples(idx) <= cfg.iq_std_tol;
end

pass = struct();
pass.label = label;
pass.theta_command = theta_table(:);
pass.theta_samples = theta_samples;
pass.iq_ref_samples = iq_ref_samples;
pass.iq_meas_samples = iq_meas_samples;
pass.iq_used = iq_ref_samples;
if cfg.use_iq_meas
    pass.iq_used = iq_meas_samples;
end
pass.w_samples = w_samples;
pass.pos_err_samples = pos_err_samples;
pass.iq_std_samples = iq_std_samples;
pass.valid_mask = valid_mask;
pass.time = t_ref;
pass.pos_ref = pos_ref;
pass.pos_meas = pos_meas;
pass.w_ref = w_ref;
pass.w_kf = w_kf;
pass.iq_ref = iq_ref;
pass.iq_meas = iq_meas;
end

function [scan_result, ff_table] = local_build_ff_table(forward, backward, cfg)
theta_forward = mod(forward.theta_command(:), 2 * pi);
theta_backward = mod(backward.theta_command(:), 2 * pi);
iq_forward = forward.iq_used(:);
iq_backward = backward.iq_used(:);

valid_forward = forward.valid_mask(:);
valid_backward = backward.valid_mask(:);
iq_backward_aligned = zeros(size(iq_forward));
valid_backward_aligned = false(size(valid_forward));

for idx = 1:numel(theta_forward)
    [~, back_idx] = min(abs(wrapToPiLocal(theta_backward - theta_forward(idx))));
    iq_backward_aligned(idx) = iq_backward(back_idx);
    valid_backward_aligned(idx) = valid_backward(back_idx);
end

combined_valid = valid_forward & valid_backward_aligned;
iq_cog_raw = 0.5 * (iq_forward + iq_backward_aligned);
iq_fric_raw = 0.5 * (iq_forward - iq_backward_aligned);

if nnz(combined_valid) >= 3
    iq_cog = local_fill_periodic(theta_forward, iq_cog_raw, combined_valid);
else
    iq_cog = iq_cog_raw;
end

iq_ff = iq_cog - mean(iq_cog);
iq_ff = local_periodic_smooth(iq_ff, 5);

ff_table = zeros(360, 1);
ff_table(1:cfg.scan_points) = iq_ff(:);

scan_result = struct();
scan_result.theta = theta_forward;
scan_result.iq_forward = iq_forward;
scan_result.iq_backward = iq_backward_aligned;
scan_result.iq_cog = iq_cog;
scan_result.iq_fric = iq_fric_raw;
scan_result.iq_ff = iq_ff;
scan_result.valid_mask = combined_valid;
end

function validation = local_validate_table(ff_table, cfg, motor, inverter, control, simcfg)
control.vib.mode = 'none';
control.vib.enable_learning = 0;
control.vib.enable_ff = 0;
control.vib.ff_table = zeros(360, 1);
control.vib.ff_table_file = cfg.ff_table_file;
control = apply_cogging_load_config(control, cfg);
control.vib.table_points = cfg.scan_points;
control.vib.output_limit = cfg.ff_output_limit;
control.vib.ff_enable_time = cfg.ff_enable_time;
control.vib.learn_start_time = cfg.learn_start_time;
control.vib.test_stop_time = cfg.test_stop_time;

assignin('base', 'motor', motor);
assignin('base', 'inverter', inverter);
assignin('base', 'simcfg', simcfg);

assignin('base', 'control', control);
build_vibration_comp_test;
set_param('vibration_comp_test', 'InitFcn', '');
base_out = sim('vibration_comp_test', 'ReturnWorkspaceOutputs', 'on');
if bdIsLoaded('vibration_comp_test')
    close_system('vibration_comp_test', 0);
end

control.vib.mode = 'offline';
control.vib.enable_ff = 1;
control.vib.ff_table = ff_table;
assignin('base', 'control', control);
build_vibration_comp_test;
set_param('vibration_comp_test', 'InitFcn', '');
comp_out = sim('vibration_comp_test', 'ReturnWorkspaceOutputs', 'on');
if bdIsLoaded('vibration_comp_test')
    close_system('vibration_comp_test', 0);
end

metrics = local_compare_validation(base_out, comp_out, control, cfg.ff_table_file);
validation = struct();
validation.base_out = base_out;
validation.comp_out = comp_out;
validation.metrics = metrics;
end

function metrics = local_compare_validation(base_out, comp_out, control, ff_table_file)
wm_base = base_out.get('log_vib_wm');
wm_comp = comp_out.get('log_vib_wm');
iqff_comp = comp_out.get('log_vib_iqff');

window_start = max([control.vib.learn_start_time + 0.25, control.vib.ff_enable_time + 0.10, 0.40]);
window_end = control.vib.test_stop_time;
base_y = local_window_centered(wm_base, window_start, window_end);
comp_y = local_window_centered(wm_comp, window_start, window_end);

metrics = struct();
metrics.window_start = window_start;
metrics.window_end = window_end;
metrics.base_ripple_std = std(base_y);
metrics.comp_ripple_std = std(comp_y);
metrics.base_ripple_pp = max(base_y) - min(base_y);
metrics.comp_ripple_pp = max(comp_y) - min(comp_y);
metrics.std_reduction_pct = 100 * (metrics.base_ripple_std - metrics.comp_ripple_std) / max(metrics.base_ripple_std, eps);
metrics.pp_reduction_pct = 100 * (metrics.base_ripple_pp - metrics.comp_ripple_pp) / max(metrics.base_ripple_pp, eps);
metrics.iqff_rms = local_rms_window(iqff_comp, window_start, window_end);
metrics.ff_table_file = ff_table_file;
end

function local_plot_scan_results(scan_result, validation, cfg)
if ~cfg.plot_results
    return;
end

wm_base = validation.base_out.get('log_vib_wm');
wm_comp = validation.comp_out.get('log_vib_wm');

figure('Name', 'Cogging Position Scan Study', 'Color', 'w');

subplot(2,2,1);
plot(rad2deg(scan_result.theta), scan_result.iq_forward, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.0); hold on;
plot(rad2deg(scan_result.theta), scan_result.iq_backward, 'Color', [0 0.45 0.74], 'LineWidth', 1.0);
grid on;
xlabel('Mechanical angle (deg)');
ylabel('Iq (A)');
title('Forward / Backward Scan');
legend('Forward', 'Backward', 'Location', 'best');

subplot(2,2,2);
plot(rad2deg(scan_result.theta), scan_result.iq_cog, 'Color', [0.47 0.67 0.19], 'LineWidth', 1.2); hold on;
plot(rad2deg(scan_result.theta), scan_result.iq_fric, 'Color', [0.49 0.18 0.56], 'LineWidth', 1.0);
grid on;
xlabel('Mechanical angle (deg)');
ylabel('Iq (A)');
title('Separated Components');
legend('Cogging estimate', 'Direction term', 'Location', 'best');

subplot(2,2,3);
plot(rad2deg(scan_result.theta), scan_result.iq_ff, 'k', 'LineWidth', 1.2);
grid on;
xlabel('Mechanical angle (deg)');
ylabel('Iq_{ff} (A)');
title('Feedforward Table');

subplot(2,2,4);
base_y = local_window_centered(wm_base, validation.metrics.window_start, validation.metrics.window_end);
comp_y = local_window_centered(wm_comp, validation.metrics.window_start, validation.metrics.window_end);
plot(linspace(validation.metrics.window_start, validation.metrics.window_end, numel(base_y)), base_y, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.0); hold on;
plot(linspace(validation.metrics.window_start, validation.metrics.window_end, numel(comp_y)), comp_y, 'Color', [0 0.45 0.74], 'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('Centered speed (rad/s)');
title(sprintf('Validation Ripple, std reduction %.1f%%', validation.metrics.std_reduction_pct));
legend('Baseline', 'Offline FF', 'Location', 'best');
end

function y = local_window_centered(ts, t0, t1)
mask = ts.Time >= t0 & ts.Time <= t1;
y = ts.Data(mask);
y = y(:) - mean(y(:));
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

function filled = local_fill_periodic(theta, values, valid)
theta = mod(theta(:), 2 * pi);
values = values(:);
valid = valid(:);
filled = values;
valid_theta = theta(valid);
valid_values = values(valid);
valid_theta_ext = [valid_theta(end) - 2 * pi; valid_theta; valid_theta(1) + 2 * pi];
valid_values_ext = [valid_values(end); valid_values; valid_values(1)];
filled(~valid) = interp1(valid_theta_ext, valid_values_ext, theta(~valid), 'linear');
end

function y = local_periodic_smooth(x, window_len)
x = x(:);
window_len = max(1, round(window_len));
if window_len == 1
    y = x;
    return;
end
kernel = ones(window_len, 1) / window_len;
pad = floor(window_len / 2);
x_ext = [x(end-pad+1:end); x; x(1:pad)];
y_ext = conv(x_ext, kernel, 'same');
y = y_ext(pad+1:pad+numel(x));
end

function wrapped = wrapToPiLocal(angle)
wrapped = mod(angle + pi, 2 * pi) - pi;
end