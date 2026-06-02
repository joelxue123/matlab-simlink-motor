function result = run_study(cfg)
% Run equivalent inertia identification from speed-loop step response.

if nargin < 1
    cfg = struct();
end

study_dir = fileparts(mfilename('fullpath'));
output_dir = fullfile(study_dir, 'outputs');
if exist(output_dir, 'dir') ~= 7
    mkdir(output_dir);
end

project_root = fileparts(fileparts(study_dir));
if exist(fullfile(project_root, 'init_project_paths.m'), 'file') == 2
    addpath(project_root);
end
addpath(study_dir);

init_project_paths(study_dir);
cfg = local_apply_defaults(cfg, default_config());

old_dir = pwd;
cleanup_dir = onCleanup(@() cd(old_dir));
cd(output_dir);

scale_list = local_get_scale_list(cfg);
case_count = numel(scale_list);
cases = [];

for idx = 1:case_count
    case_cfg = cfg;
    case_cfg.inertia_scale = scale_list(idx);
    case_cfg.case_name = local_case_name(case_cfg.inertia_scale, idx, case_count);
    case_result = local_run_case(case_cfg, output_dir);
    if idx == 1
        cases = repmat(case_result, case_count, 1);
    end
    cases(idx) = case_result;
end

result = struct();
result.config = cfg;
result.output_dir = output_dir;
result.cases = cases;
result.summary = local_build_summary_table(cases);

if cfg.save_outputs
    writetable(result.summary, fullfile(output_dir, 'inertia_identification_summary.csv'));
    save(fullfile(output_dir, 'inertia_identification_result.mat'), 'result');
end

if cfg.plot_results
    local_plot_cases(cases, output_dir, cfg.save_outputs);
end

disp(result.summary);

clear cleanup_dir
end

function cfg = local_apply_defaults(cfg, defaults)
fields = fieldnames(defaults);
for idx = 1:numel(fields)
    name = fields{idx};
    if ~isfield(cfg, name) || isempty(cfg.(name))
        cfg.(name) = defaults.(name);
    end
end

cfg.speed_source = lower(string(cfg.speed_source));
if ~isempty(cfg.inertia_scale_list)
    cfg.inertia_scale_list = cfg.inertia_scale_list(:).';
end
end

function scale_list = local_get_scale_list(cfg)
if ~isempty(cfg.inertia_scale_list)
    scale_list = cfg.inertia_scale_list;
else
    scale_list = cfg.inertia_scale;
end
end

function case_name = local_case_name(inertia_scale, idx, case_count)
if case_count == 1
    case_name = sprintf('Jx_%g', inertia_scale);
else
    case_name = sprintf('case%02d_Jx_%g', idx, inertia_scale);
end
case_name = regexprep(case_name, '[^a-zA-Z0-9_\.-]', '_');
end

function case_result = local_run_case(cfg, output_dir)
motor_control_params;

nominal_motor = motor;
simcfg.stop_time = max(simcfg.stop_time, cfg.stop_time_s);
motor.J = nominal_motor.J * cfg.inertia_scale;

if cfg.redesign_speed_pi
    control.pi_speed.Kp = ...
        2 * control.speed_damping * control.speed_bandwidth_rad_s * motor.J ...
        / motor.torque_constant;
    control.pi_speed.Ki = ...
        control.speed_bandwidth_rad_s^2 * motor.J / motor.torque_constant;
end

assignin('base', 'motor', motor);
assignin('base', 'inverter', inverter);
assignin('base', 'control', control);
assignin('base', 'simcfg', simcfg);

build_speedloop_kf_test;
set_param(cfg.model_name, 'InitFcn', '');
sim_out = sim(cfg.model_name, 'ReturnWorkspaceOutputs', 'on');
if cfg.close_model && bdIsLoaded(cfg.model_name)
    close_system(cfg.model_name, 0);
end

case_result = local_collect_case(sim_out, cfg, motor, control, nominal_motor);
case_result.output_dir = output_dir;

if cfg.save_outputs
    local_save_case_artifacts(case_result, output_dir);
end
end

function case_result = local_collect_case(sim_out, cfg, motor, control, nominal_motor)
wref_ts = sim_out.get('log_wref');
wm_ts = sim_out.get('log_wm');
wkf_ts = sim_out.get('log_wkf');

if isempty(wref_ts) || isempty(wm_ts) || isempty(wkf_ts)
    error('Missing speed logs from speedloop_kf_test.');
end

t = wm_ts.Time(:);
w_ref = local_align_timeseries(wref_ts, t);
w_meas = wm_ts.Data(:);
w_kf = local_align_timeseries(wkf_ts, t);

wm_metrics = local_step_metrics(t, w_meas, cfg.step_time_s, cfg.step_down_time_s);
wkf_metrics = local_step_metrics(t, w_kf, cfg.step_time_s, cfg.step_down_time_s);
wm_estimate = local_estimate_inertia(wm_metrics.bw_rad_s, control, motor);
wkf_estimate = local_estimate_inertia(wkf_metrics.bw_rad_s, control, motor);

[selected_speed, selected_metrics, selected_estimate] = local_pick_source(cfg, w_meas, w_kf, wm_metrics, wkf_metrics, wm_estimate, wkf_estimate);

case_result = struct();
case_result.case_name = cfg.case_name;
case_result.speed_source = char(cfg.speed_source);
case_result.redesign_speed_pi = logical(cfg.redesign_speed_pi);
case_result.inertia_scale = cfg.inertia_scale;
case_result.nominal_J = nominal_motor.J;
case_result.true_J = motor.J;
case_result.time = t;
case_result.w_ref = w_ref;
case_result.w_meas = w_meas;
case_result.w_kf = w_kf;
case_result.selected_speed = selected_speed;
case_result.wm_metrics = wm_metrics;
case_result.wkf_metrics = wkf_metrics;
case_result.selected_metrics = selected_metrics;
case_result.wm_estimate = wm_estimate;
case_result.wkf_estimate = wkf_estimate;
case_result.selected_estimate = selected_estimate;
case_result.kf_delay_s = wkf_metrics.t50 - wm_metrics.t50;
case_result.kf_delay_us = case_result.kf_delay_s * 1e6;
case_result.pi_speed_kp = control.pi_speed.Kp;
case_result.pi_speed_ki = control.pi_speed.Ki;
case_result.speed_bandwidth_hz_design = control.speed_bandwidth_hz;
case_result.selected_bw_hz = selected_metrics.bw_hz;
case_result.selected_bw_rad_s = selected_metrics.bw_rad_s;
case_result.selected_J_est = selected_estimate.combined;
case_result.selected_J_est_from_kp = selected_estimate.from_kp;
case_result.selected_J_est_from_ki = selected_estimate.from_ki;
case_result.selected_J_error_pct = 100 * ...
    (case_result.selected_J_est - case_result.true_J) / case_result.true_J;
end

function y = local_align_timeseries(ts, t_query)
t_source = ts.Time(:);
y_source = ts.Data(:);
y = interp1(t_source, y_source, t_query, 'previous', 'extrap');
end

function metrics = local_step_metrics(t, y, step_time, step_down_time)
pre_mask = t >= max(0, step_time - 0.01) & t < step_time - 0.002;
post_mask = t >= step_time + 0.08 & t < min(step_down_time - 0.02, step_time + 0.20);
if ~any(pre_mask) || ~any(post_mask)
    error('Insufficient data window for step metric calculation.');
end

initial_value = mean(y(pre_mask));
final_value = mean(y(post_mask));
step_amp = final_value - initial_value;
if abs(step_amp) < eps
    error('Step amplitude too small to estimate inertia.');
end

level10 = initial_value + 0.1 * step_amp;
level50 = initial_value + 0.5 * step_amp;
level63 = initial_value + 0.6321205588 * step_amp;
level90 = initial_value + 0.9 * step_amp;

search_mask = t >= step_time & t < step_down_time;
ts = t(search_mask);
ys = y(search_mask);

metrics = struct();
metrics.initial_value = initial_value;
metrics.final_value = final_value;
metrics.step_amplitude = step_amp;
metrics.t10 = local_first_cross(ts, ys, level10);
metrics.t50 = local_first_cross(ts, ys, level50);
metrics.t63 = local_first_cross(ts, ys, level63);
metrics.t90 = local_first_cross(ts, ys, level90);
metrics.rise_time_s = metrics.t90 - metrics.t10;
metrics.tau_eq_s = metrics.t63 - step_time;
metrics.bw_hz = 1 / (2 * pi * metrics.tau_eq_s);
metrics.bw_rad_s = 1 / metrics.tau_eq_s;
end

function t_cross = local_first_cross(t, y, level)
idx = find(y >= level, 1, 'first');
if isempty(idx)
    t_cross = NaN;
    return;
end
if idx == 1
    t_cross = t(1);
    return;
end

x1 = t(idx - 1);
x2 = t(idx);
y1 = y(idx - 1);
y2 = y(idx);
if y2 == y1
    t_cross = x2;
else
    t_cross = x1 + (level - y1) * (x2 - x1) / (y2 - y1);
end
end

function estimate = local_estimate_inertia(bw_rad_s, control, motor)
estimate = struct();
estimate.bw_hz = bw_rad_s / (2 * pi);
estimate.bw_rad_s = bw_rad_s;
estimate.from_kp = NaN;
estimate.from_ki = NaN;
estimate.combined = NaN;

if ~isfinite(bw_rad_s) || bw_rad_s <= eps
    return;
end

estimate.from_kp = control.pi_speed.Kp * motor.torque_constant ...
    / (2 * control.speed_damping * bw_rad_s);
estimate.from_ki = control.pi_speed.Ki * motor.torque_constant / (bw_rad_s^2);
values = [estimate.from_kp, estimate.from_ki];
estimate.combined = mean(values(isfinite(values)));
end

function [selected_speed, selected_metrics, selected_estimate] = local_pick_source(cfg, w_meas, w_kf, wm_metrics, wkf_metrics, wm_estimate, wkf_estimate)
if cfg.speed_source == "wm"
    selected_speed = w_meas;
    selected_metrics = wm_metrics;
    selected_estimate = wm_estimate;
    return;
end

selected_speed = w_kf;
selected_metrics = wkf_metrics;
selected_estimate = wkf_estimate;
end

function summary = local_build_summary_table(cases)
case_count = numel(cases);
case_name = cell(case_count, 1);
speed_source = cell(case_count, 1);
redesign_speed_pi = false(case_count, 1);
inertia_scale = zeros(case_count, 1);
true_J = zeros(case_count, 1);
selected_J_est = zeros(case_count, 1);
selected_J_est_from_kp = zeros(case_count, 1);
selected_J_est_from_ki = zeros(case_count, 1);
selected_J_error_pct = zeros(case_count, 1);
selected_bw_hz = zeros(case_count, 1);
wm_bw_hz = zeros(case_count, 1);
wkf_bw_hz = zeros(case_count, 1);
kf_delay_us = zeros(case_count, 1);
pi_speed_kp = zeros(case_count, 1);
pi_speed_ki = zeros(case_count, 1);

for idx = 1:case_count
    item = cases(idx);
    case_name{idx} = item.case_name;
    speed_source{idx} = item.speed_source;
    redesign_speed_pi(idx) = item.redesign_speed_pi;
    inertia_scale(idx) = item.inertia_scale;
    true_J(idx) = item.true_J;
    selected_J_est(idx) = item.selected_J_est;
    selected_J_est_from_kp(idx) = item.selected_J_est_from_kp;
    selected_J_est_from_ki(idx) = item.selected_J_est_from_ki;
    selected_J_error_pct(idx) = item.selected_J_error_pct;
    selected_bw_hz(idx) = item.selected_bw_hz;
    wm_bw_hz(idx) = item.wm_metrics.bw_hz;
    wkf_bw_hz(idx) = item.wkf_metrics.bw_hz;
    kf_delay_us(idx) = item.kf_delay_us;
    pi_speed_kp(idx) = item.pi_speed_kp;
    pi_speed_ki(idx) = item.pi_speed_ki;
end

summary = table(case_name, speed_source, redesign_speed_pi, inertia_scale, true_J, ...
    selected_J_est, selected_J_est_from_kp, selected_J_est_from_ki, ...
    selected_J_error_pct, selected_bw_hz, wm_bw_hz, wkf_bw_hz, ...
    kf_delay_us, pi_speed_kp, pi_speed_ki);
end

function local_save_case_artifacts(case_result, output_dir)
wave_table = table(case_result.time, case_result.w_ref, case_result.w_meas, ...
    case_result.w_kf, case_result.selected_speed, ...
    'VariableNames', {'time_s', 'w_ref_rad_s', 'w_meas_rad_s', 'w_kf_rad_s', 'w_selected_rad_s'});
writetable(wave_table, fullfile(output_dir, [case_result.case_name '_waveforms.csv']));
save(fullfile(output_dir, [case_result.case_name '_result.mat']), 'case_result');
end

function local_plot_cases(cases, output_dir, save_outputs)
case_count = numel(cases);
fig = figure('Name', 'Inertia Identification Study', 'Color', 'w');

subplot(2, 1, 1);
hold on;
plot(cases(1).time, cases(1).w_ref, '--', 'LineWidth', 1.2, 'Color', [0.25 0.25 0.25]);
legend_entries = [{'w ref'} cell(case_count, 1).'];
for idx = 1:case_count
    plot(cases(idx).time, cases(idx).selected_speed, 'LineWidth', 1.2);
    legend_entries{idx + 1} = sprintf('%s (%s)', cases(idx).case_name, cases(idx).speed_source);
end
grid on;
xlabel('Time (s)');
ylabel('Speed (rad/s)');
title('Selected speed response');
legend(legend_entries, 'Location', 'best');

subplot(2, 1, 2);
true_J = zeros(case_count, 1);
estimated_J = zeros(case_count, 1);
for idx = 1:case_count
    true_J(idx) = cases(idx).true_J;
    estimated_J(idx) = cases(idx).selected_J_est;
end
plot(1:case_count, true_J, 'o-', 'LineWidth', 1.2, 'MarkerSize', 6); hold on;
plot(1:case_count, estimated_J, 's-', 'LineWidth', 1.2, 'MarkerSize', 6);
grid on;
xlabel('Case index');
ylabel('J (kg*m^2)');
title('True vs identified inertia');
legend({'true J', 'identified J'}, 'Location', 'best');

if save_outputs
    saveas(fig, fullfile(output_dir, 'inertia_identification_summary.png'));
end
end