function result = run_study(cfg)
% Run current-loop PI validation on the average-voltage PMSM model.

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
cfg = local_apply_defaults(cfg);

old_dir = pwd;
cleanup_dir = onCleanup(@() cd(old_dir));
cd(output_dir);

bandwidth_list = local_get_bandwidth_list(cfg);
case_count = numel(bandwidth_list);
cases = [];

for idx = 1:case_count
    case_cfg = cfg;
    case_cfg.current_bandwidth_hz = bandwidth_list(idx);
    case_cfg.case_name = local_case_name(case_cfg.current_bandwidth_hz, idx, case_count);
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
    writetable(result.summary, fullfile(output_dir, 'current_loop_validation_summary.csv'));
    save(fullfile(output_dir, 'current_loop_validation_result.mat'), 'result');
end

if cfg.plot_results
    local_plot_cases(cases, output_dir, cfg.save_outputs);
end

disp(result.summary);

clear cleanup_dir
end

function cfg = local_apply_defaults(cfg)
defaults = struct();
defaults.model_name = 'currentloop_pi_test';
defaults.current_bandwidth_hz = [];
defaults.bandwidth_hz_list = [];
defaults.tuning_method = 'bandwidth';
defaults.phase_margin_deg = 60.0;
defaults.current_delay_s = [];
defaults.delay_safety_factor = 3.0;
defaults.ref_axis = 'id';
defaults.ref_waveform = 'square';
defaults.kp_scale = 1.0;
defaults.ki_scale = 1.0;
defaults.step_amplitude_a = 2.0;
defaults.step_time_s = 5e-3;
defaults.square_frequency_hz = 100;
defaults.stop_time_s = 3e-2;
defaults.inertia_scale = 200;
defaults.settling_band_a = 0.05;
defaults.plot_results = true;
defaults.save_outputs = true;

fields = fieldnames(defaults);
for idx = 1:numel(fields)
    name = fields{idx};
    if ~isfield(cfg, name) || isempty(cfg.(name))
        cfg.(name) = defaults.(name);
    end
end

if ~isempty(cfg.bandwidth_hz_list)
    cfg.bandwidth_hz_list = cfg.bandwidth_hz_list(:).';
end
cfg.ref_axis = lower(string(cfg.ref_axis));
cfg.ref_waveform = lower(string(cfg.ref_waveform));
cfg.tuning_method = lower(string(cfg.tuning_method));
end

function bandwidth_list = local_get_bandwidth_list(cfg)
motor_control_params;

if ~isempty(cfg.bandwidth_hz_list)
    bandwidth_list = cfg.bandwidth_hz_list;
elseif ~isempty(cfg.current_bandwidth_hz)
    bandwidth_list = cfg.current_bandwidth_hz;
else
    bandwidth_list = control.current_bandwidth_hz;
end
end

function case_name = local_case_name(current_bandwidth_hz, idx, case_count)
if case_count == 1
    case_name = sprintf('bw_%gHz', current_bandwidth_hz);
else
    case_name = sprintf('case%02d_bw_%gHz', idx, current_bandwidth_hz);
end
case_name = regexprep(case_name, '[^a-zA-Z0-9_\.-]', '_');
end

function case_result = local_run_case(cfg, output_dir)
motor_control_params;

simcfg.stop_time = cfg.stop_time_s;
motor.J = motor.J * cfg.inertia_scale;

control.current_bandwidth_hz = cfg.current_bandwidth_hz;
control.current_delay_s = cfg.current_delay_s;
if isempty(control.current_delay_s)
    control.current_delay_s = 1.5 * simcfg.Ts_ctrl;
end
control.requested_current_bandwidth_hz = control.current_bandwidth_hz;
control.design_phase_margin_deg = cfg.phase_margin_deg;
control.current_bandwidth_hz = local_resolve_current_bandwidth_hz(cfg, control, motor);
control.current_bandwidth_rad_s = 2 * pi * control.current_bandwidth_hz;
[control.pi_id.Kp, control.pi_id.Ki] = local_design_single_axis_pi(cfg, control, motor.Ld, motor.Rs);
[control.pi_iq.Kp, control.pi_iq.Ki] = local_design_single_axis_pi(cfg, control, motor.Lq, motor.Rs);
control.pi_id.output_limit = inverter.modulation_limit;
control.pi_iq.output_limit = inverter.modulation_limit;

validation_cfg = struct();
validation_cfg.ref_axis = char(cfg.ref_axis);
validation_cfg.ref_waveform = char(cfg.ref_waveform);
validation_cfg.step_time_s = cfg.step_time_s;
validation_cfg.step_amplitude_a = cfg.step_amplitude_a;
validation_cfg.square_frequency_hz = cfg.square_frequency_hz;
validation_ref_ts = local_build_reference_timeseries(cfg, simcfg.Ts_ctrl);

assignin('base', 'motor', motor);
assignin('base', 'inverter', inverter);
assignin('base', 'control', control);
assignin('base', 'simcfg', simcfg);
assignin('base', 'validation_cfg', validation_cfg);
assignin('base', 'validation_ref_ts', validation_ref_ts);

build_current_loop_test_model(cfg.model_name);
set_param(cfg.model_name, 'InitFcn', '');
sim_out = sim(cfg.model_name, 'ReturnWorkspaceOutputs', 'on');
if bdIsLoaded(cfg.model_name)
    close_system(cfg.model_name, 0);
end

case_result = local_collect_case(sim_out, cfg, control, inverter);
case_result.output_dir = output_dir;

if cfg.save_outputs
    local_save_case_artifacts(case_result, output_dir);
end
end

function case_result = local_collect_case(sim_out, cfg, control, inverter)
id_ref_ts = sim_out.get('log_id_ref');
iq_ref_ts = sim_out.get('log_iq_ref');
iq_meas_ts = sim_out.get('log_iq_meas');
id_meas_ts = sim_out.get('log_id_meas');
vd_ref_ts = sim_out.get('log_vd_ref');
vq_ref_ts = sim_out.get('log_vq_ref');
wm_ts = sim_out.get('log_wm');

t = iq_meas_ts.Time(:);
id_ref = local_align_timeseries(id_ref_ts, t);
iq_ref = local_align_timeseries(iq_ref_ts, t);
iq_meas = iq_meas_ts.Data(:);
id_meas = local_align_timeseries(id_meas_ts, t);
vd_ref = local_align_timeseries(vd_ref_ts, t);
vq_ref = local_align_timeseries(vq_ref_ts, t);
wm = local_align_timeseries(wm_ts, t);

if cfg.ref_axis == "id"
    active_ref = id_ref;
    active_meas = id_meas;
    cross_meas = iq_meas;
    kp = control.pi_id.Kp;
    ki = control.pi_id.Ki;
else
    active_ref = iq_ref;
    active_meas = iq_meas;
    cross_meas = id_meas;
    kp = control.pi_iq.Kp;
    ki = control.pi_iq.Ki;
end

eval_mask = local_build_eval_mask(t, cfg);
eval_t = t(eval_mask);
eval_active_ref = active_ref(eval_mask);
eval_active_meas = active_meas(eval_mask);
eval_cross_meas = cross_meas(eval_mask);
eval_vd_ref = vd_ref(eval_mask);
eval_vq_ref = vq_ref(eval_mask);
eval_wm = wm(eval_mask);

target = cfg.step_amplitude_a;
settling_band = max(cfg.settling_band_a, 0.02 * abs(target));
steady_samples = max(5, ceil(numel(eval_active_meas) * 0.1));
steady_state = mean(eval_active_meas(end-steady_samples+1:end));

rise_time_s = local_rise_time(eval_t, eval_active_meas, target);
settling_time_s = local_settling_time(eval_t, eval_active_meas, target, settling_band);

case_result = struct();
case_result.case_name = cfg.case_name;
case_result.time = t;
case_result.ref_axis = char(cfg.ref_axis);
case_result.ref_waveform = char(cfg.ref_waveform);
case_result.tuning_method = char(cfg.tuning_method);
case_result.id_ref = id_ref;
case_result.iq_ref = iq_ref;
case_result.active_ref = active_ref;
case_result.active_meas = active_meas;
case_result.iq_meas = iq_meas;
case_result.id_meas = id_meas;
case_result.vd_ref = vd_ref;
case_result.vq_ref = vq_ref;
case_result.wm = wm;
case_result.requested_current_bandwidth_hz = control.requested_current_bandwidth_hz;
case_result.current_bandwidth_hz = cfg.current_bandwidth_hz;
case_result.effective_current_bandwidth_hz = control.current_bandwidth_hz;
case_result.current_delay_s = control.current_delay_s;
case_result.design_phase_margin_deg = control.design_phase_margin_deg;
case_result.kp = kp;
case_result.ki = ki;
case_result.step_amplitude_a = cfg.step_amplitude_a;
case_result.step_time_s = cfg.step_time_s;
case_result.steady_state_a = steady_state;
case_result.steady_state_error_a = target - steady_state;
case_result.rise_time_s = rise_time_s;
case_result.settling_time_s = settling_time_s;
case_result.overshoot_a = max(eval_active_meas) - target;
case_result.rmse_a = sqrt(mean((eval_active_ref - eval_active_meas).^2));
case_result.max_abs_cross_axis_a = max(abs(eval_cross_meas));
case_result.max_abs_vd_v = max(abs(eval_vd_ref));
case_result.max_abs_vq_v = max(abs(eval_vq_ref));
case_result.max_abs_speed_rad_s = max(abs(eval_wm));
case_result.voltage_utilization = max(sqrt(eval_vd_ref.^2 + eval_vq_ref.^2)) / inverter.modulation_limit;
case_result.settling_band_a = settling_band;
end

function eval_mask = local_build_eval_mask(t, cfg)
eval_mask = t >= cfg.step_time_s;
if cfg.ref_waveform ~= "square"
    return;
end

half_period_s = 0.5 / cfg.square_frequency_hz;
eval_mask = eval_mask & t < (cfg.step_time_s + half_period_s);
end

function ref_ts = local_build_reference_timeseries(cfg, Ts)
t = (0:Ts:cfg.stop_time_s).';
ref = zeros(size(t));
active_mask = t >= cfg.step_time_s;

if cfg.ref_waveform == "square"
    phase = mod(t(active_mask) - cfg.step_time_s, 1 / cfg.square_frequency_hz);
    active_idx = find(active_mask);
    ref(active_mask) = cfg.step_amplitude_a;
    ref(active_idx(phase >= 0.5 / cfg.square_frequency_hz)) = -cfg.step_amplitude_a;
else
    ref(active_mask) = cfg.step_amplitude_a;
end

ref_ts = timeseries(ref, t);
end

function y = local_align_timeseries(ts, t_query)
t_source = ts.Time(:);
y_source = ts.Data(:);

if numel(t_source) < 2
    y = repmat(y_source(1), size(t_query));
    return;
end

y = interp1(t_source, y_source, t_query, 'previous', 'extrap');
end

function current_bandwidth_hz = local_resolve_current_bandwidth_hz(cfg, control, motor)
current_bandwidth_hz = control.current_bandwidth_hz;
if cfg.tuning_method ~= "delay_aware"
    if cfg.tuning_method == "bandwidth_pm"
        max_bw_d_hz = local_max_bandwidth_for_pm(motor.Ld, motor.Rs, control.current_delay_s, cfg.phase_margin_deg);
        max_bw_q_hz = local_max_bandwidth_for_pm(motor.Lq, motor.Rs, control.current_delay_s, cfg.phase_margin_deg);
        current_bandwidth_hz = min(current_bandwidth_hz, min(max_bw_d_hz, max_bw_q_hz));
    end
    return;
end

delay_limited_bw_hz = 1 / (2 * pi * cfg.delay_safety_factor * control.current_delay_s);
current_bandwidth_hz = min(current_bandwidth_hz, delay_limited_bw_hz);
end

function [Kp, Ki] = local_design_single_axis_pi(cfg, control, inductance_h, resistance_ohm)
omega_c = control.current_bandwidth_rad_s;

if cfg.tuning_method == "bandwidth_pm"
    phase_i_deg = 180 - cfg.phase_margin_deg - atan2d(omega_c * inductance_h, resistance_ohm) ...
        - rad2deg(omega_c * control.current_delay_s);
    phase_i_deg = min(max(phase_i_deg, 1.0), 89.0);
    omega_i = omega_c * tand(phase_i_deg);
    Kp = sqrt(resistance_ohm^2 + (omega_c * inductance_h)^2) ...
        / sqrt(1 + (omega_i / omega_c)^2);
    Ki = Kp * omega_i;
else
    Kp = inductance_h * omega_c;
    Ki = resistance_ohm * omega_c;
end

Kp = Kp * cfg.kp_scale;
Ki = Ki * cfg.ki_scale;
end

function max_bandwidth_hz = local_max_bandwidth_for_pm(inductance_h, resistance_ohm, delay_s, phase_margin_deg)
phase_i_limit_deg = 1.0;
lo = 0.0;
hi = 2 * pi * max(1, 1e4);

for idx = 1:60
    phase_i_hi = 180 - phase_margin_deg - atan2d(hi * inductance_h, resistance_ohm) - rad2deg(hi * delay_s);
    if phase_i_hi <= phase_i_limit_deg
        break;
    end
    hi = hi * 2;
end

for idx = 1:80
    mid = 0.5 * (lo + hi);
    phase_i_mid = 180 - phase_margin_deg - atan2d(mid * inductance_h, resistance_ohm) - rad2deg(mid * delay_s);
    if phase_i_mid <= phase_i_limit_deg
        hi = mid;
    else
        lo = mid;
    end
end

max_bandwidth_hz = hi / (2 * pi);
end

function rise_time_s = local_rise_time(t, signal, target)
rise_time_s = NaN;
if target == 0 || isempty(t)
    return;
end

level10 = 0.1 * target;
level90 = 0.9 * target;
idx10 = find(signal >= level10, 1, 'first');
idx90 = find(signal >= level90, 1, 'first');
if isempty(idx10) || isempty(idx90) || idx90 < idx10
    return;
end

rise_time_s = t(idx90) - t(idx10);
end

function settling_time_s = local_settling_time(t, signal, target, band)
settling_time_s = NaN;
if isempty(t)
    return;
end

for idx = 1:numel(t)
    if all(abs(signal(idx:end) - target) <= band)
        settling_time_s = t(idx) - t(1);
        return;
    end
end
end

function summary = local_build_summary_table(cases)
case_count = numel(cases);
case_name = cell(case_count, 1);
ref_axis = cell(case_count, 1);
ref_waveform = cell(case_count, 1);
tuning_method = cell(case_count, 1);
requested_current_bandwidth_hz = zeros(case_count, 1);
current_bandwidth_hz = zeros(case_count, 1);
effective_current_bandwidth_hz = zeros(case_count, 1);
current_delay_us = zeros(case_count, 1);
design_phase_margin_deg = zeros(case_count, 1);
kp = zeros(case_count, 1);
ki = zeros(case_count, 1);
rise_time_s = zeros(case_count, 1);
settling_time_s = zeros(case_count, 1);
overshoot_a = zeros(case_count, 1);
steady_state_error_a = zeros(case_count, 1);
rmse_a = zeros(case_count, 1);
max_abs_cross_axis_a = zeros(case_count, 1);
voltage_utilization = zeros(case_count, 1);
max_abs_speed_rad_s = zeros(case_count, 1);

for idx = 1:case_count
    item = cases(idx);
    case_name{idx} = item.case_name;
    ref_axis{idx} = item.ref_axis;
    ref_waveform{idx} = item.ref_waveform;
    tuning_method{idx} = item.tuning_method;
    requested_current_bandwidth_hz(idx) = item.requested_current_bandwidth_hz;
    current_bandwidth_hz(idx) = item.current_bandwidth_hz;
    effective_current_bandwidth_hz(idx) = item.effective_current_bandwidth_hz;
    current_delay_us(idx) = item.current_delay_s * 1e6;
    design_phase_margin_deg(idx) = item.design_phase_margin_deg;
    kp(idx) = item.kp;
    ki(idx) = item.ki;
    rise_time_s(idx) = item.rise_time_s;
    settling_time_s(idx) = item.settling_time_s;
    overshoot_a(idx) = item.overshoot_a;
    steady_state_error_a(idx) = item.steady_state_error_a;
    rmse_a(idx) = item.rmse_a;
    max_abs_cross_axis_a(idx) = item.max_abs_cross_axis_a;
    voltage_utilization(idx) = item.voltage_utilization;
    max_abs_speed_rad_s(idx) = item.max_abs_speed_rad_s;
end

summary = table(case_name, ref_axis, ref_waveform, tuning_method, requested_current_bandwidth_hz, ...
    current_bandwidth_hz, effective_current_bandwidth_hz, current_delay_us, design_phase_margin_deg, kp, ki, rise_time_s, ...
    settling_time_s, overshoot_a, steady_state_error_a, rmse_a, ...
    max_abs_cross_axis_a, voltage_utilization, max_abs_speed_rad_s);
end

function local_save_case_artifacts(case_result, output_dir)
wave_table = table(case_result.time, case_result.id_ref, case_result.iq_ref, ...
    case_result.id_meas, case_result.iq_meas, case_result.vd_ref, case_result.vq_ref, case_result.wm, ...
    'VariableNames', {'time_s', 'id_ref_a', 'iq_ref_a', 'id_meas_a', 'iq_meas_a', ...
    'vd_ref_v', 'vq_ref_v', 'wm_rad_s'});
wave_file = fullfile(output_dir, ['waveform_' case_result.case_name '.csv']);
mat_file = fullfile(output_dir, ['result_' case_result.case_name '.mat']);
plot_file = fullfile(output_dir, ['response_' case_result.case_name '.png']);

writetable(wave_table, wave_file);
save(mat_file, 'case_result');

fig = figure('Name', ['Current Loop Validation - ' case_result.case_name], 'Color', 'w');
subplot(3, 1, 1);
plot(case_result.time, case_result.id_ref, '--', 'LineWidth', 1.1); hold on;
plot(case_result.time, case_result.iq_ref, '--', 'LineWidth', 1.1);
plot(case_result.time, case_result.id_meas, 'LineWidth', 1.2);
plot(case_result.time, case_result.iq_meas, 'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('Current (A)');
title(sprintf('Current response: %s (%s %s)', case_result.case_name, case_result.ref_axis, case_result.ref_waveform));
legend('id ref', 'iq ref', 'id meas', 'iq meas', 'Location', 'best');

subplot(3, 1, 2);
plot(case_result.time, case_result.vd_ref, 'LineWidth', 1.1); hold on;
plot(case_result.time, case_result.vq_ref, 'LineWidth', 1.1);
grid on;
xlabel('Time (s)');
ylabel('Voltage (V)');
title('Controller output');
legend('vd ref', 'vq ref', 'Location', 'best');

subplot(3, 1, 3);
plot(case_result.time, case_result.wm, 'LineWidth', 1.1);
grid on;
xlabel('Time (s)');
ylabel('Speed (rad/s)');
title('Mechanical speed drift');

saveas(fig, plot_file);
close(fig);
end

function local_plot_cases(cases, output_dir, save_outputs)
if numel(cases) <= 1
    return;
end

fig = figure('Name', 'Current Loop Validation Summary', 'Color', 'w');

subplot(2, 1, 1);
hold on;
for idx = 1:numel(cases)
    plot(cases(idx).time, cases(idx).active_meas, 'LineWidth', 1.1, ...
        'DisplayName', cases(idx).case_name);
end
plot(cases(1).time, cases(1).active_ref, 'k--', 'LineWidth', 1.2, 'DisplayName', 'active ref');
grid on;
xlabel('Time (s)');
ylabel('Current (A)');
title(sprintf('%s response overlay', cases(1).ref_axis));
legend('Location', 'best');

subplot(2, 1, 2);
bandwidth = zeros(numel(cases), 1);
settling = zeros(numel(cases), 1);
overshoot = zeros(numel(cases), 1);
for idx = 1:numel(cases)
    bandwidth(idx) = cases(idx).current_bandwidth_hz;
    settling(idx) = cases(idx).settling_time_s;
    overshoot(idx) = cases(idx).overshoot_a;
end
yyaxis left;
plot(bandwidth, settling, 'o-', 'LineWidth', 1.1);
ylabel('Settling time (s)');
yyaxis right;
plot(bandwidth, overshoot, 's-', 'LineWidth', 1.1);
ylabel('Overshoot (A)');
grid on;
xlabel('Bandwidth (Hz)');
title('Bandwidth sweep summary');

if save_outputs
    saveas(fig, fullfile(output_dir, 'current_loop_validation_summary.png'));
end
close(fig);
end