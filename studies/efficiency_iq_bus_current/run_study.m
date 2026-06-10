function result = run_study(cfg)
% Sweep speed and target Iq, then estimate DC-bus current and efficiency.
%
% The study reuses the folder speed-loop model. Each operating point sets a
% speed reference and a load torque that asks the speed loop to hold the
% requested q-axis current in steady state.

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

local_prepare_model(cfg);
cleanup_model = onCleanup(@() local_close_model(cfg));

speed_list = cfg.speed_rpm_list(:).';
iq_list = cfg.iq_target_a_list(:).';
if cfg.include_negative_iq
    iq_list = unique([-fliplr(abs(iq_list)), iq_list], 'stable');
end

cases = [];
case_index = 0;
for speed_idx = 1:numel(speed_list)
    for iq_idx = 1:numel(iq_list)
        case_index = case_index + 1;
        case_cfg = cfg;
        case_cfg.speed_rpm = speed_list(speed_idx);
        case_cfg.iq_target_a = iq_list(iq_idx);
        case_cfg.case_name = local_case_name(case_cfg.speed_rpm, case_cfg.iq_target_a, case_index);

        case_result = local_run_case(case_cfg, output_dir);
        if case_index == 1
            cases = repmat(case_result, numel(speed_list) * numel(iq_list), 1);
        end
        cases(case_index) = case_result;
    end
end

result = struct();
result.config = cfg;
result.output_dir = output_dir;
result.cases = cases;
result.summary = local_build_summary_table(cases);
result.output_files = local_output_files(output_dir, cfg);

if cfg.save_outputs
    writetable(result.summary, result.output_files.summary_csv);
    save(result.output_files.result_mat, 'result');
end

if cfg.plot_results
    local_plot_results(result.summary, cfg.save_outputs, result.output_files.plot_png);
end

disp(result.summary);

clear cleanup_model
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

cfg.speed_rpm_list = cfg.speed_rpm_list(:).';
cfg.iq_target_a_list = cfg.iq_target_a_list(:).';
if isempty(cfg.eval_end_s)
    cfg.eval_end_s = cfg.stop_time_s;
end
if cfg.eval_start_s >= cfg.eval_end_s
    error('eval_start_s must be smaller than eval_end_s.');
end
if ~isempty(cfg.drive_efficiency) && (cfg.drive_efficiency <= 0 || cfg.drive_efficiency > 1)
    error('drive_efficiency must be in the interval (0, 1].');
end
local_validate_struct_override(cfg, 'motor');
local_validate_struct_override(cfg, 'inverter');
local_validate_struct_override(cfg, 'control');
local_validate_struct_override(cfg, 'simcfg');
local_validate_positive_field(cfg.inverter, 'Vdc', 'cfg.inverter.Vdc');
local_validate_positive_field(cfg.inverter, 'current_limit', 'cfg.inverter.current_limit');
local_validate_efficiency_field(cfg.inverter, 'drive_efficiency', 'cfg.inverter.drive_efficiency');
local_validate_positive_field(cfg.motor, 'pole_pairs', 'cfg.motor.pole_pairs');
local_validate_positive_field(cfg.motor, 'kv_vrms_per_krpm', 'cfg.motor.kv_vrms_per_krpm');
local_validate_positive_field(cfg.motor, 'back_emf_vrms_per_krpm', 'cfg.motor.back_emf_vrms_per_krpm');
local_validate_positive_field(cfg.motor, 'line_to_line_resistance', 'cfg.motor.line_to_line_resistance');
local_validate_positive_field(cfg.motor, 'line_to_line_inductance', 'cfg.motor.line_to_line_inductance');
local_validate_positive_field(cfg.motor, 'J', 'cfg.motor.J');
end

function local_validate_struct_override(cfg, field_name)
if ~isstruct(cfg.(field_name))
    error('cfg.%s must be a struct.', field_name);
end
end

function local_validate_positive_field(parent, field_name, label)
if isfield(parent, field_name) && parent.(field_name) <= 0
    error('%s must be positive.', label);
end
end

function local_validate_efficiency_field(parent, field_name, label)
if isfield(parent, field_name) && ...
        (parent.(field_name) <= 0 || parent.(field_name) > 1)
    error('%s must be in the interval (0, 1].', label);
end
end

function case_name = local_case_name(speed_rpm, iq_target_a, idx)
case_name = sprintf('case%03d_%grpm_%gA', idx, speed_rpm, iq_target_a);
case_name = regexprep(case_name, '[^a-zA-Z0-9_\.-]', '_');
end

function local_prepare_model(cfg)
motor_control_params;
[motor, inverter, control, simcfg] = local_apply_customer_parameters( ...
    motor, inverter, control, simcfg, cfg);
simcfg.stop_time = cfg.stop_time_s;

assignin('base', 'motor', motor);
assignin('base', 'inverter', inverter);
assignin('base', 'control', control);
assignin('base', 'simcfg', simcfg);

build_speedloop_kf_test;
set_param(cfg.model_name, 'InitFcn', '');
local_add_power_logs(cfg.model_name);
end

function local_close_model(cfg)
if cfg.close_model && bdIsLoaded(cfg.model_name)
    close_system(cfg.model_name, 0);
end
end

function case_result = local_run_case(cfg, output_dir)
motor_control_params;
[motor, inverter, control, simcfg] = local_apply_customer_parameters( ...
    motor, inverter, control, simcfg, cfg);

simcfg.stop_time = cfg.stop_time_s;
motor.speed_ref_rpm = cfg.speed_rpm;
motor.speed_ref_mech_rad_s = motor.speed_ref_rpm * 2 * pi / 60;
motor.speed_ref_elec_rad_s = motor.speed_ref_mech_rad_s * motor.pole_pairs;

target_load_torque = motor.torque_constant * cfg.iq_target_a - ...
    motor.B * motor.speed_ref_mech_rad_s;
motor.load_torque = 0;
control.load_step_time = cfg.speed_step_time_s;
control.load_step_torque = target_load_torque;
control.speed_ramp_time = cfg.speed_step_time_s;
control.iq_ref_limit = max(inverter.current_limit, abs(cfg.iq_target_a) * 1.25 + 0.5);
control.pi_speed.output_limit = control.iq_ref_limit;

assignin('base', 'motor', motor);
assignin('base', 'inverter', inverter);
assignin('base', 'control', control);
assignin('base', 'simcfg', simcfg);

local_configure_speed_reference(cfg.model_name, cfg);

sim_out = sim(cfg.model_name, 'ReturnWorkspaceOutputs', 'on');

case_result = local_collect_case(sim_out, cfg, motor, inverter, target_load_torque);
case_result.output_dir = output_dir;
end

function [motor, inverter, control, simcfg] = local_apply_customer_parameters( ...
    motor, inverter, control, simcfg, cfg)
simcfg = local_overlay_struct(simcfg, cfg.simcfg);
inverter = local_overlay_struct(inverter, cfg.inverter);
motor = local_overlay_struct(motor, cfg.motor);
control = local_overlay_struct(control, cfg.control);

if ~isempty(cfg.drive_efficiency)
    inverter.drive_efficiency = cfg.drive_efficiency;
end

[motor, inverter, control] = local_refresh_derived_parameters( ...
    motor, inverter, control, simcfg, cfg);
control = local_overlay_struct(control, cfg.control);
end

function target = local_overlay_struct(target, overlay)
if isempty(overlay)
    return
end

names = fieldnames(overlay);
for idx = 1:numel(names)
    name = names{idx};
    value = overlay.(name);
    if isstruct(value) && isfield(target, name) && isstruct(target.(name))
        target.(name) = local_overlay_struct(target.(name), value);
    else
        target.(name) = value;
    end
end
end

function [motor, inverter, control] = local_refresh_derived_parameters( ...
    motor, inverter, control, simcfg, cfg)
motor_cfg = cfg.motor;
inverter_cfg = cfg.inverter;

if isfield(motor_cfg, 'kv_vrms_per_krpm')
    motor.back_emf_vrms_per_krpm = motor.kv_vrms_per_krpm;
elseif isfield(motor_cfg, 'back_emf_vrms_per_krpm')
    motor.kv_vrms_per_krpm = motor.back_emf_vrms_per_krpm;
end

if isfield(inverter_cfg, 'Vdc') && ~isfield(inverter_cfg, 'modulation_limit')
    inverter.modulation_limit = 0.577 * inverter.Vdc;
end

if isfield(motor_cfg, 'line_to_line_resistance') && ~isfield(motor_cfg, 'Rs')
    motor.Rs = motor.line_to_line_resistance / 2;
elseif isfield(motor_cfg, 'Rs') && ~isfield(motor_cfg, 'line_to_line_resistance')
    motor.line_to_line_resistance = 2 * motor.Rs;
end

if isfield(motor_cfg, 'line_to_line_inductance')
    if ~isfield(motor_cfg, 'Ld')
        motor.Ld = motor.line_to_line_inductance / 2;
    end
    if ~isfield(motor_cfg, 'Lq')
        motor.Lq = motor.Ld;
    end
elseif isfield(motor_cfg, 'Ld') && ~isfield(motor_cfg, 'line_to_line_inductance')
    motor.line_to_line_inductance = 2 * motor.Ld;
end

should_refresh_flux = isfield(motor_cfg, 'back_emf_vrms_per_krpm') || ...
    isfield(motor_cfg, 'kv_vrms_per_krpm') || ...
    isfield(motor_cfg, 'pole_pairs');
if should_refresh_flux && ~isfield(motor_cfg, 'psi_f')
    motor.psi_f = motor.back_emf_vrms_per_krpm / ...
        (sqrt(3/2) * motor.pole_pairs * (1000 * 2 * pi / 60));
end

if ~isfield(motor_cfg, 'torque_constant')
    motor.torque_constant = 1.5 * motor.pole_pairs * motor.psi_f;
end
motor.saliency_ratio = motor.Lq / motor.Ld;
motor.speed_ref_mech_rad_s = motor.speed_ref_rpm * 2 * pi / 60;
motor.speed_ref_elec_rad_s = motor.speed_ref_mech_rad_s * motor.pole_pairs;

control.current_bandwidth_rad_s = 2 * pi * control.current_bandwidth_hz;
control.pi_id.Kp = motor.Ld * control.current_bandwidth_rad_s;
control.pi_id.Ki = motor.Rs * control.current_bandwidth_rad_s;
control.pi_iq.Kp = motor.Lq * control.current_bandwidth_rad_s;
control.pi_iq.Ki = motor.Rs * control.current_bandwidth_rad_s;
control.pi_id.output_limit = inverter.modulation_limit;
control.pi_iq.output_limit = inverter.modulation_limit;
control.iq_ref_limit = inverter.current_limit;

control.tau_current_cl = 1 / (2 * pi * control.current_bandwidth_hz);
control.tau_speed_delay = 1.5 * simcfg.Ts_speed;
control.tau_sigma = control.tau_current_cl + control.tau_speed_delay;
control.speed_bandwidth_hz = 1 / (2 * pi * 3 * control.tau_sigma);
if strcmp(control.speed_feedback_source, 'w_kf')
    control.speed_bandwidth_hz = min(control.speed_bandwidth_hz, 40);
end
control.speed_bandwidth_rad_s = 2 * pi * control.speed_bandwidth_hz;
control.pi_speed.Kp = ...
    2 * control.speed_damping * control.speed_bandwidth_rad_s * motor.J ...
    / motor.torque_constant;
control.pi_speed.Ki = ...
    control.speed_bandwidth_rad_s^2 * motor.J / motor.torque_constant;
control.pi_speed.output_limit = control.iq_ref_limit;
control.speed_ref_filter_tau = 1 / (2 * pi * control.speed_bandwidth_hz);
end

function local_configure_speed_reference(model_name, cfg)
set_param([model_name '/WrefStepUp'], ...
    'Time', sprintf('%.17g', cfg.speed_step_time_s), ...
    'After', sprintf('%.17g', cfg.speed_rpm * 2 * pi / 60));
set_param([model_name '/WrefStepDown'], ...
    'Time', sprintf('%.17g', cfg.stop_time_s + 1), ...
    'After', '0');
end

function local_add_power_logs(model_name)
local_add_timeseries_log(model_name, 'log_ia', 'ia', [520 1680 610 1705]);
local_add_timeseries_log(model_name, 'log_ib', 'ib', [520 1715 610 1740]);
local_add_timeseries_log(model_name, 'log_ic', 'ic', [520 1750 610 1775]);
local_add_timeseries_log(model_name, 'log_id_meas', 'id_meas', [520 1785 610 1810]);
local_add_timeseries_log(model_name, 'log_iq_meas', 'iq_meas', [520 1820 610 1845]);
local_add_timeseries_log(model_name, 'log_iq_ref', 'iq_ref', [520 1855 610 1880]);
local_add_timeseries_log(model_name, 'log_vd_ref', 'vd_ref', [520 1890 610 1915]);
local_add_timeseries_log(model_name, 'log_vq_ref', 'vq_ref', [520 1925 610 1950]);
local_add_timeseries_log(model_name, 'log_vabc', 'Vabc_out', [520 1960 610 1985]);
end

function local_add_timeseries_log(model_name, variable_name, goto_tag, log_pos)
signal_name = regexprep(variable_name, '^log_', '');
block_name = ['Log_' signal_name];
from_name = ['From_' signal_name '_for_log'];
block_path = [model_name '/' block_name];
from_path = [model_name '/' from_name];

if ~isempty(find_system(model_name, 'SearchDepth', 1, 'Name', block_name))
    delete_block(block_path);
end
if ~isempty(find_system(model_name, 'SearchDepth', 1, 'Name', from_name))
    delete_block(from_path);
end

from_pos = log_pos + [-120 0 -120 0];
add_block('simulink/Signal Routing/From', from_path, ...
    'Position', from_pos, 'GotoTag', goto_tag);
add_block('simulink/Sinks/To Workspace', block_path, ...
    'Position', log_pos, ...
    'VariableName', variable_name, ...
    'SaveFormat', 'Timeseries');
add_line(model_name, [from_name '/1'], [block_name '/1']);
end

function case_result = local_collect_case(sim_out, cfg, motor, inverter, target_load_torque)
wm_ts = sim_out.get('log_wm');
wkf_ts = sim_out.get('log_wkf');
wref_ts = sim_out.get('log_wref');
iq_ref_ts = sim_out.get('log_iq_ref');
iq_meas_ts = sim_out.get('log_iq_meas');
id_meas_ts = sim_out.get('log_id_meas');
ia_ts = sim_out.get('log_ia');
ib_ts = sim_out.get('log_ib');
ic_ts = sim_out.get('log_ic');
vd_ref_ts = sim_out.get('log_vd_ref');
vq_ref_ts = sim_out.get('log_vq_ref');
vabc_ts = sim_out.get('log_vabc');

t = wm_ts.Time(:);
w_meas = wm_ts.Data(:);
w_kf = local_align_timeseries(wkf_ts, t);
w_ref = local_align_timeseries(wref_ts, t);
iq_ref = local_align_timeseries(iq_ref_ts, t);
iq_meas = local_align_timeseries(iq_meas_ts, t);
id_meas = local_align_timeseries(id_meas_ts, t);
ia = local_align_timeseries(ia_ts, t);
ib = local_align_timeseries(ib_ts, t);
ic = local_align_timeseries(ic_ts, t);
vd_ref = local_align_timeseries(vd_ref_ts, t);
vq_ref = local_align_timeseries(vq_ref_ts, t);
vabc = local_align_timeseries(vabc_ts, t);

if size(vabc, 2) ~= 3
    vabc = reshape(vabc, [], 3);
end

p_elec = vabc(:, 1) .* ia + vabc(:, 2) .* ib + vabc(:, 3) .* ic;
p_dc = local_dc_power_from_motor_power(p_elec, inverter.drive_efficiency);
ibus = p_dc ./ inverter.Vdc;
v_dq_mag = hypot(vd_ref, vq_ref);
modulation_ratio = v_dq_mag ./ inverter.modulation_limit;
t_elec = motor.torque_constant .* iq_meas;
p_mech_shaft = target_load_torque .* w_meas;
p_mech_electromagnetic = t_elec .* w_meas;

eval_mask = t >= cfg.eval_start_s & t <= cfg.eval_end_s;
if ~any(eval_mask)
    error('No samples in evaluation window [%.6g, %.6g] s.', cfg.eval_start_s, cfg.eval_end_s);
end

mean_w = mean(w_meas(eval_mask));
mean_wkf = mean(w_kf(eval_mask));
mean_iq_ref = mean(iq_ref(eval_mask));
mean_iq = mean(iq_meas(eval_mask));
mean_id = mean(id_meas(eval_mask));
mean_ia_rms = rms(ia(eval_mask));
mean_ib_rms = rms(ib(eval_mask));
mean_ic_rms = rms(ic(eval_mask));
mean_phase_current_rms = mean([mean_ia_rms, mean_ib_rms, mean_ic_rms]);
mean_p_elec = mean(p_elec(eval_mask));
mean_p_dc = mean(p_dc(eval_mask));
mean_ibus = mean(ibus(eval_mask));
mean_abs_ibus = mean(abs(ibus(eval_mask)));
mean_p_mech_shaft = mean(p_mech_shaft(eval_mask));
mean_p_mech_electromagnetic = mean(p_mech_electromagnetic(eval_mask));
mean_p_cu = 3 * motor.Rs * mean_phase_current_rms^2;
mean_p_fric = motor.B * mean_w^2;
mean_p_drive_loss = mean_p_dc - mean_p_elec;
mean_vd_ref = mean(vd_ref(eval_mask));
mean_vq_ref = mean(vq_ref(eval_mask));
mean_v_dq_mag = mean(v_dq_mag(eval_mask));
mean_modulation_ratio = mean(modulation_ratio(eval_mask));
max_modulation_ratio = max(modulation_ratio(eval_mask));

motor_efficiency = local_efficiency(mean_p_mech_shaft, mean_p_elec);
system_efficiency = local_efficiency(mean_p_mech_shaft, mean_p_dc);
speed_target_rad_s = cfg.speed_rpm * 2 * pi / 60;
speed_error_pct = 100 * (mean_w - speed_target_rad_s) / max(abs(speed_target_rad_s), eps);
iq_error_a = mean_iq - cfg.iq_target_a;
iq_error_pct = 100 * iq_error_a / max(abs(cfg.iq_target_a), 1);
is_speed_valid = abs(speed_error_pct) <= cfg.speed_tolerance_pct;
is_iq_valid = abs(iq_error_a) <= max(cfg.iq_tolerance_a, abs(cfg.iq_target_a) * cfg.iq_tolerance_pct / 100);

case_result = struct();
case_result.case_name = cfg.case_name;
case_result.speed_target_rpm = cfg.speed_rpm;
case_result.iq_target_a = cfg.iq_target_a;
case_result.load_torque_nm = target_load_torque;
case_result.time = t;
case_result.w_ref = w_ref;
case_result.w_meas = w_meas;
case_result.w_kf = w_kf;
case_result.id_meas = id_meas;
case_result.iq_ref = iq_ref;
case_result.iq_meas = iq_meas;
case_result.ia = ia;
case_result.ib = ib;
case_result.ic = ic;
case_result.vd_ref = vd_ref;
case_result.vq_ref = vq_ref;
case_result.vabc = vabc;
case_result.p_elec_w = p_elec;
case_result.p_dc_w = p_dc;
case_result.ibus_a = ibus;
case_result.modulation_ratio = modulation_ratio;
case_result.p_mech_shaft_w = p_mech_shaft;
case_result.p_mech_electromagnetic_w = p_mech_electromagnetic;
case_result.eval_window_s = [cfg.eval_start_s, cfg.eval_end_s];
case_result.mean_speed_rpm = mean_w * 60 / (2 * pi);
case_result.mean_w_rad_s = mean_w;
case_result.mean_wkf_rad_s = mean_wkf;
case_result.mean_id_a = mean_id;
case_result.mean_iq_ref_a = mean_iq_ref;
case_result.mean_iq_a = mean_iq;
case_result.mean_phase_current_rms_a = mean_phase_current_rms;
case_result.mean_p_elec_w = mean_p_elec;
case_result.mean_p_dc_w = mean_p_dc;
case_result.mean_ibus_a = mean_ibus;
case_result.mean_abs_ibus_a = mean_abs_ibus;
case_result.mean_p_mech_shaft_w = mean_p_mech_shaft;
case_result.mean_p_mech_electromagnetic_w = mean_p_mech_electromagnetic;
case_result.mean_p_cu_w = mean_p_cu;
case_result.mean_p_fric_w = mean_p_fric;
case_result.mean_p_drive_loss_w = mean_p_drive_loss;
case_result.mean_vd_ref_v = mean_vd_ref;
case_result.mean_vq_ref_v = mean_vq_ref;
case_result.mean_v_dq_mag_v = mean_v_dq_mag;
case_result.mean_modulation_ratio = mean_modulation_ratio;
case_result.max_modulation_ratio = max_modulation_ratio;
case_result.motor_efficiency = motor_efficiency;
case_result.motor_efficiency_pct = 100 * motor_efficiency;
case_result.system_efficiency = system_efficiency;
case_result.system_efficiency_pct = 100 * system_efficiency;
case_result.efficiency = system_efficiency;
case_result.efficiency_pct = 100 * system_efficiency;
case_result.speed_error_pct = speed_error_pct;
case_result.iq_error_a = iq_error_a;
case_result.iq_error_pct = iq_error_pct;
case_result.is_valid = logical(is_speed_valid && is_iq_valid);
case_result.is_speed_valid = logical(is_speed_valid);
case_result.is_iq_valid = logical(is_iq_valid);
case_result.motor_Rs_ohm = motor.Rs;
case_result.motor_line_to_line_resistance_ohm = motor.line_to_line_resistance;
case_result.motor_line_to_line_inductance_h = motor.line_to_line_inductance;
case_result.motor_back_emf_vrms_per_krpm = motor.back_emf_vrms_per_krpm;
case_result.motor_pole_pairs = motor.pole_pairs;
case_result.motor_psi_f_wb = motor.psi_f;
case_result.inverter_vdc_v = inverter.Vdc;
case_result.drive_efficiency = inverter.drive_efficiency;
end

function y = local_align_timeseries(ts, t_query)
t_source = ts.Time(:);
y_source = ts.Data;
if isvector(y_source)
    y_source = y_source(:);
else
    y_source = reshape(y_source, size(y_source, 1), []);
end
y = interp1(t_source, y_source, t_query, 'linear', 'extrap');
end

function eta = local_efficiency(p_mech, p_elec)
if abs(p_mech) <= eps
    eta = NaN;
elseif p_mech >= 0 && p_elec > eps
    eta = p_mech / p_elec;
elseif p_mech < 0 && p_elec < -eps
    eta = p_elec / p_mech;
else
    eta = NaN;
end

if ~isnan(eta)
    eta = max(0, min(eta, 1));
end
end

function p_dc = local_dc_power_from_motor_power(p_motor, drive_efficiency)
p_dc = zeros(size(p_motor));
motoring_mask = p_motor >= 0;
p_dc(motoring_mask) = p_motor(motoring_mask) ./ drive_efficiency;
p_dc(~motoring_mask) = p_motor(~motoring_mask) .* drive_efficiency;
end

function summary = local_build_summary_table(cases)
count = numel(cases);
case_name = strings(count, 1);
speed_target_rpm = zeros(count, 1);
iq_target_a = zeros(count, 1);
load_torque_nm = zeros(count, 1);
mean_speed_rpm = zeros(count, 1);
mean_id_a = zeros(count, 1);
mean_iq_ref_a = zeros(count, 1);
mean_iq_a = zeros(count, 1);
mean_phase_current_rms_a = zeros(count, 1);
mean_p_elec_w = zeros(count, 1);
mean_p_dc_w = zeros(count, 1);
mean_ibus_a = zeros(count, 1);
mean_abs_ibus_a = zeros(count, 1);
mean_p_mech_shaft_w = zeros(count, 1);
mean_p_cu_w = zeros(count, 1);
mean_p_fric_w = zeros(count, 1);
mean_p_drive_loss_w = zeros(count, 1);
mean_vd_ref_v = zeros(count, 1);
mean_vq_ref_v = zeros(count, 1);
mean_v_dq_mag_v = zeros(count, 1);
mean_modulation_ratio = zeros(count, 1);
max_modulation_ratio = zeros(count, 1);
motor_efficiency_pct = zeros(count, 1);
efficiency_pct = zeros(count, 1);
speed_error_pct = zeros(count, 1);
iq_error_a = zeros(count, 1);
is_valid = false(count, 1);
inverter_vdc_v = zeros(count, 1);
drive_efficiency = zeros(count, 1);

for idx = 1:count
    item = cases(idx);
    case_name(idx) = string(item.case_name);
    speed_target_rpm(idx) = item.speed_target_rpm;
    iq_target_a(idx) = item.iq_target_a;
    load_torque_nm(idx) = item.load_torque_nm;
    mean_speed_rpm(idx) = item.mean_speed_rpm;
    mean_id_a(idx) = item.mean_id_a;
    mean_iq_ref_a(idx) = item.mean_iq_ref_a;
    mean_iq_a(idx) = item.mean_iq_a;
    mean_phase_current_rms_a(idx) = item.mean_phase_current_rms_a;
    mean_p_elec_w(idx) = item.mean_p_elec_w;
    mean_p_dc_w(idx) = item.mean_p_dc_w;
    mean_ibus_a(idx) = item.mean_ibus_a;
    mean_abs_ibus_a(idx) = item.mean_abs_ibus_a;
    mean_p_mech_shaft_w(idx) = item.mean_p_mech_shaft_w;
    mean_p_cu_w(idx) = item.mean_p_cu_w;
    mean_p_fric_w(idx) = item.mean_p_fric_w;
    mean_p_drive_loss_w(idx) = item.mean_p_drive_loss_w;
    mean_vd_ref_v(idx) = item.mean_vd_ref_v;
    mean_vq_ref_v(idx) = item.mean_vq_ref_v;
    mean_v_dq_mag_v(idx) = item.mean_v_dq_mag_v;
    mean_modulation_ratio(idx) = item.mean_modulation_ratio;
    max_modulation_ratio(idx) = item.max_modulation_ratio;
    motor_efficiency_pct(idx) = item.motor_efficiency_pct;
    efficiency_pct(idx) = item.efficiency_pct;
    speed_error_pct(idx) = item.speed_error_pct;
    iq_error_a(idx) = item.iq_error_a;
    is_valid(idx) = item.is_valid;
    inverter_vdc_v(idx) = item.inverter_vdc_v;
    drive_efficiency(idx) = item.drive_efficiency;
end

summary = table(case_name, speed_target_rpm, iq_target_a, load_torque_nm, ...
    mean_speed_rpm, mean_id_a, mean_iq_ref_a, mean_iq_a, ...
    mean_phase_current_rms_a, mean_p_elec_w, mean_p_dc_w, mean_ibus_a, ...
    mean_abs_ibus_a, mean_p_mech_shaft_w, mean_p_cu_w, mean_p_fric_w, ...
    mean_p_drive_loss_w, mean_vd_ref_v, mean_vq_ref_v, mean_v_dq_mag_v, ...
    mean_modulation_ratio, max_modulation_ratio, motor_efficiency_pct, ...
    efficiency_pct, speed_error_pct, iq_error_a, is_valid, ...
    inverter_vdc_v, drive_efficiency);
end

function output_files = local_output_files(output_dir, cfg)
base_name = 'efficiency_iq_bus_current';
tag = strtrim(string(cfg.output_tag));
if strlength(tag) > 0
    safe_tag = regexprep(char(tag), '[^a-zA-Z0-9_\.-]', '_');
    base_name = [base_name '_' safe_tag];
end

output_files = struct();
output_files.summary_csv = fullfile(output_dir, [base_name '_summary.csv']);
output_files.result_mat = fullfile(output_dir, [base_name '_result.mat']);
output_files.plot_png = fullfile(output_dir, [base_name '.png']);
end

function local_plot_results(summary, save_outputs, plot_filename)
speed_values = unique(summary.speed_target_rpm, 'stable');

fig = figure('Name', 'Iq vs DC bus current and system efficiency', 'Color', 'w');
tiledlayout(fig, 2, 1, 'TileSpacing', 'compact');

nexttile;
hold on;
grid on;
for idx = 1:numel(speed_values)
    mask = summary.speed_target_rpm == speed_values(idx);
    plot(summary.mean_iq_a(mask), summary.mean_ibus_a(mask), 'o-', ...
        'DisplayName', sprintf('%g rpm', speed_values(idx)));
end
xlabel('Iq measured (A)');
ylabel('DC bus current (A)');
title('Iq vs DC bus current with drive efficiency');
legend('Location', 'best');

nexttile;
hold on;
grid on;
for idx = 1:numel(speed_values)
    mask = summary.speed_target_rpm == speed_values(idx);
    plot(summary.mean_iq_a(mask), summary.efficiency_pct(mask), 'o-', ...
        'DisplayName', sprintf('%g rpm', speed_values(idx)));
end
xlabel('Iq measured (A)');
ylabel('System efficiency (%)');
title('Pout / Pdc estimate');
legend('Location', 'best');

if save_outputs
    saveas(fig, plot_filename);
end
end
