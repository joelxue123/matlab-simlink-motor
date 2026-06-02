function result = reproduce_position_scan_validation(cfg)
% Reproduce the position-scan -> save -> validate chain with fixed settings.
%
% Usage:
%   result = reproduce_position_scan_validation;
%   result = reproduce_position_scan_validation(struct('scan_points', 180));

if nargin < 1
    cfg = struct();
end

cfg = local_fill_defaults(cfg);
local_clear_previous_state(cfg.clear_workspace_results);
bdclose('all');

motor_control_params;

control.pos_ref_mode = 'scan_table';
control.pos_use_planner = false;
control.use_periodic_load = true;

control.pos_scan.start_time = cfg.start_time;
control.pos_scan.hold_time = cfg.hold_time;
control.pos_scan.points = cfg.scan_points;
control.pos_scan.theta_table = zeros(360, 1);
control.pos_scan.theta_table(1:cfg.scan_points) = local_theta_table(cfg.scan_points);

control.pos_bandwidth_hz = control.pos_bandwidth_hz * cfg.position_bandwidth_scale;
control.pos_bandwidth_rad_s = 2 * pi * control.pos_bandwidth_hz;
control.pi_pos.Kp = control.pos_bandwidth_rad_s;
control.pi_pos.output_limit = motor.speed_ref_mech_rad_s * cfg.position_output_limit_scale;
control.pid_pos.Kp = 2 * control.pid_pos.damping * control.pos_bandwidth_rad_s;
control.pid_pos.Ki_cont = 0;
control.pid_pos.Ki = 0;
control.pid_pos.output_limit = motor.speed_ref_mech_rad_s * cfg.position_output_limit_scale;

control = apply_cogging_load_config(control, cfg);

simcfg.stop_time = cfg.start_time + cfg.scan_points * cfg.hold_time + cfg.stop_margin;

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

extract_cfg = struct();
extract_cfg.avg_time = cfg.avg_time;
extract_cfg.settle_time = cfg.settle_time;
extract_cfg.use_iq_meas = cfg.use_iq_meas;
extract_cfg.use_last_sample = cfg.use_last_sample;
extract_cfg.file_name = cfg.scan_file_name;
scan_result = save_position_scan_iq_table(sim_out, extract_cfg);

validate_cfg = struct();
validate_cfg.plot_results = cfg.plot_results;
validate_cfg.table_points = cfg.scan_points;
validate_cfg.ff_table_file = cfg.ff_table_file;
validate_cfg.ff_csv_file = cfg.ff_csv_file;
validate_cfg.ff_text_file = cfg.ff_text_file;
validated_scan_result = validate_position_scan_ff_table(scan_result, validate_cfg);

consistency = local_check_consistency(scan_result, validated_scan_result.config);

result = struct();
result.scan_result = scan_result;
result.validated_scan_result = validated_scan_result;
result.consistency = consistency;

assignin('base', 'reproduce_scan_validation_result', result);

fprintf('\n=== Reproduce position scan validation ===\n');
fprintf('Scan file              : %s\n', cfg.scan_file_name);
fprintf('FF table file          : %s\n', cfg.ff_table_file);
fprintf('FF csv file            : %s\n', validated_scan_result.export.csv_file);
fprintf('FF text file           : %s\n', validated_scan_result.export.text_file);
fprintf('scan harmonic1         : %g\n', scan_result.disturbance.load_harmonic1);
fprintf('scan harmonic2         : %g\n', scan_result.disturbance.load_harmonic2);
fprintf('validation harmonic1   : %g\n', validated_scan_result.config.harmonic1);
fprintf('validation harmonic2   : %g\n', validated_scan_result.config.harmonic2);
fprintf('config match           : %d\n', consistency.all_match);
fprintf('std reduction (%%)      : %.2f\n', validated_scan_result.validation.metrics.std_reduction_pct);
fprintf('p-p reduction (%%)      : %.2f\n', validated_scan_result.validation.metrics.pp_reduction_pct);

if ~consistency.all_match
    error('Scan disturbance metadata and validation config do not match.');
end
end

function cfg = local_fill_defaults(cfg)
defaults = struct();
defaults.scan_points = 72;
defaults.start_time = 0.05;
defaults.hold_time = 0.02;
defaults.avg_time = 0.008;
defaults.settle_time = 0.012;
defaults.stop_margin = 0.02;
defaults.position_bandwidth_scale = 0.30;
defaults.position_output_limit_scale = 0.25;
load_defaults = cogging_load_config(cfg);
defaults.load_base_torque = load_defaults.load_base_torque;
defaults.harmonic1 = load_defaults.harmonic1;
defaults.harmonic2 = load_defaults.harmonic2;
defaults.amp1 = load_defaults.amp1;
defaults.amp2 = load_defaults.amp2;
defaults.phase1_deg = load_defaults.phase1_deg;
defaults.phase2_deg = load_defaults.phase2_deg;
defaults.use_iq_meas = false;
defaults.use_last_sample = false;
defaults.plot_results = false;
defaults.scan_file_name = 'position_scan_iq_result.mat';
defaults.ff_table_file = 'validated_scan_ff_table.mat';
defaults.ff_csv_file = 'validated_scan_ff_table.csv';
defaults.ff_text_file = 'validated_scan_ff_table.txt';
defaults.clear_workspace_results = true;

names = fieldnames(defaults);
for idx = 1:numel(names)
    name = names{idx};
    if ~isfield(cfg, name) || isempty(cfg.(name))
        cfg.(name) = defaults.(name);
    end
end

cfg.scan_points = max(24, min(360, round(cfg.scan_points)));
cfg.avg_time = max(0.001, cfg.avg_time);
cfg.settle_time = max(0, cfg.settle_time);
end

function theta = local_theta_table(scan_points)
theta = linspace(0, 2 * pi, scan_points + 1);
theta(end) = [];
theta = theta(:);
end

function local_clear_previous_state(clear_results)
if ~clear_results
    return;
end

evalin('base', ['clear position_scan_iq_result position_scan_iq_result_backward ' ...
    'validated_scan_result validated_scan_ff_table reproduce_scan_validation_result sim_out']);
end

function consistency = local_check_consistency(scan_result, validate_cfg)
consistency = struct();
consistency.harmonic1 = isequaln(scan_result.disturbance.load_harmonic1, validate_cfg.harmonic1);
consistency.harmonic2 = isequaln(scan_result.disturbance.load_harmonic2, validate_cfg.harmonic2);
consistency.amp1 = isequaln(scan_result.disturbance.load_amp1, validate_cfg.amp1);
consistency.amp2 = isequaln(scan_result.disturbance.load_amp2, validate_cfg.amp2);
consistency.phase1_deg = isequaln(scan_result.disturbance.load_phase1_deg, validate_cfg.phase1_deg);
consistency.phase2_deg = isequaln(scan_result.disturbance.load_phase2_deg, validate_cfg.phase2_deg);
consistency.load_base_torque = isequaln(scan_result.disturbance.load_base_torque, validate_cfg.load_base_torque);
consistency.all_match = all(struct2array(consistency));
end