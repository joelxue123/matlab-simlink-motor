function open_cogging_position_scan_model(cfg)
% Build and open the position-scan Simulink model for cogging-table study.
%
% Usage:
%   open_cogging_position_scan_model
%   open_cogging_position_scan_model(struct('scan_points', 72, 'hold_time', 0.05))

if nargin < 1
    cfg = struct();
end

motor_control_params;
cfg = local_fill_defaults(cfg, motor, control);

control.pos_ref_mode = 'scan_table';
control.pos_use_planner = false;
control.use_periodic_load = true;

control.pos_scan.start_time = cfg.start_time;
control.pos_scan.hold_time = cfg.hold_time;
control.pos_scan.points = cfg.scan_points;
control.pos_scan.theta_table = zeros(360, 1);
control.pos_scan.theta_table(1:cfg.scan_points) = cfg.theta_table(:);

control.pos_bandwidth_hz = cfg.position_bandwidth_hz;
control.pos_bandwidth_rad_s = 2 * pi * control.pos_bandwidth_hz;
control.pi_pos.Kp = control.pos_bandwidth_rad_s;
control.pi_pos.output_limit = cfg.position_output_limit;
control.pid_pos.Kp = 2 * control.pid_pos.damping * control.pos_bandwidth_rad_s;
control.pid_pos.Ki_cont = cfg.pid_pos_Ki;
control.pid_pos.Ki = cfg.pid_pos_Ki;
control.pid_pos.output_limit = cfg.position_output_limit;

control = apply_cogging_load_config(control, cfg);

simcfg.stop_time = cfg.stop_time;

assignin('base', 'motor', motor);
assignin('base', 'inverter', inverter);
assignin('base', 'control', control);
assignin('base', 'simcfg', simcfg);

build_average_inverter_foc_model;
set_param('average_inverter_foc', 'InitFcn', '');

open_system('average_inverter_foc');
open_system('average_inverter_foc/Position Scope');
open_system('average_inverter_foc/Speed Scope');
open_system('average_inverter_foc/Current dq Scope');

if strcmpi(control.pos_controller_mode, 'pid_reg3')
    open_system('average_inverter_foc/Position Ui Scope');
end

fprintf('\nPosition-scan model is ready.\n');
fprintf('Model : average_inverter_foc\n');
fprintf('Points: %d\n', cfg.scan_points);
fprintf('Hold  : %.4f s\n', cfg.hold_time);
fprintf('Stop  : %.4f s\n', cfg.stop_time);
fprintf('\nRun in MATLAB command window:\n');
fprintf('  sim(''average_inverter_foc'');\n');
end

function cfg = local_fill_defaults(cfg, motor, control)
defaults = struct();
defaults.scan_points = 180;
defaults.start_time = 0.05;
defaults.hold_time = 0.05;
defaults.position_bandwidth_scale = 0.30;
defaults.position_output_limit_scale = 0.25;
defaults.pid_pos_Ki = 0;
load_defaults = cogging_load_config(cfg);
defaults.load_base_torque = load_defaults.load_base_torque;
defaults.harmonic1 = load_defaults.harmonic1;
defaults.harmonic2 = load_defaults.harmonic2;
defaults.amp1 = load_defaults.amp1;
defaults.amp2 = load_defaults.amp2;
defaults.phase1_deg = load_defaults.phase1_deg;
defaults.phase2_deg = load_defaults.phase2_deg;

names = fieldnames(defaults);
for idx = 1:numel(names)
    name = names{idx};
    if ~isfield(cfg, name) || isempty(cfg.(name))
        cfg.(name) = defaults.(name);
    end
end

cfg.scan_points = min(360, max(24, round(cfg.scan_points)));
theta_grid = linspace(0, 2 * pi, cfg.scan_points + 1);
theta_grid(end) = [];
cfg.theta_table = theta_grid(:);

cfg.position_bandwidth_hz = control.pos_bandwidth_hz * cfg.position_bandwidth_scale;
cfg.position_output_limit = motor.speed_ref_mech_rad_s * cfg.position_output_limit_scale;
cfg.stop_time = cfg.start_time + cfg.scan_points * cfg.hold_time + 0.02;
end