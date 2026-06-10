function mdl_path = build_model_following_position_ref_model(cfg)
% Build a discrete model-following position-reference demo.
%
% Reference-model state equation:
%
%   pos_dot = vel
%   vel_dot = sat_acc(wn^2 * (pos_cmd - pos) - 2*zeta*wn*vel)
%
% The model outputs pos_ref, vel_ref, and acc_ref directly from its states,
% avoiding continuous derivative blocks. A simple servo-axis proxy compares:
%
%   1. raw position step into a position loop
%   2. model-followed position only
%   3. model-followed position plus velocity feedforward

if nargin < 1
    cfg = struct();
end
cfg = local_default_cfg(cfg);

study_dir = fileparts(mfilename('fullpath'));
mdl = 'model_following_position_ref';
mdl_path = fullfile(study_dir, [mdl '.slx']);

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
if exist(mdl_path, 'file')
    delete(mdl_path);
end

assignin('base', 'mf_cfg', cfg);

new_system(mdl);
if cfg.open_model
    open_system(mdl);
end
set_param(mdl, ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', 'mf_cfg.Ts_ref', ...
    'StopTime', 'mf_cfg.stop_time', ...
    'SaveFormat', 'StructureWithTime');

% =====================================================================
% Row 0: command and discrete model-following reference generator
% =====================================================================
add_block('simulink/Sources/Step', [mdl '/Position Step Cmd'], ...
    'Position', [40 80 110 110], ...
    'Time', 'mf_cfg.step_time', ...
    'Before', '0', ...
    'After', 'mf_cfg.step_rad', ...
    'SampleTime', 'mf_cfg.Ts_ref');
add_block('simulink/Sources/Constant', [mdl '/RefModelParams'], ...
    'Position', [40 150 240 175], ...
    'Value', 'mf_cfg.ref_params');

add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl '/Discrete Reference Model'], ...
    'Position', [300 55 520 185]);
local_set_matlab_function([mdl '/Discrete Reference Model'], local_reference_model_script());

add_line(mdl, 'Position Step Cmd/1', 'Discrete Reference Model/1');
add_line(mdl, 'RefModelParams/1', 'Discrete Reference Model/2');

local_to_workspace(mdl, 'cmd_raw_log', [165 35 255 60], 'Position Step Cmd/1');
local_to_workspace(mdl, 'pos_ref_log', [580 45 680 70], 'Discrete Reference Model/1');
local_to_workspace(mdl, 'vel_ref_log', [580 75 680 100], 'Discrete Reference Model/2');
local_to_workspace(mdl, 'acc_ref_log', [580 105 680 130], 'Discrete Reference Model/3');
local_to_workspace(mdl, 'acc_unsat_log', [580 135 680 160], 'Discrete Reference Model/4');
local_to_workspace(mdl, 'acc_sat_flag_log', [580 165 695 190], 'Discrete Reference Model/5');

% =====================================================================
% Row 1: simplified servo-axis proxies
% =====================================================================
add_block('simulink/Sources/Constant', [mdl '/AxisParams'], ...
    'Position', [40 275 240 300], ...
    'Value', 'mf_cfg.axis_params');
add_block('simulink/Sources/Constant', [mdl '/ZeroVelFF'], ...
    'Position', [40 325 120 350], ...
    'Value', '0');

add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl '/Axis Raw Step P'], ...
    'Position', [300 230 510 310]);
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl '/Axis MF Position Only'], ...
    'Position', [300 355 510 435]);
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl '/Axis MF Velocity FF'], ...
    'Position', [300 480 510 560]);
local_set_matlab_function([mdl '/Axis Raw Step P'], local_axis_proxy_script());
local_set_matlab_function([mdl '/Axis MF Position Only'], local_axis_proxy_script());
local_set_matlab_function([mdl '/Axis MF Velocity FF'], local_axis_proxy_script());

% Raw step: no velocity feedforward.
add_line(mdl, 'Position Step Cmd/1', 'Axis Raw Step P/1', 'autorouting', 'on');
add_line(mdl, 'ZeroVelFF/1', 'Axis Raw Step P/2', 'autorouting', 'on');
add_line(mdl, 'AxisParams/1', 'Axis Raw Step P/3', 'autorouting', 'on');

% Model-followed position only: same shaped position, no velocity FF.
add_line(mdl, 'Discrete Reference Model/1', 'Axis MF Position Only/1', 'autorouting', 'on');
add_line(mdl, 'ZeroVelFF/1', 'Axis MF Position Only/2', 'autorouting', 'on');
add_line(mdl, 'AxisParams/1', 'Axis MF Position Only/3', 'autorouting', 'on');

% Model-followed position plus velocity feedforward.
add_line(mdl, 'Discrete Reference Model/1', 'Axis MF Velocity FF/1', 'autorouting', 'on');
add_line(mdl, 'Discrete Reference Model/2', 'Axis MF Velocity FF/2', 'autorouting', 'on');
add_line(mdl, 'AxisParams/1', 'Axis MF Velocity FF/3', 'autorouting', 'on');

local_to_workspace(mdl, 'axis_raw_log', [575 235 675 260], 'Axis Raw Step P/1');
local_to_workspace(mdl, 'axis_mf_noff_log', [575 360 700 385], 'Axis MF Position Only/1');
local_to_workspace(mdl, 'axis_mf_vff_log', [575 485 700 510], 'Axis MF Velocity FF/1');
local_to_workspace(mdl, 'speed_cmd_raw_log', [575 270 700 295], 'Axis Raw Step P/3');
local_to_workspace(mdl, 'speed_cmd_mf_noff_log', [575 395 730 420], 'Axis MF Position Only/3');
local_to_workspace(mdl, 'speed_cmd_mf_vff_log', [575 520 730 545], 'Axis MF Velocity FF/3');

% =====================================================================
% Scopes
% =====================================================================
add_block('simulink/Signal Routing/Mux', [mdl '/Mux Reference Signals'], ...
    'Position', [770 50 775 180], ...
    'Inputs', '5');
add_block('simulink/Sinks/Scope', [mdl '/Reference Signals Scope'], ...
    'Position', [830 65 1040 165], ...
    'NumInputPorts', '1');
add_line(mdl, 'Position Step Cmd/1', 'Mux Reference Signals/1', 'autorouting', 'on');
add_line(mdl, 'Discrete Reference Model/1', 'Mux Reference Signals/2', 'autorouting', 'on');
add_line(mdl, 'Discrete Reference Model/2', 'Mux Reference Signals/3', 'autorouting', 'on');
add_line(mdl, 'Discrete Reference Model/3', 'Mux Reference Signals/4', 'autorouting', 'on');
add_line(mdl, 'Discrete Reference Model/4', 'Mux Reference Signals/5', 'autorouting', 'on');
add_line(mdl, 'Mux Reference Signals/1', 'Reference Signals Scope/1');

add_block('simulink/Signal Routing/Mux', [mdl '/Mux Axis Compare'], ...
    'Position', [770 320 775 470], ...
    'Inputs', '4');
add_block('simulink/Sinks/Scope', [mdl '/Axis Compare Scope'], ...
    'Position', [830 340 1040 450], ...
    'NumInputPorts', '1');
add_line(mdl, 'Position Step Cmd/1', 'Mux Axis Compare/1', 'autorouting', 'on');
add_line(mdl, 'Axis Raw Step P/1', 'Mux Axis Compare/2', 'autorouting', 'on');
add_line(mdl, 'Axis MF Position Only/1', 'Mux Axis Compare/3', 'autorouting', 'on');
add_line(mdl, 'Axis MF Velocity FF/1', 'Mux Axis Compare/4', 'autorouting', 'on');
add_line(mdl, 'Mux Axis Compare/1', 'Axis Compare Scope/1');

add_block('simulink/Signal Routing/Mux', [mdl '/Mux Speed Commands'], ...
    'Position', [770 540 775 650], ...
    'Inputs', '3');
add_block('simulink/Sinks/Scope', [mdl '/Speed Command Scope'], ...
    'Position', [830 555 1040 635], ...
    'NumInputPorts', '1');
add_line(mdl, 'Axis Raw Step P/3', 'Mux Speed Commands/1', 'autorouting', 'on');
add_line(mdl, 'Axis MF Position Only/3', 'Mux Speed Commands/2', 'autorouting', 'on');
add_line(mdl, 'Axis MF Velocity FF/3', 'Mux Speed Commands/3', 'autorouting', 'on');
add_line(mdl, 'Mux Speed Commands/1', 'Speed Command Scope/1');

add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/State Equation Note'], ...
    'Position', [40 610 590 685]);
set_param([mdl '/State Equation Note'], 'MaskDisplay', ...
    sprintf('disp(''pos_dot = vel;  vel_dot = sat_acc(wn^2*(cmd-pos)-2*zeta*wn*vel)'')'));

save_system(mdl, mdl_path);
fprintf('Model-following position reference model saved: %s\n', mdl_path);
end

function cfg = local_default_cfg(cfg)
defaults = struct();
defaults.Ts_ref = 1e-4;
defaults.stop_time = 0.45;
defaults.step_time = 0.02;
defaults.step_rad = 2 * pi;

% Reference model knobs.
defaults.ref_bandwidth_hz = 12;
defaults.ref_damping = 1.0;
defaults.max_acc_ref_rad_s2 = 8000;
defaults.max_vel_ref_rad_s = 220;

% Simplified axis: speed command = Kp_pos * position_error + Kff * vel_ref.
defaults.axis_pos_kp = 2 * pi * 35;
defaults.axis_vel_ff_gain = 1.0;
defaults.axis_speed_tau = 0.008;
defaults.axis_speed_limit_rad_s = 300;

defaults.open_model = false;

fields = fieldnames(defaults);
for idx = 1:numel(fields)
    name = fields{idx};
    if ~isfield(cfg, name)
        cfg.(name) = defaults.(name);
    end
end

cfg.wn_ref = 2 * pi * cfg.ref_bandwidth_hz;
cfg.ref_params = [
    cfg.wn_ref
    cfg.ref_damping
    cfg.Ts_ref
    cfg.max_acc_ref_rad_s2
    cfg.max_vel_ref_rad_s
    ];

cfg.axis_params = [
    cfg.axis_pos_kp
    cfg.axis_vel_ff_gain
    cfg.axis_speed_tau
    cfg.Ts_ref
    cfg.axis_speed_limit_rad_s
    ];
end

function local_to_workspace(mdl, name, pos, src)
add_block('simulink/Sinks/To Workspace', [mdl '/' name], ...
    'Position', pos, ...
    'VariableName', name, ...
    'SaveFormat', 'StructureWithTime');
add_line(mdl, src, [name '/1'], 'autorouting', 'on');
end

function local_set_matlab_function(block_path, script)
rt = sfroot;
chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', block_path);
chart.Script = char(script);
end

function script = local_reference_model_script()
script = strjoin([
    "function [pos_ref, vel_ref, acc_ref, acc_unsat, sat_flag] = reference_model(pos_cmd, params)"
    "% Discrete saturated second-order reference model."
    "% params = [wn, zeta, Ts, acc_limit, vel_limit]"
    "persistent pos vel"
    "if isempty(pos)"
    "    pos = 0;"
    "    vel = 0;"
    "end"
    ""
    "wn = params(1);"
    "zeta = params(2);"
    "Ts = params(3);"
    "acc_limit = abs(params(4));"
    "vel_limit = abs(params(5));"
    ""
    "acc_unsat = wn * wn * (pos_cmd - pos) - 2 * zeta * wn * vel;"
    "acc_limited = min(max(acc_unsat, -acc_limit), acc_limit);"
    ""
    "vel_candidate = vel + acc_limited * Ts;"
    "vel_next = min(max(vel_candidate, -vel_limit), vel_limit);"
    "acc_ref = (vel_next - vel) / Ts;"
    "pos_next = pos + vel_next * Ts;"
    ""
    "sat_flag = double(abs(acc_unsat - acc_ref) > 1e-9 || abs(vel_candidate - vel_next) > 1e-9);"
    ""
    "pos = pos_next;"
    "vel = vel_next;"
    "pos_ref = pos;"
    "vel_ref = vel;"
    "end"
    ], newline);
end

function script = local_axis_proxy_script()
script = strjoin([
    "function [theta_out, w_out, speed_cmd] = axis_proxy(pos_cmd, vel_ff, params)"
    "% Discrete position loop + velocity feedforward + first-order speed loop."
    "% params = [pos_kp, vel_ff_gain, speed_tau, Ts, speed_limit]"
    "persistent theta w"
    "if isempty(theta)"
    "    theta = 0;"
    "    w = 0;"
    "end"
    ""
    "pos_kp = params(1);"
    "vel_ff_gain = params(2);"
    "speed_tau = max(params(3), eps);"
    "Ts = params(4);"
    "speed_limit = abs(params(5));"
    ""
    "speed_cmd_unsat = pos_kp * (pos_cmd - theta) + vel_ff_gain * vel_ff;"
    "speed_cmd = min(max(speed_cmd_unsat, -speed_limit), speed_limit);"
    "w = w + (speed_cmd - w) * Ts / speed_tau;"
    "theta = theta + w * Ts;"
    ""
    "theta_out = theta;"
    "w_out = w;"
    "end"
    ], newline);
end
