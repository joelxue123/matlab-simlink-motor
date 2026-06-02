function build_current_loop_test_model(mdl)
% Build an average-voltage PMSM current-loop validation model.

if nargin < 1 || isempty(mdl)
    mdl = 'currentloop_pi_test';
end

load_system('mcbplantlib');
load_system('mcblib');

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
if exist([mdl '.slx'], 'file') == 2
    delete([mdl '.slx']);
end

new_system(mdl);
set_param(mdl, 'Solver', 'FixedStepAuto', ...
    'FixedStep', 'simcfg.Ts_plant', ...
    'StopTime', 'simcfg.stop_time', ...
    'InitFcn', 'motor_control_params');

project_root = init_project_paths(fileparts(mfilename('fullpath')));
addpath(fullfile(project_root, 'build_modules'));
addpath(fullfile(project_root, 'algorithms'));

ref_axis = evalin('base', 'validation_cfg.ref_axis');

% =====================================================================
% Row 0: reference generation at current-loop rate
% =====================================================================
y = 50;
add_block('simulink/Sources/From Workspace', [mdl '/CurrentRefWaveform'], ...
    'Position', [40 y 140 y+25], ...
    'VariableName', 'validation_ref_ts');
add_goto(mdl, 'active_ref_cmd_fast', [175 y 285 y+25], 'CurrentRefWaveform/1');

% =====================================================================
% Row 1: Axis-select current reference with saturation
% =====================================================================
y = 150;
add_from(mdl, 'active_ref_cmd_fast', [20 y+35 90 y+55]);
add_block('simulink/Sources/Constant', [mdl '/ZeroRef'], ...
    'Position', [20 y+85 90 y+105], ...
    'Value', '0');
add_block('simulink/Discontinuities/Saturation', [mdl '/Sat_id_ref'], ...
    'Position', [140 y+10 220 y+35], ...
    'UpperLimit', 'control.iq_ref_limit', ...
    'LowerLimit', '-control.iq_ref_limit');
add_block('simulink/Discontinuities/Saturation', [mdl '/Sat_iq_ref'], ...
    'Position', [140 y+60 220 y+85], ...
    'UpperLimit', 'control.iq_ref_limit', ...
    'LowerLimit', '-control.iq_ref_limit');

if strcmpi(ref_axis, 'id')
    add_line(mdl, 'From_active_ref_cmd_fast/1', 'Sat_id_ref/1');
    add_line(mdl, 'ZeroRef/1', 'Sat_iq_ref/1');
else
    add_line(mdl, 'ZeroRef/1', 'Sat_id_ref/1');
    add_line(mdl, 'From_active_ref_cmd_fast/1', 'Sat_iq_ref/1');
end

add_goto(mdl, 'id_ref', [255 y+10 335 y+30], 'Sat_id_ref/1');
add_goto(mdl, 'iq_ref', [255 y+60 335 y+80], 'Sat_iq_ref/1');

% =====================================================================
% Row 2: abc -> dq feedback path (Ts_ctrl)
% =====================================================================
y = 270;
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_ia'], ...
    'Position', [-50 y+5 0 y+25], 'OutPortSampleTime', 'simcfg.Ts_ctrl');
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_ib'], ...
    'Position', [-50 y+30 0 y+50], 'OutPortSampleTime', 'simcfg.Ts_ctrl');
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_ic'], ...
    'Position', [-50 y+55 0 y+75], 'OutPortSampleTime', 'simcfg.Ts_ctrl');
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_theta_ctrl'], ...
    'Position', [-50 y+80 0 y+100], 'OutPortSampleTime', 'simcfg.Ts_ctrl');

add_from(mdl, 'ia', [-130 y+5 -80 y+25]);
add_from(mdl, 'ib', [-130 y+30 -80 y+50]);
add_from(mdl, 'ic', [-130 y+55 -80 y+75]);
add_from(mdl, 'theta_e', [-130 y+80 -80 y+100]);
add_line(mdl, 'From_ia/1', 'RT_ia/1');
add_line(mdl, 'From_ib/1', 'RT_ib/1');
add_line(mdl, 'From_ic/1', 'RT_ic/1');
add_line(mdl, 'From_theta_e/1', 'RT_theta_ctrl/1');

create_subsystem(mdl, 'abc to dq', [120 y 300 y+120], ...
    {'ia', 'ib', 'ic', 'theta_e'}, {'id_meas', 'iq_meas'});
populate_abc2dq(mdl);
add_line(mdl, 'RT_ia/1', 'abc to dq/1');
add_line(mdl, 'RT_ib/1', 'abc to dq/2');
add_line(mdl, 'RT_ic/1', 'abc to dq/3');
add_line(mdl, 'RT_theta_ctrl/1', 'abc to dq/4');
add_goto(mdl, 'id_meas', [330 y+20 410 y+45], 'abc to dq/1');
add_goto(mdl, 'iq_meas', [330 y+70 410 y+95], 'abc to dq/2');

% =====================================================================
% Row 3: Current PI (Ts_ctrl)
% =====================================================================
y = 460;
create_subsystem(mdl, 'Current PI', [120 y 320 y+155], ...
    {'id_ref', 'iq_ref', 'id_meas', 'iq_meas', 'omega_e'}, {'vd_ref', 'vq_ref'});
populate_current_pi(mdl);
add_from(mdl, 'id_ref', [20 y+5 80 y+25]);
add_from(mdl, 'iq_ref', [20 y+30 80 y+50]);
add_from(mdl, 'id_meas', [20 y+55 80 y+75]);
add_from(mdl, 'iq_meas', [20 y+80 80 y+100]);

add_from(mdl, 'omega_e', [-90 y+110 -40 y+130]);
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_omega_ctrl'], ...
    'Position', [-30 y+110 40 y+130], ...
    'OutPortSampleTime', 'simcfg.Ts_ctrl');
add_line(mdl, 'From_omega_e/1', 'RT_omega_ctrl/1');

add_line(mdl, 'From_id_ref/1', 'Current PI/1');
add_line(mdl, 'From_iq_ref/1', 'Current PI/2');
add_line(mdl, 'From_id_meas/1', 'Current PI/3');
add_line(mdl, 'From_iq_meas/1', 'Current PI/4');
add_line(mdl, 'RT_omega_ctrl/1', 'Current PI/5');
add_goto(mdl, 'vd_ref', [360 y+30 430 y+55], 'Current PI/1');
add_goto(mdl, 'vq_ref', [360 y+95 430 y+120], 'Current PI/2');

% =====================================================================
% Row 4: dq -> abc and modulation (Ts_ctrl)
% =====================================================================
y = 680;
create_subsystem(mdl, 'dq to abc', [120 y 300 y+120], ...
    {'vd_ref', 'vq_ref', 'theta_e', 'Vdc'}, {'da', 'db', 'dc'});
populate_dq2abc(mdl);
add_from(mdl, 'vd_ref', [20 y+5 80 y+25]);
add_from(mdl, 'vq_ref', [20 y+35 80 y+55]);

add_from(mdl, 'theta_e_2', [-70 y+70 -20 y+90]);
set_param([mdl '/From_theta_e_2'], 'GotoTag', 'theta_e');
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_theta_dq2abc'], ...
    'Position', [-15 y+70 55 y+90], ...
    'OutPortSampleTime', 'simcfg.Ts_ctrl');
add_line(mdl, 'From_theta_e_2/1', 'RT_theta_dq2abc/1');

add_block('simulink/Sources/Constant', [mdl '/Vdc_dq2abc'], ...
    'Position', [20 y+100 80 y+120], 'Value', 'inverter.Vdc');
add_line(mdl, 'From_vd_ref/1', 'dq to abc/1');
add_line(mdl, 'From_vq_ref/1', 'dq to abc/2');
add_line(mdl, 'RT_theta_dq2abc/1', 'dq to abc/3');
add_line(mdl, 'Vdc_dq2abc/1', 'dq to abc/4');
add_goto(mdl, 'da', [330 y 410 y+20], 'dq to abc/1');
add_goto(mdl, 'db', [330 y+35 410 y+55], 'dq to abc/2');
add_goto(mdl, 'dc', [330 y+70 410 y+90], 'dq to abc/3');

% =====================================================================
% Row 5: Average inverter
% =====================================================================
y = 850;
add_from(mdl, 'da', [20 y 80 y+20]);
add_from(mdl, 'db', [20 y+40 80 y+60]);
add_from(mdl, 'dc', [20 y+80 80 y+100]);

add_block('simulink/Signal Routing/Mux', [mdl '/Mux_Mod'], ...
    'Position', [120 y+10 125 y+90], 'Inputs', '3');
add_line(mdl, 'From_da/1', 'Mux_Mod/1');
add_line(mdl, 'From_db/1', 'Mux_Mod/2');
add_line(mdl, 'From_dc/1', 'Mux_Mod/3');

add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_mod_to_plant'], ...
    'Position', [150 y+30 220 y+70], ...
    'OutPortSampleTime', 'simcfg.Ts_plant');
add_line(mdl, 'Mux_Mod/1', 'RT_mod_to_plant/1');

add_block('simulink/Sources/Constant', [mdl '/Vdc'], ...
    'Position', [220 y+110 270 y+130], 'Value', 'inverter.Vdc');

add_block('mcbplantlib/Average-Value Inverter', [mdl '/Average Inverter'], ...
    'Position', [380 y 540 y+90]);
add_line(mdl, 'RT_mod_to_plant/1', 'Average Inverter/1');
add_line(mdl, 'Vdc/1', 'Average Inverter/2');
add_goto(mdl, 'Vabc_out', [570 y+20 650 y+45], 'Average Inverter/1');

% =====================================================================
% Row 6: PMSM plant
% =====================================================================
y = 1020;
add_block('simulink/Sources/Constant', [mdl '/LoadTorque'], ...
    'Position', [40 y 110 y+25], ...
    'Value', 'motor.load_torque');
add_goto(mdl, 'T_load', [145 y 230 y+25], 'LoadTorque/1');

add_from(mdl, 'T_load', [20 y 70 y+20]);
add_from(mdl, 'Vabc_out', [20 y+60 70 y+80]);
add_block('mcblib/Electrical Systems/Motors/Surface Mount PMSM', ...
    [mdl '/Surface Mount PMSM'], 'Position', [120 y+40 330 y+200]);
set_param([mdl '/Surface Mount PMSM'], ...
    'port_config', 'Torque', ...
    'sim_type', 'Discrete', ...
    'Ts', 'simcfg.Ts_plant', ...
    'P', 'motor.pole_pairs', ...
    'Rs', 'motor.Rs', ...
    'Ldq_', 'motor.Ld', ...
    'lambda_pm', 'motor.psi_f', ...
    'mechanical', '[motor.J, motor.B, 0]', ...
    'idq0', '[0 0]', ...
    'theta_init', '0', ...
    'omega_init', '0');
add_line(mdl, 'From_T_load/1', 'Surface Mount PMSM/1');
add_line(mdl, 'From_Vabc_out/1', 'Surface Mount PMSM/2');

add_block('simulink/Signal Routing/Bus Selector', [mdl '/BusSel_Info'], ...
    'Position', [360 y+40 365 y+80], 'OutputSignals', 'MtrPos');
add_line(mdl, 'Surface Mount PMSM/1', 'BusSel_Info/1');

add_block('simulink/Math Operations/Gain', [mdl '/pos2theta_e'], ...
    'Position', [400 y+45 460 y+75], 'Gain', 'motor.pole_pairs');
add_line(mdl, 'BusSel_Info/1', 'pos2theta_e/1');
add_goto(mdl, 'theta_e', [490 y+45 560 y+75], 'pos2theta_e/1');

add_block('simulink/Signal Routing/Demux', [mdl '/Demux_Iabc'], ...
    'Position', [360 y+95 365 y+175], 'Outputs', '3');
add_line(mdl, 'Surface Mount PMSM/2', 'Demux_Iabc/1');
add_goto(mdl, 'ia', [400 y+80 460 y+100], 'Demux_Iabc/1');
add_goto(mdl, 'ib', [400 y+115 460 y+135], 'Demux_Iabc/2');
add_goto(mdl, 'ic', [400 y+150 460 y+170], 'Demux_Iabc/3');

add_goto(mdl, 'w_meas', [360 y+190 440 y+210], 'Surface Mount PMSM/3');

add_block('simulink/Math Operations/Gain', [mdl '/wm2we'], ...
    'Position', [360 y+230 420 y+260], 'Gain', 'motor.pole_pairs');
add_line(mdl, 'Surface Mount PMSM/3', 'wm2we/1');
add_goto(mdl, 'omega_e', [460 y+230 530 y+255], 'wm2we/1');

% =====================================================================
% Row 7: Scopes and logs
% =====================================================================
y = 1290;
add_from(mdl, 'id_ref_scope', [20 y 70 y+20]);
set_param([mdl '/From_id_ref_scope'], 'GotoTag', 'id_ref');
add_from(mdl, 'iq_ref_scope', [20 y+30 70 y+50]);
set_param([mdl '/From_iq_ref_scope'], 'GotoTag', 'iq_ref');
add_from(mdl, 'id_meas_scope', [20 y+60 70 y+80]);
set_param([mdl '/From_id_meas_scope'], 'GotoTag', 'id_meas');
add_from(mdl, 'iq_meas_scope', [20 y+90 70 y+110]);
set_param([mdl '/From_iq_meas_scope'], 'GotoTag', 'iq_meas');

add_block('simulink/Signal Routing/Mux', [mdl '/Mux_Current'], ...
    'Position', [110 y 115 y+110], 'Inputs', '4');
add_line(mdl, 'From_id_ref_scope/1', 'Mux_Current/1');
add_line(mdl, 'From_iq_ref_scope/1', 'Mux_Current/2');
add_line(mdl, 'From_id_meas_scope/1', 'Mux_Current/3');
add_line(mdl, 'From_iq_meas_scope/1', 'Mux_Current/4');
add_block('simulink/Sinks/Scope', [mdl '/Current Scope'], ...
    'Position', [150 y+15 200 y+85], 'NumInputPorts', '1');
add_line(mdl, 'Mux_Current/1', 'Current Scope/1');

add_from(mdl, 'vd_ref_scope', [260 y 310 y+20]);
set_param([mdl '/From_vd_ref_scope'], 'GotoTag', 'vd_ref');
add_from(mdl, 'vq_ref_scope', [260 y+30 310 y+50]);
set_param([mdl '/From_vq_ref_scope'], 'GotoTag', 'vq_ref');
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_Vdq'], ...
    'Position', [350 y 355 y+50], 'Inputs', '2');
add_line(mdl, 'From_vd_ref_scope/1', 'Mux_Vdq/1');
add_line(mdl, 'From_vq_ref_scope/1', 'Mux_Vdq/2');
add_block('simulink/Sinks/Scope', [mdl '/Voltage Scope'], ...
    'Position', [390 y+5 440 y+45], 'NumInputPorts', '1');
add_line(mdl, 'Mux_Vdq/1', 'Voltage Scope/1');

add_block('simulink/Sinks/To Workspace', [mdl '/Log_id_ref'], ...
    'Position', [120 y+120 200 y+145], ...
    'VariableName', 'log_id_ref', 'SaveFormat', 'Timeseries');
add_from(mdl, 'id_ref_log', [20 y+120 70 y+140]);
set_param([mdl '/From_id_ref_log'], 'GotoTag', 'id_ref');
add_line(mdl, 'From_id_ref_log/1', 'Log_id_ref/1');

add_block('simulink/Sinks/To Workspace', [mdl '/Log_iq_ref'], ...
    'Position', [120 y+150 200 y+175], ...
    'VariableName', 'log_iq_ref', 'SaveFormat', 'Timeseries');
add_from(mdl, 'iq_ref_log', [20 y+150 70 y+170]);
set_param([mdl '/From_iq_ref_log'], 'GotoTag', 'iq_ref');
add_line(mdl, 'From_iq_ref_log/1', 'Log_iq_ref/1');

add_block('simulink/Sinks/To Workspace', [mdl '/Log_iq_meas'], ...
    'Position', [120 y+180 200 y+205], ...
    'VariableName', 'log_iq_meas', 'SaveFormat', 'Timeseries');
add_from(mdl, 'iq_meas_log', [20 y+180 70 y+200]);
set_param([mdl '/From_iq_meas_log'], 'GotoTag', 'iq_meas');
add_line(mdl, 'From_iq_meas_log/1', 'Log_iq_meas/1');

add_block('simulink/Sinks/To Workspace', [mdl '/Log_id_meas'], ...
    'Position', [120 y+210 200 y+235], ...
    'VariableName', 'log_id_meas', 'SaveFormat', 'Timeseries');
add_from(mdl, 'id_meas_log', [20 y+210 70 y+230]);
set_param([mdl '/From_id_meas_log'], 'GotoTag', 'id_meas');
add_line(mdl, 'From_id_meas_log/1', 'Log_id_meas/1');

add_block('simulink/Sinks/To Workspace', [mdl '/Log_vd_ref'], ...
    'Position', [120 y+240 200 y+265], ...
    'VariableName', 'log_vd_ref', 'SaveFormat', 'Timeseries');
add_from(mdl, 'vd_ref_log', [20 y+240 70 y+260]);
set_param([mdl '/From_vd_ref_log'], 'GotoTag', 'vd_ref');
add_line(mdl, 'From_vd_ref_log/1', 'Log_vd_ref/1');

add_block('simulink/Sinks/To Workspace', [mdl '/Log_vq_ref'], ...
    'Position', [120 y+270 200 y+295], ...
    'VariableName', 'log_vq_ref', 'SaveFormat', 'Timeseries');
add_from(mdl, 'vq_ref_log', [20 y+270 70 y+290]);
set_param([mdl '/From_vq_ref_log'], 'GotoTag', 'vq_ref');
add_line(mdl, 'From_vq_ref_log/1', 'Log_vq_ref/1');

add_block('simulink/Sinks/To Workspace', [mdl '/Log_wm'], ...
    'Position', [120 y+300 200 y+325], ...
    'VariableName', 'log_wm', 'SaveFormat', 'Timeseries');
add_from(mdl, 'w_meas_log', [20 y+300 70 y+320]);
set_param([mdl '/From_w_meas_log'], 'GotoTag', 'w_meas');
add_line(mdl, 'From_w_meas_log/1', 'Log_wm/1');

save_system(mdl, fullfile(pwd, [mdl '.slx']));
fprintf('Current-loop validation model saved: %s.slx\n', mdl);
fprintf('Logged signals: log_id_ref, log_iq_ref, log_id_meas, log_iq_meas, log_vd_ref, log_vq_ref, log_wm\n');
end