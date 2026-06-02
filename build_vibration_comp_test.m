
function build_vibration_comp_test()
% Build a vibration-compensation test model with periodic compressor-like load.
%
% The model keeps a constant speed reference, injects a mechanical-angle-
% synchronous load ripple, and evaluates whether the learned feedforward term
% reduces speed ripple.

mdl = 'vibration_comp_test';

load_system('mcbplantlib');
load_system('mcblib');

if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end

new_system(mdl);
set_param(mdl, 'Solver', 'FixedStepAuto', ...
    'FixedStep', 'simcfg.Ts_plant', ...
    'StopTime', 'control.vib.test_stop_time', ...
    'InitFcn', 'motor_control_params');

control = evalin('base', 'control');

script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, 'build_modules'));
addpath(fullfile(script_dir, 'algorithms'));

% =====================================================================
% Row 0: Constant speed command profile
% =====================================================================
y = 50;
add_block('simulink/Sources/Step', [mdl '/WrefStep'], ...
    'Position', [40 y 100 y+25], ...
    'Time', '0.02', ...
    'Before', '0', ...
    'After', 'motor.speed_ref_mech_rad_s', ...
    'SampleTime', 'simcfg.Ts_speed');
add_goto(mdl, 'w_ref', [140 y 210 y+25], 'WrefStep/1');

% =====================================================================
% Row 1: Speed PI using w_meas feedback for a stable baseline loop
% =====================================================================
y = 150;
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_wm_to_speed'], ...
    'Position', [20 y+35 80 y+55], ...
    'OutPortSampleTime', 'simcfg.Ts_speed');
create_subsystem(mdl, 'Speed PI', [120 y 300 y+90], ...
    {'w_ref', 'w_meas'}, {'iq_ref'});
populate_speed_pi(mdl);
add_from(mdl, 'w_ref', [20 y+5 80 y+25]);
add_from(mdl, 'w_meas_speed', [-60 y+35 -10 y+55]);
set_param([mdl '/From_w_meas_speed'], 'GotoTag', 'w_meas');
add_line(mdl, 'From_w_meas_speed/1', 'RT_wm_to_speed/1');
add_line(mdl, 'From_w_ref/1', 'Speed PI/1');
add_line(mdl, 'RT_wm_to_speed/1', 'Speed PI/2');
add_goto(mdl, 'iq_ref_base', [330 y+20 420 y+45], 'Speed PI/1');

% =====================================================================
% Row 1b: Vibration compensation path at speed-loop rate
%   control.vib.mode: 'none' | 'online' | 'offline'
% =====================================================================
y = 280;
if strcmp(control.vib.mode, 'online')
    add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_theta_to_vib'], ...
        'Position', [20 y+5 80 y+25], ...
        'OutPortSampleTime', 'simcfg.Ts_speed');
    add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_wm_to_vib'], ...
        'Position', [20 y+40 80 y+60], ...
        'OutPortSampleTime', 'simcfg.Ts_speed');
    create_subsystem(mdl, 'Vibration Compensation', [120 y 340 y+155], ...
        {'theta_meas', 'iq_ref_base', 'w_ref', 'w_meas', 't_now'}, ...
        {'iq_ref_cmd', 'iq_ff', 'learn_active'});
    populate_vibration_comp(mdl);
    add_from(mdl, 'theta_meas_vib', [-60 y+5 -10 y+25]);
    set_param([mdl '/From_theta_meas_vib'], 'GotoTag', 'theta_meas');
    add_from(mdl, 'w_meas_vib', [-60 y+40 -10 y+60]);
    set_param([mdl '/From_w_meas_vib'], 'GotoTag', 'w_meas');
    add_from(mdl, 'iq_ref_base_vib', [20 y+75 80 y+95]);
    set_param([mdl '/From_iq_ref_base_vib'], 'GotoTag', 'iq_ref_base');
    add_from(mdl, 'w_ref_vib', [20 y+110 80 y+130]);
    set_param([mdl '/From_w_ref_vib'], 'GotoTag', 'w_ref');
    add_block('simulink/Sources/Clock', [mdl '/SimTime'], ...
        'Position', [20 y+145 80 y+165]);
    add_line(mdl, 'From_theta_meas_vib/1', 'RT_theta_to_vib/1');
    add_line(mdl, 'From_w_meas_vib/1', 'RT_wm_to_vib/1');
    add_line(mdl, 'RT_theta_to_vib/1', 'Vibration Compensation/1');
    add_line(mdl, 'From_iq_ref_base_vib/1', 'Vibration Compensation/2');
    add_line(mdl, 'From_w_ref_vib/1', 'Vibration Compensation/3');
    add_line(mdl, 'RT_wm_to_vib/1', 'Vibration Compensation/4');
    add_line(mdl, 'SimTime/1', 'Vibration Compensation/5');
    add_goto(mdl, 'iq_ref_cmd', [380 y+20 470 y+45], 'Vibration Compensation/1');
    add_goto(mdl, 'iq_ff', [380 y+70 450 y+95], 'Vibration Compensation/2');
    add_goto(mdl, 'learn_active', [380 y+115 480 y+140], 'Vibration Compensation/3');
elseif strcmp(control.vib.mode, 'offline')
    add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_theta_to_vib'], ...
        'Position', [20 y+5 80 y+25], ...
        'OutPortSampleTime', 'simcfg.Ts_speed');
    add_block('simulink/Sources/Clock', [mdl '/SimTime'], ...
        'Position', [20 y+145 80 y+165]);
    create_subsystem(mdl, 'Vibration FF Lookup', [120 y 340 y+155], ...
        {'theta_meas', 'iq_ref_base', 't_now'}, {'iq_ref_cmd', 'iq_ff', 'learn_active'});
    populate_vibration_ff_lookup(mdl);
    add_from(mdl, 'theta_meas_vib', [-60 y+5 -10 y+25]);
    set_param([mdl '/From_theta_meas_vib'], 'GotoTag', 'theta_meas');
    add_from(mdl, 'iq_ref_base_vib', [20 y+75 80 y+95]);
    set_param([mdl '/From_iq_ref_base_vib'], 'GotoTag', 'iq_ref_base');
    add_line(mdl, 'From_theta_meas_vib/1', 'RT_theta_to_vib/1');
    add_line(mdl, 'RT_theta_to_vib/1', 'Vibration FF Lookup/1');
    add_line(mdl, 'From_iq_ref_base_vib/1', 'Vibration FF Lookup/2');
    add_line(mdl, 'SimTime/1', 'Vibration FF Lookup/3');
    add_goto(mdl, 'iq_ref_cmd', [380 y+20 470 y+45], 'Vibration FF Lookup/1');
    add_goto(mdl, 'iq_ff', [380 y+70 450 y+95], 'Vibration FF Lookup/2');
    add_goto(mdl, 'learn_active', [380 y+115 480 y+140], 'Vibration FF Lookup/3');
else
    add_from(mdl, 'iq_ref_base_passthrough', [20 y+20 80 y+40]);
    set_param([mdl '/From_iq_ref_base_passthrough'], 'GotoTag', 'iq_ref_base');
    add_goto(mdl, 'iq_ref_cmd', [120 y+20 210 y+45], 'From_iq_ref_base_passthrough/1');
    add_block('simulink/Sources/Constant', [mdl '/IqffZero'], ...
        'Position', [20 y+70 80 y+90], 'Value', '0');
    add_block('simulink/Sources/Constant', [mdl '/LearnZero'], ...
        'Position', [20 y+110 80 y+130], 'Value', '0');
    add_goto(mdl, 'iq_ff', [120 y+65 190 y+90], 'IqffZero/1');
    add_goto(mdl, 'learn_active', [120 y+105 220 y+130], 'LearnZero/1');
end

% =====================================================================
% Row 2: Current reference and saturation
% =====================================================================
y = 480;
create_subsystem(mdl, 'Current Ref', [120 y 300 y+65], ...
    {'iq_ref_cmd'}, {'id_ref', 'iq_ref'});
populate_current_ref(mdl);
add_from(mdl, 'iq_ref_cmd_fast', [20 y+10 80 y+30]);
set_param([mdl '/From_iq_ref_cmd_fast'], 'GotoTag', 'iq_ref_cmd');
add_line(mdl, 'From_iq_ref_cmd_fast/1', 'Current Ref/1');
add_goto(mdl, 'id_ref', [330 y 410 y+20], 'Current Ref/1');
add_goto(mdl, 'iq_ref', [330 y+35 410 y+55], 'Current Ref/2');

% =====================================================================
% Row 3: abc -> dq transform
% =====================================================================
y = 600;
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
% Row 4: Current PI
% =====================================================================
y = 780;
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
% Row 5: dq -> abc
% =====================================================================
y = 1000;
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
% Row 6: Inverter
% =====================================================================
y = 1160;
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
% Row 7: PMSM plant + periodic load disturbance
% =====================================================================
y = 1340;
add_from(mdl, 'theta_meas_load', [20 y 80 y+20]);
set_param([mdl '/From_theta_meas_load'], 'GotoTag', 'theta_meas');
add_block('simulink/Discrete/Unit Delay', [mdl '/UD_theta_load'], ...
    'Position', [100 y 150 y+20], ...
    'InitialCondition', '0', ...
    'SampleTime', 'simcfg.Ts_plant');
add_line(mdl, 'From_theta_meas_load/1', 'UD_theta_load/1');
create_subsystem(mdl, 'Periodic Load', [190 y-10 390 y+70], ...
    {'theta_meas'}, {'T_load'});
populate_periodic_load(mdl);
add_line(mdl, 'UD_theta_load/1', 'Periodic Load/1');
add_goto(mdl, 'T_load', [420 y 490 y+20], 'Periodic Load/1');

add_from(mdl, 'T_load', [20 y+100 80 y+120]);
add_from(mdl, 'Vabc_out', [20 y+160 80 y+180]);
add_block('mcblib/Electrical Systems/Motors/Surface Mount PMSM', ...
    [mdl '/Surface Mount PMSM'], 'Position', [120 y+80 330 y+240]);
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
    'Position', [360 y+80 365 y+120], 'OutputSignals', 'MtrPos');
add_line(mdl, 'Surface Mount PMSM/1', 'BusSel_Info/1');
add_block('simulink/Math Operations/Gain', [mdl '/pos2theta_e'], ...
    'Position', [400 y+85 460 y+115], 'Gain', 'motor.pole_pairs');
add_line(mdl, 'BusSel_Info/1', 'pos2theta_e/1');
add_goto(mdl, 'theta_e', [490 y+85 560 y+115], 'pos2theta_e/1');
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/PosUnwrap'], ...
    'Position', [400 y+45 490 y+75]);
populate_pos_unwrap(mdl);
add_line(mdl, 'BusSel_Info/1', 'PosUnwrap/1');
add_goto(mdl, 'theta_meas', [520 y+45 600 y+75], 'PosUnwrap/1');
add_block('simulink/Signal Routing/Demux', [mdl '/Demux_Iabc'], ...
    'Position', [360 y+135 365 y+215], 'Outputs', '3');
add_line(mdl, 'Surface Mount PMSM/2', 'Demux_Iabc/1');
add_goto(mdl, 'ia', [400 y+120 460 y+140], 'Demux_Iabc/1');
add_goto(mdl, 'ib', [400 y+155 460 y+175], 'Demux_Iabc/2');
add_goto(mdl, 'ic', [400 y+190 460 y+210], 'Demux_Iabc/3');
add_goto(mdl, 'w_meas', [360 y+230 440 y+250], 'Surface Mount PMSM/3');
add_block('simulink/Math Operations/Gain', [mdl '/wm2we'], ...
    'Position', [360 y+270 420 y+300], 'Gain', 'motor.pole_pairs');
add_line(mdl, 'Surface Mount PMSM/3', 'wm2we/1');
add_goto(mdl, 'omega_e', [460 y+270 530 y+295], 'wm2we/1');

% =====================================================================
% Row 8: Scopes and logs
% =====================================================================
y = 1720;
add_from(mdl, 'w_ref_scope', [20 y 80 y+20]);
set_param([mdl '/From_w_ref_scope'], 'GotoTag', 'w_ref');
add_from(mdl, 'w_meas_scope', [20 y+30 80 y+50]);
set_param([mdl '/From_w_meas_scope'], 'GotoTag', 'w_meas');
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_Speed'], ...
    'Position', [120 y 125 y+50], 'Inputs', '2');
add_line(mdl, 'From_w_ref_scope/1', 'Mux_Speed/1');
add_line(mdl, 'From_w_meas_scope/1', 'Mux_Speed/2');
add_block('simulink/Sinks/Scope', [mdl '/Speed Scope'], ...
    'Position', [160 y+5 210 y+45], 'NumInputPorts', '1');
add_line(mdl, 'Mux_Speed/1', 'Speed Scope/1');

add_from(mdl, 'iq_ff_scope', [260 y 320 y+20]);
set_param([mdl '/From_iq_ff_scope'], 'GotoTag', 'iq_ff');
add_from(mdl, 'T_load_scope', [260 y+30 320 y+50]);
set_param([mdl '/From_T_load_scope'], 'GotoTag', 'T_load');
add_from(mdl, 'learn_scope', [260 y+60 320 y+80]);
set_param([mdl '/From_learn_scope'], 'GotoTag', 'learn_active');
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_Vib'], ...
    'Position', [360 y 365 y+80], 'Inputs', '3');
add_line(mdl, 'From_iq_ff_scope/1', 'Mux_Vib/1');
add_line(mdl, 'From_T_load_scope/1', 'Mux_Vib/2');
add_line(mdl, 'From_learn_scope/1', 'Mux_Vib/3');
add_block('simulink/Sinks/Scope', [mdl '/Vibration Scope'], ...
    'Position', [400 y+10 450 y+70], 'NumInputPorts', '1');
add_line(mdl, 'Mux_Vib/1', 'Vibration Scope/1');

add_block('simulink/Sinks/To Workspace', [mdl '/Log_vib_wref'], ...
    'Position', [120 y+120 210 y+145], ...
    'VariableName', 'log_vib_wref', 'SaveFormat', 'Timeseries');
add_from(mdl, 'w_ref_log', [20 y+120 80 y+140]);
set_param([mdl '/From_w_ref_log'], 'GotoTag', 'w_ref');
add_line(mdl, 'From_w_ref_log/1', 'Log_vib_wref/1');

add_block('simulink/Sinks/To Workspace', [mdl '/Log_vib_wm'], ...
    'Position', [120 y+155 210 y+180], ...
    'VariableName', 'log_vib_wm', 'SaveFormat', 'Timeseries');
add_from(mdl, 'w_meas_log', [20 y+155 80 y+175]);
set_param([mdl '/From_w_meas_log'], 'GotoTag', 'w_meas');
add_line(mdl, 'From_w_meas_log/1', 'Log_vib_wm/1');

add_block('simulink/Sinks/To Workspace', [mdl '/Log_vib_theta'], ...
    'Position', [120 y+190 210 y+215], ...
    'VariableName', 'log_vib_theta', 'SaveFormat', 'Timeseries');
add_from(mdl, 'theta_meas_log', [20 y+190 80 y+210]);
set_param([mdl '/From_theta_meas_log'], 'GotoTag', 'theta_meas');
add_line(mdl, 'From_theta_meas_log/1', 'Log_vib_theta/1');

add_block('simulink/Sinks/To Workspace', [mdl '/Log_vib_iqbase'], ...
    'Position', [120 y+225 210 y+250], ...
    'VariableName', 'log_vib_iqbase', 'SaveFormat', 'Timeseries');
add_from(mdl, 'iq_ref_base_log', [20 y+225 80 y+245]);
set_param([mdl '/From_iq_ref_base_log'], 'GotoTag', 'iq_ref_base');
add_line(mdl, 'From_iq_ref_base_log/1', 'Log_vib_iqbase/1');

add_block('simulink/Sinks/To Workspace', [mdl '/Log_vib_iqff'], ...
    'Position', [320 y+120 410 y+145], ...
    'VariableName', 'log_vib_iqff', 'SaveFormat', 'Timeseries');
add_from(mdl, 'iq_ff_log', [260 y+120 320 y+140]);
set_param([mdl '/From_iq_ff_log'], 'GotoTag', 'iq_ff');
add_line(mdl, 'From_iq_ff_log/1', 'Log_vib_iqff/1');

add_block('simulink/Sinks/To Workspace', [mdl '/Log_vib_tload'], ...
    'Position', [320 y+155 410 y+180], ...
    'VariableName', 'log_vib_tload', 'SaveFormat', 'Timeseries');
add_from(mdl, 'T_load_log', [260 y+155 320 y+175]);
set_param([mdl '/From_T_load_log'], 'GotoTag', 'T_load');
add_line(mdl, 'From_T_load_log/1', 'Log_vib_tload/1');

add_block('simulink/Sinks/To Workspace', [mdl '/Log_vib_learn'], ...
    'Position', [320 y+190 410 y+215], ...
    'VariableName', 'log_vib_learn', 'SaveFormat', 'Timeseries');
add_from(mdl, 'learn_log', [260 y+190 320 y+210]);
set_param([mdl '/From_learn_log'], 'GotoTag', 'learn_active');
add_line(mdl, 'From_learn_log/1', 'Log_vib_learn/1');

save_system(mdl, fullfile(pwd, [mdl '.slx']));
fprintf('Vibration compensation test model saved: %s.slx\n', mdl);
fprintf('Logged signals: log_vib_wref, log_vib_wm, log_vib_theta, log_vib_iqbase, log_vib_iqff, log_vib_tload, log_vib_learn\n');
end
