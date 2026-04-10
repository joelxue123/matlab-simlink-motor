function build_speedloop_kf_test()
% Build a speed-loop-only FOC model with Kalman speed estimator evaluation.
% This model isolates speed loop dynamics and compares w_meas vs w_kf.
%
% Usage:
%   motor_control_params;
%   build_speedloop_kf_test;
%   open_system('speedloop_kf_test');
%   sim('speedloop_kf_test');

mdl = 'speedloop_kf_test';

load_system('mcbplantlib');
load_system('mcblib');

if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end

new_system(mdl);
set_param(mdl, 'Solver', 'FixedStepAuto', ...
    'FixedStep', 'simcfg.Ts_plant', ...
    'StopTime', 'simcfg.stop_time', ...
    'InitFcn', 'motor_control_params');

% Add reusable module directories to path
script_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(script_dir, 'build_modules'));
addpath(fullfile(script_dir, 'algorithms'));

% =====================================================================
% Row 0: Speed reference profile (Ts_speed)
% =====================================================================
y = 50;
add_block('simulink/Sources/Step', [mdl '/WrefStepUp'], ...
    'Position', [40 y 100 y+25], ...
    'Time', '0.02', ...
    'Before', '0', ...
    'After', 'motor.speed_ref_mech_rad_s', ...
    'SampleTime', 'simcfg.Ts_speed');

add_block('simulink/Sources/Step', [mdl '/WrefStepDown'], ...
    'Position', [40 y+35 100 y+60], ...
    'Time', '0.30', ...
    'Before', '0', ...
    'After', '-motor.speed_ref_mech_rad_s', ...
    'SampleTime', 'simcfg.Ts_speed');

add_block('simulink/Math Operations/Add', [mdl '/WrefSum'], ...
    'Position', [140 y+10 180 y+50], ...
    'Inputs', '++');
add_line(mdl, 'WrefStepUp/1', 'WrefSum/1');
add_line(mdl, 'WrefStepDown/1', 'WrefSum/2');
add_goto(mdl, 'w_ref', [210 y+15 280 y+40], 'WrefSum/1');

% =====================================================================
% Row 1: Speed PI (Ts_speed)
%   Rate Transition: w_kf (Ts_ctrl) -> Ts_speed
% =====================================================================
y = 150;
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_wm_to_speed'], ...
    'Position', [20 y+35 80 y+55], ...
    'OutPortSampleTime', 'simcfg.Ts_speed');

create_subsystem(mdl, 'Speed PI', [120 y 300 y+90], ...
    {'w_ref', 'w_meas'}, {'iq_ref'});
populate_speed_pi(mdl);
add_from(mdl, 'w_ref', [20 y+5 80 y+25]);
add_from(mdl, 'w_kf_speed', [-60 y+35 -10 y+55]);
set_param([mdl '/From_w_kf_speed'], 'GotoTag', 'w_kf');
add_line(mdl, 'From_w_kf_speed/1', 'RT_wm_to_speed/1');
add_line(mdl, 'From_w_ref/1', 'Speed PI/1');
add_line(mdl, 'RT_wm_to_speed/1', 'Speed PI/2');
add_goto(mdl, 'iq_ref_cmd', [330 y+20 410 y+45], 'Speed PI/1');

% Rate Transition: iq_ref_cmd (Ts_speed) -> Ts_ctrl
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_iqref_to_ctrl'], ...
    'Position', [440 y+20 510 y+45], ...
    'OutPortSampleTime', 'simcfg.Ts_ctrl');
add_from(mdl, 'iq_ref_cmd_slow', [360 y+20 420 y+45]);
set_param([mdl '/From_iq_ref_cmd_slow'], 'GotoTag', 'iq_ref_cmd');
add_line(mdl, 'From_iq_ref_cmd_slow/1', 'RT_iqref_to_ctrl/1');
add_goto(mdl, 'iq_ref_cmd_fast', [530 y+20 620 y+45], 'RT_iqref_to_ctrl/1');

% =====================================================================
% Row 2: Current reference (id_ref=0, iq_ref passthrough)
% =====================================================================
y = 290;
create_subsystem(mdl, 'Current Ref', [120 y 300 y+65], ...
    {'iq_ref_cmd'}, {'id_ref', 'iq_ref'});
populate_current_ref(mdl);
add_from(mdl, 'iq_ref_cmd_fast', [20 y+10 80 y+30]);
add_line(mdl, 'From_iq_ref_cmd_fast/1', 'Current Ref/1');
add_goto(mdl, 'id_ref', [330 y 410 y+20], 'Current Ref/1');
add_goto(mdl, 'iq_ref', [330 y+35 410 y+55], 'Current Ref/2');

% =====================================================================
% Row 3: abc -> dq (Ts_ctrl)
%   Rate Transition: ia, ib, ic, theta_e (Ts_plant) -> Ts_ctrl
% =====================================================================
y = 410;
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
% Row 4: Current PI (Ts_ctrl)
%   Rate Transition: omega_e (Ts_plant) -> Ts_ctrl
% =====================================================================
y = 590;
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
% Row 5: dq -> abc (Ts_ctrl)
%   Rate Transition: theta_e (Ts_plant) -> Ts_ctrl
% =====================================================================
y = 810;
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
% Row 6: Inverter and modulation (ctrl -> plant)
% =====================================================================
y = 970;
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
% Row 7: PMSM plant (Ts_plant)
% =====================================================================
y = 1140;
add_block('simulink/Sources/Step', [mdl '/LoadStep'], ...
    'Position', [40 y 100 y+25], ...
    'Time', 'control.load_step_time', ...
    'Before', 'motor.load_torque', ...
    'After', 'control.load_step_torque', ...
    'SampleTime', 'simcfg.Ts_plant');
add_goto(mdl, 'T_load', [140 y 210 y+20], 'LoadStep/1');

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

add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/PosUnwrap'], ...
    'Position', [400 y+5 490 y+35]);
populate_pos_unwrap(mdl);
add_line(mdl, 'BusSel_Info/1', 'PosUnwrap/1');
add_goto(mdl, 'theta_meas', [520 y+5 600 y+35], 'PosUnwrap/1');

% Optional measurement noise for KF input quality test
add_block('simulink/Sources/Random Number', [mdl '/ThetaNoiseRnd'], ...
    'Position', [620 y+5 700 y+35], ...
    'Mean', '0', ...
    'Variance', 'control.kf.test_noise_var', ...
    'SampleTime', 'simcfg.Ts_ctrl');
add_block('simulink/Math Operations/Add', [mdl '/ThetaNoisyAdd'], ...
    'Position', [730 y+5 770 y+35], ...
    'Inputs', '++');
add_from(mdl, 'theta_meas_noise', [650 y-40 700 y-20]);
set_param([mdl '/From_theta_meas_noise'], 'GotoTag', 'theta_meas');
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_theta_to_kf'], ...
    'Position', [710 y-35 780 y-15], ...
    'OutPortSampleTime', 'simcfg.Ts_ctrl');
add_line(mdl, 'From_theta_meas_noise/1', 'RT_theta_to_kf/1');
add_line(mdl, 'RT_theta_to_kf/1', 'ThetaNoisyAdd/1');
add_line(mdl, 'ThetaNoiseRnd/1', 'ThetaNoisyAdd/2');
add_goto(mdl, 'theta_meas_noisy', [800 y+5 900 y+35], 'ThetaNoisyAdd/1');

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
% Row 8: Kalman speed estimator (observer rate: Ts_ctrl)
% =====================================================================
y = 1390;
create_subsystem(mdl, 'Kalman Speed', [120 y 300 y+90], ...
    {'theta_meas'}, {'w_kf', 'theta_kf'});
populate_kalman_speed(mdl);
add_from(mdl, 'theta_meas_noisy_kf', [20 y+20 90 y+40]);
set_param([mdl '/From_theta_meas_noisy_kf'], 'GotoTag', 'theta_meas_noisy');
add_line(mdl, 'From_theta_meas_noisy_kf/1', 'Kalman Speed/1');
add_goto(mdl, 'w_kf', [330 y+15 400 y+35], 'Kalman Speed/1');
add_goto(mdl, 'theta_kf', [330 y+50 410 y+70], 'Kalman Speed/2');

% =====================================================================
% Row 9: Scopes and logs for KF evaluation
% =====================================================================
y = 1560;
add_from(mdl, 'w_ref_scope', [20 y 70 y+20]);
set_param([mdl '/From_w_ref_scope'], 'GotoTag', 'w_ref');
add_from(mdl, 'w_meas_scope', [20 y+30 70 y+50]);
set_param([mdl '/From_w_meas_scope'], 'GotoTag', 'w_meas');
add_from(mdl, 'w_kf_scope', [20 y+60 70 y+80]);
set_param([mdl '/From_w_kf_scope'], 'GotoTag', 'w_kf');

add_block('simulink/Signal Routing/Mux', [mdl '/Mux_Speed'], ...
    'Position', [110 y 115 y+80], 'Inputs', '3');
add_line(mdl, 'From_w_ref_scope/1', 'Mux_Speed/1');
add_line(mdl, 'From_w_meas_scope/1', 'Mux_Speed/2');
add_line(mdl, 'From_w_kf_scope/1', 'Mux_Speed/3');
add_block('simulink/Sinks/Scope', [mdl '/Speed Scope'], ...
    'Position', [150 y+15 200 y+65], 'NumInputPorts', '1');
add_line(mdl, 'Mux_Speed/1', 'Speed Scope/1');

% Error signal: w_kf - w_meas
add_from(mdl, 'w_meas_err', [250 y+20 300 y+40]);
set_param([mdl '/From_w_meas_err'], 'GotoTag', 'w_meas');
add_from(mdl, 'w_kf_err', [250 y+50 300 y+70]);
set_param([mdl '/From_w_kf_err'], 'GotoTag', 'w_kf');
add_block('simulink/Math Operations/Add', [mdl '/SpeedError'], ...
    'Position', [330 y+30 370 y+60], 'Inputs', '+-');
add_line(mdl, 'From_w_kf_err/1', 'SpeedError/1');
add_line(mdl, 'From_w_meas_err/1', 'SpeedError/2');
add_block('simulink/Sinks/Scope', [mdl '/KF Error Scope'], ...
    'Position', [410 y+30 460 y+60], 'NumInputPorts', '1');
add_line(mdl, 'SpeedError/1', 'KF Error Scope/1');
add_goto(mdl, 'w_err', [390 y+75 450 y+95], 'SpeedError/1');

add_block('simulink/Sinks/To Workspace', [mdl '/Log_wref'], ...
    'Position', [120 y+120 190 y+145], ...
    'VariableName', 'log_wref', 'SaveFormat', 'Timeseries');
add_from(mdl, 'w_ref_log', [20 y+120 70 y+140]);
set_param([mdl '/From_w_ref_log'], 'GotoTag', 'w_ref');
add_line(mdl, 'From_w_ref_log/1', 'Log_wref/1');

add_block('simulink/Sinks/To Workspace', [mdl '/Log_wm'], ...
    'Position', [120 y+150 190 y+175], ...
    'VariableName', 'log_wm', 'SaveFormat', 'Timeseries');
add_from(mdl, 'w_meas_log', [20 y+150 70 y+170]);
set_param([mdl '/From_w_meas_log'], 'GotoTag', 'w_meas');
add_line(mdl, 'From_w_meas_log/1', 'Log_wm/1');

add_block('simulink/Sinks/To Workspace', [mdl '/Log_wkf'], ...
    'Position', [120 y+180 190 y+205], ...
    'VariableName', 'log_wkf', 'SaveFormat', 'Timeseries');
add_from(mdl, 'w_kf_log', [20 y+180 70 y+200]);
set_param([mdl '/From_w_kf_log'], 'GotoTag', 'w_kf');
add_line(mdl, 'From_w_kf_log/1', 'Log_wkf/1');

add_block('simulink/Sinks/To Workspace', [mdl '/Log_werr'], ...
    'Position', [120 y+210 190 y+235], ...
    'VariableName', 'log_werr', 'SaveFormat', 'Timeseries');
add_from(mdl, 'w_err_log', [20 y+210 70 y+230]);
set_param([mdl '/From_w_err_log'], 'GotoTag', 'w_err');
add_line(mdl, 'From_w_err_log/1', 'Log_werr/1');

save_system(mdl, fullfile(pwd, [mdl '.slx']));
fprintf('Speed-loop KF test model saved: %s.slx\n', mdl);
fprintf('Logged signals: log_wref, log_wm, log_wkf, log_werr\n');
end
