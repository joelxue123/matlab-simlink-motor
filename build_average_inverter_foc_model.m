
function build_average_inverter_foc_model()
% Build SPMSM FOC model with average inverter.
% Uses Goto/From labels instead of wires for a clean layout.
%
% Signal labels used:
%   pos_ref, theta_meas                    — position reference & feedback
%   w_ref, w_meas, omega_e, theta_e       — speed/angle signals
%   iq_ref, id_ref, iq_meas, id_meas      — current references & feedback
%   vd_ref, vq_ref                         — voltage references (dq)
%   da, db, dc                             — duty cycles [0,1] (abc)
%   ia, ib, ic                             — phase currents
%   Vabc_mod, Vdc_bus                      — inverter inputs
%   Vabc_out                               — inverter output
%   T_load                                 — load torque

mdl = 'average_inverter_foc';

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
% Row 0: Position reference generation  (runs at position-loop rate)
% =====================================================================
y = 50;
add_block('simulink/Sources/Step', [mdl '/PosRefStep'], ...
    'Position', [50 y 100 y+30], ...
    'Time', 'control.pos_step_time', 'Before', '0', 'After', 'control.pos_ref_rad', ...
    'SampleTime', 'simcfg.Ts_pos');

% 梯形速度规划: 限速+限加速度, 平滑位置指令
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/TrajPlanner'], ...
    'Position', [140 y-5 260 y+35]);
populate_traj_planner(mdl);

add_block('simulink/Sources/Constant', [mdl '/TrajParams'], ...
    'Position', [50 y+40 230 y+60], ...
    'Value', '[control.pos_max_vel, control.pos_max_acc, simcfg.Ts_pos]');
add_line(mdl, 'PosRefStep/1', 'TrajPlanner/1');
add_line(mdl, 'TrajParams/1', 'TrajPlanner/2');
add_goto(mdl, 'pos_ref', [290 y 370 y+30], 'TrajPlanner/1');

% =====================================================================
% Row 1: Position P controller  (position-loop rate: Ts_pos)
%   Rate Transition: theta_meas (Ts_plant) → Ts_pos
% =====================================================================
y = 130;
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_theta_to_pos'], ...
    'Position', [20 y+40 70 y+60], ...
    'OutPortSampleTime', 'simcfg.Ts_pos');
create_subsystem(mdl, 'Position P', [100 y 260 y+80], ...
    {'pos_ref', 'theta_meas'}, {'w_ref'});
populate_position_p(mdl);
add_from(mdl, 'pos_ref',  [20 y+10  70 y+30]);
add_from(mdl, 'theta_meas_pos', [20-80 y+40 20-30 y+60]);
set_param([mdl '/From_theta_meas_pos'], 'GotoTag', 'theta_meas');
add_line(mdl, 'From_theta_meas_pos/1', 'RT_theta_to_pos/1');
add_line(mdl, 'From_pos_ref/1',   'Position P/1');
add_line(mdl, 'RT_theta_to_pos/1', 'Position P/2');
add_goto(mdl, 'w_ref_pos', [300 y+20 380 y+50], 'Position P/1');

% Rate Transition: w_ref (Ts_pos) → Ts_speed
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_wref_to_speed'], ...
    'Position', [400 y+20 470 y+50], ...
    'OutPortSampleTime', 'simcfg.Ts_speed');
add_from(mdl, 'w_ref_pos_rt', [400-80 y+20 400-30 y+50]);
set_param([mdl '/From_w_ref_pos_rt'], 'GotoTag', 'w_ref_pos');
add_line(mdl, 'From_w_ref_pos_rt/1', 'RT_wref_to_speed/1');
add_goto(mdl, 'w_ref', [490 y+20 560 y+50], 'RT_wref_to_speed/1');

% =====================================================================
% Row 2: Speed PI controller  (speed-loop rate: Ts_speed)
% Rate Transition: w_kf (Ts_ctrl) → Ts_speed
% =====================================================================
y = 260;
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_wm_to_speed'], ...
    'Position', [20 y+40 70 y+60], ...
    'OutPortSampleTime', 'simcfg.Ts_speed');

create_subsystem(mdl, 'Speed PI', [100 y 260 y+80], ...
    {'w_ref', 'w_meas'}, {'iq_ref'});
populate_speed_pi(mdl);
add_from(mdl, 'w_ref',  [20 y+10  70 y+30]);
add_from(mdl, 'w_kf_speed', [20-80 y+40 20-30 y+60]);
set_param([mdl '/From_w_kf_speed'], 'GotoTag', 'w_kf');
add_line(mdl, 'From_w_kf_speed/1', 'RT_wm_to_speed/1');
add_line(mdl, 'From_w_ref/1',   'Speed PI/1');
add_line(mdl, 'RT_wm_to_speed/1', 'Speed PI/2');
add_goto(mdl, 'iq_ref_cmd', [300 y+20 380 y+50], 'Speed PI/1');

% Rate Transition: iq_ref_cmd (Ts_speed) → Ts_ctrl (50us)
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_iqref_to_ctrl'], ...
    'Position', [400 y+20 470 y+50], ...
    'OutPortSampleTime', 'simcfg.Ts_ctrl');
add_from(mdl, 'iq_ref_cmd_slow', [400-80 y+20 400-30 y+50]);
set_param([mdl '/From_iq_ref_cmd_slow'], 'GotoTag', 'iq_ref_cmd');
add_line(mdl, 'From_iq_ref_cmd_slow/1', 'RT_iqref_to_ctrl/1');
add_goto(mdl, 'iq_ref_cmd_fast', [490 y+20 580 y+50], 'RT_iqref_to_ctrl/1');

% =====================================================================
% Row 3: Current reference (id_ref=0, iq_ref passthrough)
%         Runs at current-loop rate (Ts_ctrl = 50us)
% =====================================================================
y = 390;
create_subsystem(mdl, 'Current Ref', [100 y 260 y+60], ...
    {'iq_ref_cmd'}, {'id_ref', 'iq_ref'});
populate_current_ref(mdl);
add_from(mdl, 'iq_ref_cmd_fast', [20 y+10 70 y+30]);
add_line(mdl, 'From_iq_ref_cmd_fast/1', 'Current Ref/1');
add_goto(mdl, 'id_ref', [300 y     380 y+20],  'Current Ref/1');
add_goto(mdl, 'iq_ref', [300 y+35  380 y+55],  'Current Ref/2');

% =====================================================================
% Row 4: abc->dq transform  (current-loop rate: Ts_ctrl = 50us)
%   Rate Transition: ia,ib,ic,theta_e from plant (25us) → ctrl (50us)
% =====================================================================
y = 500;

% RT blocks: plant → ctrl rate for current feedback and angle
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_ia'], ...
    'Position', [-40 y+5  10 y+25], 'OutPortSampleTime', 'simcfg.Ts_ctrl');
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_ib'], ...
    'Position', [-40 y+30 10 y+50], 'OutPortSampleTime', 'simcfg.Ts_ctrl');
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_ic'], ...
    'Position', [-40 y+55 10 y+75], 'OutPortSampleTime', 'simcfg.Ts_ctrl');
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_theta_ctrl'], ...
    'Position', [-40 y+85 10 y+105], 'OutPortSampleTime', 'simcfg.Ts_ctrl');

add_from(mdl, 'ia',      [-120 y+5   -70 y+25]);
add_from(mdl, 'ib',      [-120 y+30  -70 y+50]);
add_from(mdl, 'ic',      [-120 y+55  -70 y+75]);
add_from(mdl, 'theta_e', [-120 y+85  -70 y+105]);
add_line(mdl, 'From_ia/1',      'RT_ia/1');
add_line(mdl, 'From_ib/1',      'RT_ib/1');
add_line(mdl, 'From_ic/1',      'RT_ic/1');
add_line(mdl, 'From_theta_e/1', 'RT_theta_ctrl/1');

create_subsystem(mdl, 'abc to dq', [100 y 260 y+120], ...
    {'ia', 'ib', 'ic', 'theta_e'}, {'id_meas', 'iq_meas'});
populate_abc2dq(mdl);
add_line(mdl, 'RT_ia/1',           'abc to dq/1');
add_line(mdl, 'RT_ib/1',           'abc to dq/2');
add_line(mdl, 'RT_ic/1',           'abc to dq/3');
add_line(mdl, 'RT_theta_ctrl/1',   'abc to dq/4');
add_goto(mdl, 'id_meas', [300 y+20  380 y+45],  'abc to dq/1');
add_goto(mdl, 'iq_meas', [300 y+65  380 y+90],  'abc to dq/2');

% =====================================================================
% Row 5: Current PI controller  (current-loop rate: Ts_ctrl = 50us)
%   Rate Transition: omega_e from plant (25us) → ctrl (50us)
% =====================================================================
y = 670;
create_subsystem(mdl, 'Current PI', [100 y 300 y+150], ...
    {'id_ref', 'iq_ref', 'id_meas', 'iq_meas', 'omega_e'}, {'vd_ref', 'vq_ref'});
populate_current_pi(mdl);
add_from(mdl, 'id_ref',  [20 y+5   70 y+25]);
add_from(mdl, 'iq_ref',  [20 y+30  70 y+50]);
add_from(mdl, 'id_meas', [20 y+55  70 y+75]);
add_from(mdl, 'iq_meas', [20 y+80  70 y+100]);

% omega_e needs RT from plant rate to ctrl rate
add_from(mdl, 'omega_e', [-80 y+110 -30 y+130]);
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_omega_ctrl'], ...
    'Position', [-20 y+110 50 y+130], ...
    'OutPortSampleTime', 'simcfg.Ts_ctrl');
add_line(mdl, 'From_omega_e/1', 'RT_omega_ctrl/1');

add_line(mdl, 'From_id_ref/1',  'Current PI/1');
add_line(mdl, 'From_iq_ref/1',  'Current PI/2');
add_line(mdl, 'From_id_meas/1', 'Current PI/3');
add_line(mdl, 'From_iq_meas/1', 'Current PI/4');
add_line(mdl, 'RT_omega_ctrl/1', 'Current PI/5');
add_goto(mdl, 'vd_ref', [340 y+30  410 y+55],  'Current PI/1');
add_goto(mdl, 'vq_ref', [340 y+90  410 y+115], 'Current PI/2');

% =====================================================================
% Row 6: dq->abc transform (逆Park + 逆Clarke + SVPWM + 占空比)
%         Runs at current-loop rate (Ts_ctrl = 50us)
%   Rate Transition: theta_e from plant (25us) → ctrl (50us)
% =====================================================================
y = 870;
create_subsystem(mdl, 'dq to abc', [100 y 280 y+120], ...
    {'vd_ref', 'vq_ref', 'theta_e', 'Vdc'}, {'da', 'db', 'dc'});
populate_dq2abc(mdl);  % Fill subsystem internals
add_from(mdl, 'vd_ref',   [20 y+5   70 y+25]);
add_from(mdl, 'vq_ref',   [20 y+35  70 y+55]);

% theta_e at plant rate → RT → ctrl rate for inverse Park
add_from(mdl, 'theta_e_2',[-60 y+70  -10 y+90]);
set_param([mdl '/From_theta_e_2'], 'GotoTag', 'theta_e');
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_theta_dq2abc'], ...
    'Position', [-5 y+70 65 y+90], ...
    'OutPortSampleTime', 'simcfg.Ts_ctrl');
add_line(mdl, 'From_theta_e_2/1', 'RT_theta_dq2abc/1');

add_block('simulink/Sources/Constant', [mdl '/Vdc_dq2abc'], ...
    'Position', [20 y+100 70 y+120], 'Value', 'inverter.Vdc');
add_line(mdl, 'From_vd_ref/1',      'dq to abc/1');
add_line(mdl, 'From_vq_ref/1',      'dq to abc/2');
add_line(mdl, 'RT_theta_dq2abc/1',  'dq to abc/3');
add_line(mdl, 'Vdc_dq2abc/1',       'dq to abc/4');
add_goto(mdl, 'da', [300 y     380 y+20],  'dq to abc/1');
add_goto(mdl, 'db', [300 y+35  380 y+55],  'dq to abc/2');
add_goto(mdl, 'dc', [300 y+70  380 y+90],  'dq to abc/3');

% =====================================================================
% Row 7: 占空比 [0,1] → 调制指数 [-1,+1] → Average Inverter
%   Rate Transition: duty (Ts_ctrl 50us) → plant (Ts_plant 25us)
% =====================================================================
y = 1030;
add_from(mdl, 'da', [20 y     70 y+20]);
add_from(mdl, 'db', [20 y+40  70 y+60]);
add_from(mdl, 'dc', [20 y+80  70 y+100]);

add_block('simulink/Signal Routing/Mux', [mdl '/Mux_Mod'], ...
    'Position', [100 y+10 105 y+90], 'Inputs', '3');
add_line(mdl, 'From_da/1', 'Mux_Mod/1');
add_line(mdl, 'From_db/1', 'Mux_Mod/2');
add_line(mdl, 'From_dc/1', 'Mux_Mod/3');

% Rate Transition: ctrl → plant (50us → 25us)
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_mod_to_plant'], ...
    'Position', [130 y+30 200 y+70], ...
    'OutPortSampleTime', 'simcfg.Ts_plant');
add_line(mdl, 'Mux_Mod/1', 'RT_mod_to_plant/1');

add_block('simulink/Sources/Constant', [mdl '/Vdc'], ...
    'Position', [200 y+110 250 y+130], 'Value', 'inverter.Vdc');

add_block('mcbplantlib/Average-Value Inverter', [mdl '/Average Inverter'], ...
    'Position', [360 y 520 y+90]);
add_line(mdl, 'RT_mod_to_plant/1', 'Average Inverter/1');
add_line(mdl, 'Vdc/1', 'Average Inverter/2');
add_goto(mdl, 'Vabc_out', [560 y+20 640 y+45], 'Average Inverter/1');

% =====================================================================
% Row 8: PMSM Plant  (runs at plant rate: Ts_plant = 25us)
% =====================================================================
y = 1200;
add_from(mdl, 'T_load',   [20 y     70 y+20]);
add_from(mdl, 'Vabc_out', [20 y+60  70 y+80]);
add_block('simulink/Sources/Step', [mdl '/LoadStep'], ...
    'Position', [100 y-5 150 y+25], ...
    'Time', 'control.load_step_time', ...
    'Before', 'motor.load_torque', ...
    'After', 'control.load_step_torque', ...
    'SampleTime', 'simcfg.Ts_plant');
add_goto(mdl, 'T_load', [190 y 260 y+20], 'LoadStep/1');

add_block('mcblib/Electrical Systems/Motors/Surface Mount PMSM', ...
    [mdl '/Surface Mount PMSM'], 'Position', [100 y+40 310 y+200]);
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
add_line(mdl, 'From_T_load/1',   'Surface Mount PMSM/1');
add_line(mdl, 'From_Vabc_out/1', 'Surface Mount PMSM/2');

% PMSM outputs: port1=Info(bus), port2=Iabc, port3=wm  (all at Ts_plant)

% Info bus → Bus Selector → MtrPos (机械角 θm)
add_block('simulink/Signal Routing/Bus Selector', [mdl '/BusSel_Info'], ...
    'Position', [350 y+40 355 y+80], ...
    'OutputSignals', 'MtrPos');
add_line(mdl, 'Surface Mount PMSM/1', 'BusSel_Info/1');

% theta_e = MtrPos * pole_pairs (电角度, 直接从模型获取, 无积分漂移)
add_block('simulink/Math Operations/Gain', [mdl '/pos2theta_e'], ...
    'Position', [390 y+45 450 y+75], 'Gain', 'motor.pole_pairs');
add_line(mdl, 'BusSel_Info/1', 'pos2theta_e/1');
add_goto(mdl, 'theta_e', [480 y+45 550 y+75], 'pos2theta_e/1');

% theta_meas = unwrap(MtrPos), 检测2π跳变累积连续位置
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/PosUnwrap'], ...
    'Position', [390 y+5 480 y+35]);
populate_pos_unwrap(mdl);
add_line(mdl, 'BusSel_Info/1', 'PosUnwrap/1');
add_goto(mdl, 'theta_meas', [510 y+5 590 y+35], 'PosUnwrap/1');

% Demux Iabc
add_block('simulink/Signal Routing/Demux', [mdl '/Demux_Iabc'], ...
    'Position', [350 y+95 355 y+175], 'Outputs', '3');
add_line(mdl, 'Surface Mount PMSM/2', 'Demux_Iabc/1');
add_goto(mdl, 'ia', [390 y+80  450 y+100], 'Demux_Iabc/1');
add_goto(mdl, 'ib', [390 y+115 450 y+135], 'Demux_Iabc/2');
add_goto(mdl, 'ic', [390 y+150 450 y+170], 'Demux_Iabc/3');

% wm → w_meas (at plant rate, will be rate-transitioned to speed loop)
add_goto(mdl, 'w_meas', [350 y+190 430 y+210], 'Surface Mount PMSM/3');

% wm → omega_e  (all at plant rate)
add_block('simulink/Math Operations/Gain', [mdl '/wm2we'], ...
    'Position', [350 y+230 410 y+260], 'Gain', 'motor.pole_pairs');
add_line(mdl, 'Surface Mount PMSM/3', 'wm2we/1');
add_goto(mdl, 'omega_e', [450 y+230 520 y+255], 'wm2we/1');

% =====================================================================
% Row 8b: Kalman filter speed estimator (observer rate: Ts_ctrl)
%   Input: theta_meas rate-transitioned to 50us observation period
%   Output: w_kf (filtered speed), theta_kf (filtered position)
% =====================================================================
y = 1370;
add_block('simulink/Signal Attributes/Rate Transition', [mdl '/RT_theta_to_kf'], ...
    'Position', [20 y+20 80 y+40], ...
    'OutPortSampleTime', 'simcfg.Ts_ctrl');
create_subsystem(mdl, 'Kalman Speed', [100 y 280 y+80], ...
    {'theta_meas'}, {'w_kf', 'theta_kf'});
populate_kalman_speed(mdl);
add_from(mdl, 'theta_meas_kf', [-60 y+20 -10 y+40]);
set_param([mdl '/From_theta_meas_kf'], 'GotoTag', 'theta_meas');
add_line(mdl, 'From_theta_meas_kf/1', 'RT_theta_to_kf/1');
add_line(mdl, 'RT_theta_to_kf/1', 'Kalman Speed/1');
add_goto(mdl, 'w_kf',     [320 y+10  390 y+30],  'Kalman Speed/1');
add_goto(mdl, 'theta_kf', [320 y+45  390 y+65],  'Kalman Speed/2');

% =====================================================================
% Row 9: Scope outputs
% =====================================================================
y = 1570;
add_from(mdl, 'w_meas_scope',  [20 y     70 y+20]);
set_param([mdl '/From_w_meas_scope'], 'GotoTag', 'w_meas');
add_from(mdl, 'id_meas_scope', [20 y+40  70 y+60]);
set_param([mdl '/From_id_meas_scope'], 'GotoTag', 'id_meas');
add_from(mdl, 'iq_meas_scope', [20 y+80  70 y+100]);
set_param([mdl '/From_iq_meas_scope'], 'GotoTag', 'iq_meas');

add_from(mdl, 'w_kf_scope', [20 y+20 70 y+40]);
set_param([mdl '/From_w_kf_scope'], 'GotoTag', 'w_kf');
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_Speed'], ...
    'Position', [100 y-5 105 y+35], 'Inputs', '2');
add_line(mdl, 'From_w_meas_scope/1', 'Mux_Speed/1');
add_line(mdl, 'From_w_kf_scope/1',   'Mux_Speed/2');
add_block('simulink/Sinks/Scope', [mdl '/Speed Scope'], ...
    'Position', [140 y-5 190 y+35], 'NumInputPorts', '1');
add_line(mdl, 'Mux_Speed/1', 'Speed Scope/1');

add_block('simulink/Signal Routing/Mux', [mdl '/Mux_Idq'], ...
    'Position', [100 y+45 105 y+95], 'Inputs', '2');
add_line(mdl, 'From_id_meas_scope/1', 'Mux_Idq/1');
add_line(mdl, 'From_iq_meas_scope/1', 'Mux_Idq/2');
add_block('simulink/Sinks/Scope', [mdl '/Current dq Scope'], ...
    'Position', [140 y+50 190 y+90], 'NumInputPorts', '1');
add_line(mdl, 'Mux_Idq/1', 'Current dq Scope/1');

% Also log to workspace
add_block('simulink/Sinks/To Workspace', [mdl '/Log_wm'], ...
    'Position', [120 y+120 190 y+145], ...
    'VariableName', 'log_wm', 'SaveFormat', 'Timeseries');
add_from(mdl, 'w_meas_log', [20 y+120 70 y+140]);
set_param([mdl '/From_w_meas_log'], 'GotoTag', 'w_meas');
add_line(mdl, 'From_w_meas_log/1', 'Log_wm/1');

% Log Kalman-filtered speed to workspace
add_block('simulink/Sinks/To Workspace', [mdl '/Log_wkf'], ...
    'Position', [120 y+150 190 y+175], ...
    'VariableName', 'log_wkf', 'SaveFormat', 'Timeseries');
add_from(mdl, 'w_kf_log', [20 y+150 70 y+170]);
set_param([mdl '/From_w_kf_log'], 'GotoTag', 'w_kf');
add_line(mdl, 'From_w_kf_log/1', 'Log_wkf/1');

% Position scope
add_from(mdl, 'pos_ref_scope', [20 y+170 70 y+190]);
set_param([mdl '/From_pos_ref_scope'], 'GotoTag', 'pos_ref');
add_from(mdl, 'theta_meas_scope', [20 y+210 70 y+230]);
set_param([mdl '/From_theta_meas_scope'], 'GotoTag', 'theta_meas');
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_Pos'], ...
    'Position', [100 y+175 105 y+225], 'Inputs', '2');
add_line(mdl, 'From_pos_ref_scope/1', 'Mux_Pos/1');
add_line(mdl, 'From_theta_meas_scope/1', 'Mux_Pos/2');
add_block('simulink/Sinks/Scope', [mdl '/Position Scope'], ...
    'Position', [140 y+180 190 y+220], 'NumInputPorts', '1');
add_line(mdl, 'Mux_Pos/1', 'Position Scope/1');

add_block('simulink/Sinks/To Workspace', [mdl '/Log_pos'], ...
    'Position', [250 y+180 320 y+205], ...
    'VariableName', 'log_pos', 'SaveFormat', 'Timeseries');
add_from(mdl, 'theta_meas_log', [170 y+180 220 y+200]);
set_param([mdl '/From_theta_meas_log'], 'GotoTag', 'theta_meas');
add_line(mdl, 'From_theta_meas_log/1', 'Log_pos/1');

save_system(mdl, fullfile(pwd, [mdl '.slx']));
fprintf('Model saved: %s.slx\n', mdl);
fprintf('Signal labels used (Goto/From):\n');
fprintf('  w_ref, w_meas, omega_e, theta_e\n');
fprintf('  id_ref, iq_ref, id_meas, iq_meas\n');
fprintf('  vd_ref, vq_ref, da, db, dc\n');
fprintf('  ia, ib, ic, Vabc_out, T_load\n');
fprintf('  w_kf, theta_kf (Kalman estimates)\n');
end