function build_openloop_test()
% 开环测试模型：固定 vd/vq + 线性增长 theta_e → dq2abc → 逆变器 → PMSM
% 用于验证电机是否能在开环下正常旋转。
%
% 使用方法:
%   motor_control_params;
%   build_openloop_test;
%   open_system('openloop_vf_test');
%   sim('openloop_vf_test');

mdl = 'openloop_vf_test';

load_system('mcbplantlib');
load_system('mcblib');

if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end

new_system(mdl);
set_param(mdl, 'Solver', 'FixedStepAuto', ...
    'FixedStep', 'simcfg.Ts_power', ...
    'StopTime', '2', ...
    'InitFcn', 'motor_control_params');

% =====================================================================
% 开环参数 (可在 workspace 中修改)
% =====================================================================
%   openloop.vd      — d轴电压 (V), 通常设 0
%   openloop.vq      — q轴电压 (V), 控制输出力矩大小
%   openloop.freq_hz — 电角频率 (Hz), 决定目标转速
%   目标机械转速 = freq_hz * 60 / pole_pairs (RPM)

% =====================================================================
% Row 1: vd, vq 恒定输入
% =====================================================================
y = 50;
add_block('simulink/Sources/Constant', [mdl '/Vd_const'], ...
    'Position', [50 y 120 y+25], 'Value', 'openloop.vd');
add_goto(mdl, 'vd_ref', [160 y 230 y+25], 'Vd_const/1');

add_block('simulink/Sources/Constant', [mdl '/Vq_const'], ...
    'Position', [50 y+50 120 y+75], 'Value', 'openloop.vq');
add_goto(mdl, 'vq_ref', [160 y+50 230 y+75], 'Vq_const/1');

% =====================================================================
% Row 2: theta_e = 2*pi*freq_hz * t (线性增长, mod 2*pi)
% =====================================================================
y = 180;
% 用 Ramp 生成 omega_e * t, 斜率 = 2*pi*freq_hz
add_block('simulink/Sources/Ramp', [mdl '/ThetaRamp'], ...
    'Position', [50 y 120 y+30], ...
    'slope', '2*pi*openloop.freq_hz', ...
    'start', '0', 'InitialOutput', '0');

% mod 2*pi 防止数值溢出
add_block('simulink/Math Operations/Math Function', [mdl '/Mod2pi'], ...
    'Position', [160 y 220 y+30], 'Operator', 'mod');
add_block('simulink/Sources/Constant', [mdl '/TwoPi'], ...
    'Position', [100 y+45 150 y+65], 'Value', '2*pi');
add_line(mdl, 'ThetaRamp/1', 'Mod2pi/1');
add_line(mdl, 'TwoPi/1', 'Mod2pi/2');
add_goto(mdl, 'theta_e', [260 y 330 y+25], 'Mod2pi/1');

% =====================================================================
% Row 3: dq → abc (逆Park + 逆Clarke + SVPWM + 占空比)
% =====================================================================
y = 300;
create_subsystem(mdl, 'dq to abc', [100 y 280 y+120], ...
    {'vd_ref', 'vq_ref', 'theta_e', 'Vdc'}, {'da', 'db', 'dc'});
populate_dq2abc(mdl);

add_from(mdl, 'vd_ref',  [20 y+5   70 y+25]);
add_from(mdl, 'vq_ref',  [20 y+35  70 y+55]);
add_from(mdl, 'theta_e', [20 y+70  70 y+90]);
add_block('simulink/Sources/Constant', [mdl '/Vdc_dq2abc'], ...
    'Position', [20 y+100 70 y+120], 'Value', 'inverter.Vdc');
add_line(mdl, 'From_vd_ref/1',  'dq to abc/1');
add_line(mdl, 'From_vq_ref/1',  'dq to abc/2');
add_line(mdl, 'From_theta_e/1', 'dq to abc/3');
add_line(mdl, 'Vdc_dq2abc/1',   'dq to abc/4');
add_goto(mdl, 'da', [300 y     380 y+20],  'dq to abc/1');
add_goto(mdl, 'db', [300 y+35  380 y+55],  'dq to abc/2');
add_goto(mdl, 'dc', [300 y+70  380 y+90],  'dq to abc/3');

% =====================================================================
% Row 4: 占空比 [0,1] → 调制指数 [-1,+1] → Average Inverter
% =====================================================================
y = 460;
add_from(mdl, 'da', [20 y     70 y+20]);
add_from(mdl, 'db', [20 y+40  70 y+60]);
add_from(mdl, 'dc', [20 y+80  70 y+100]);

add_block('simulink/Signal Routing/Mux', [mdl '/Mux_Duty'], ...
    'Position', [100 y+10 105 y+90], 'Inputs', '3');
add_line(mdl, 'From_da/1', 'Mux_Duty/1');
add_line(mdl, 'From_db/1', 'Mux_Duty/2');
add_line(mdl, 'From_dc/1', 'Mux_Duty/3');

% duty [0,1] → mod [-1,+1]:  mod = 2*duty - 1
add_block('simulink/Math Operations/Gain', [mdl '/Duty2Mod'], ...
    'Position', [130 y+30 175 y+70], 'Gain', '2');
add_block('simulink/Math Operations/Bias', [mdl '/BiasN1'], ...
    'Position', [200 y+30 245 y+70], 'Bias', '-1');
add_line(mdl, 'Mux_Duty/1', 'Duty2Mod/1');
add_line(mdl, 'Duty2Mod/1', 'BiasN1/1');

add_block('simulink/Sources/Constant', [mdl '/Vdc'], ...
    'Position', [200 y+110 250 y+130], 'Value', 'inverter.Vdc');

add_block('mcbplantlib/Average-Value Inverter', [mdl '/Average Inverter'], ...
    'Position', [280 y 440 y+90]);
add_line(mdl, 'BiasN1/1', 'Average Inverter/1');
add_line(mdl, 'Vdc/1', 'Average Inverter/2');
add_goto(mdl, 'Vabc_out', [480 y+20 560 y+45], 'Average Inverter/1');

% =====================================================================
% Row 5: PMSM 电机模型
% =====================================================================
y = 630;
add_block('simulink/Sources/Constant', [mdl '/LoadTorque'], ...
    'Position', [50 y 120 y+25], 'Value', 'motor.load_torque');

add_from(mdl, 'Vabc_out', [20 y+60 70 y+80]);
add_block('mcblib/Electrical Systems/Motors/Surface Mount PMSM', ...
    [mdl '/Surface Mount PMSM'], 'Position', [100 y+40 310 y+200]);
set_param([mdl '/Surface Mount PMSM'], ...
    'port_config', 'Torque', ...
    'sim_type', 'Discrete', ...
    'Ts', 'simcfg.Ts_power', ...
    'P', 'motor.pole_pairs', ...
    'Rs', 'motor.Rs', ...
    'Ldq_', 'motor.Ld', ...
    'lambda_pm', 'motor.psi_f', ...
    'mechanical', '[motor.J, motor.B, 0]', ...
    'idq0', '[0 0]', ...
    'theta_init', '0', ...
    'omega_init', '0');
add_line(mdl, 'LoadTorque/1',    'Surface Mount PMSM/1');
add_line(mdl, 'From_Vabc_out/1', 'Surface Mount PMSM/2');

% Demux Iabc
add_block('simulink/Signal Routing/Demux', [mdl '/Demux_Iabc'], ...
    'Position', [350 y+95 355 y+175], 'Outputs', '3');
add_line(mdl, 'Surface Mount PMSM/2', 'Demux_Iabc/1');
add_goto(mdl, 'ia', [390 y+80  450 y+100], 'Demux_Iabc/1');
add_goto(mdl, 'ib', [390 y+115 450 y+135], 'Demux_Iabc/2');
add_goto(mdl, 'ic', [390 y+150 450 y+170], 'Demux_Iabc/3');

% wm
add_goto(mdl, 'w_meas', [350 y+190 430 y+210], 'Surface Mount PMSM/3');

% =====================================================================
% Row 6: Scope 输出
% =====================================================================
y = 900;

% 转速
add_from(mdl, 'w_meas_scope', [20 y 70 y+20]);
set_param([mdl '/From_w_meas_scope'], 'GotoTag', 'w_meas');
add_block('simulink/Sinks/Scope', [mdl '/Speed Scope'], ...
    'Position', [120 y-5 170 y+25], 'NumInputPorts', '1');
add_line(mdl, 'From_w_meas_scope/1', 'Speed Scope/1');

% 三相电流
add_from(mdl, 'ia_scope', [20 y+50 70 y+70]);
set_param([mdl '/From_ia_scope'], 'GotoTag', 'ia');
add_from(mdl, 'ib_scope', [20 y+80 70 y+100]);
set_param([mdl '/From_ib_scope'], 'GotoTag', 'ib');
add_from(mdl, 'ic_scope', [20 y+110 70 y+130]);
set_param([mdl '/From_ic_scope'], 'GotoTag', 'ic');
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_Iabc'], ...
    'Position', [100 y+60 105 y+120], 'Inputs', '3');
add_line(mdl, 'From_ia_scope/1', 'Mux_Iabc/1');
add_line(mdl, 'From_ib_scope/1', 'Mux_Iabc/2');
add_line(mdl, 'From_ic_scope/1', 'Mux_Iabc/3');
add_block('simulink/Sinks/Scope', [mdl '/Current abc Scope'], ...
    'Position', [140 y+65 190 y+115], 'NumInputPorts', '1');
add_line(mdl, 'Mux_Iabc/1', 'Current abc Scope/1');

% 给定角度 theta_e
add_from(mdl, 'theta_e_scope', [20 y+155 70 y+175]);
set_param([mdl '/From_theta_e_scope'], 'GotoTag', 'theta_e');
add_block('simulink/Sinks/Scope', [mdl '/Theta Scope'], ...
    'Position', [120 y+150 170 y+180], 'NumInputPorts', '1');
add_line(mdl, 'From_theta_e_scope/1', 'Theta Scope/1');

% Log to workspace
add_block('simulink/Sinks/To Workspace', [mdl '/Log_wm'], ...
    'Position', [120 y+200 190 y+225], ...
    'VariableName', 'log_wm', 'SaveFormat', 'Timeseries');
add_from(mdl, 'w_meas_log', [20 y+200 70 y+220]);
set_param([mdl '/From_w_meas_log'], 'GotoTag', 'w_meas');
add_line(mdl, 'From_w_meas_log/1', 'Log_wm/1');

add_block('simulink/Sinks/To Workspace', [mdl '/Log_Iabc'], ...
    'Position', [120 y+240 190 y+265], ...
    'VariableName', 'log_iabc', 'SaveFormat', 'Timeseries');
add_from(mdl, 'ia_log', [20 y+240 70 y+260]);
set_param([mdl '/From_ia_log'], 'GotoTag', 'ia');
add_line(mdl, 'From_ia_log/1', 'Log_Iabc/1');

save_system(mdl, fullfile(pwd, [mdl '.slx']));
fprintf('Open-loop test model saved: %s.slx\n', mdl);
fprintf('\n');
fprintf('使用方法:\n');
fprintf('  1. 在 motor_control_params.m 运行后设置开环参数:\n');
fprintf('     openloop.vd = 0;          %% d轴电压\n');
fprintf('     openloop.vq = 5;          %% q轴电压 (V)\n');
fprintf('     openloop.freq_hz = 66.7;  %% 电频率 (Hz) → 1000 RPM @ 4 pole pairs\n');
fprintf('  2. sim(''openloop_vf_test'');\n');
fprintf('  3. 看 Speed Scope 和 Current abc Scope\n');
end

%% ===== Helper functions (same as build_average_inverter_foc_model) =====

function add_goto(mdl, tag, pos, src_port)
    blk = [mdl '/Goto_' tag];
    add_block('simulink/Signal Routing/Goto', blk, ...
        'Position', pos, 'GotoTag', tag, 'TagVisibility', 'global');
    if nargin >= 4 && ~isempty(src_port)
        add_line(mdl, src_port, ['Goto_' tag '/1']);
    end
end

function add_from(mdl, tag, pos)
    blk = [mdl '/From_' tag];
    add_block('simulink/Signal Routing/From', blk, ...
        'Position', pos, 'GotoTag', tag);
end

function create_subsystem(mdl, name, pos, inports, outports)
    path = [mdl '/' name];
    add_block('simulink/Ports & Subsystems/Subsystem', path, 'Position', pos);
    lines = find_system(path, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line');
    if ~isempty(lines), delete_line(lines); end
    blks = find_system(path, 'SearchDepth', 1, 'Type', 'Block');
    for k = 2:numel(blks), delete_block(blks{k}); end
    for k = 1:numel(inports)
        p = [path '/' inports{k}];
        add_block('simulink/Sources/In1', p, ...
            'Position', [30 35+45*(k-1) 60 49+45*(k-1)]);
        set_param(p, 'Port', num2str(k));
    end
    for k = 1:numel(outports)
        p = [path '/' outports{k}];
        add_block('simulink/Sinks/Out1', p, ...
            'Position', [260 35+45*(k-1) 290 49+45*(k-1)]);
        set_param(p, 'Port', num2str(k));
    end
end

function populate_dq2abc(mdl)
    path = [mdl '/dq to abc'];
    fcn_blk = [path '/dq2abc_fcn'];
    add_block('simulink/User-Defined Functions/MATLAB Function', fcn_blk, ...
        'Position', [100 30 220 120]);
    rt = sfroot;
    chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', fcn_blk);
    chart.Script = sprintf([...
        'function [da, db, dc] = dq2abc_fcn(vd, vq, theta_e, Vdc)\n' ...
        '%% dq → 占空比: 逆Park + 逆Clarke + SVPWM + 归一化\n' ...
        '\n' ...
        '%% 1. 逆Park: dq → αβ\n' ...
        'v_alpha = vd * cos(theta_e) - vq * sin(theta_e);\n' ...
        'v_beta  = vd * sin(theta_e) + vq * cos(theta_e);\n' ...
        '\n' ...
        '%% 2. 逆Clarke: αβ → abc\n' ...
        'va = v_alpha;\n' ...
        'vb = -0.5 * v_alpha + sqrt(3)/2 * v_beta;\n' ...
        'vc = -0.5 * v_alpha - sqrt(3)/2 * v_beta;\n' ...
        '\n' ...
        '%% 3. SVPWM零序注入\n' ...
        'v_max = max(max(va, vb), vc);\n' ...
        'v_min = min(min(va, vb), vc);\n' ...
        'v_n0  = -0.5 * (v_max + v_min);\n' ...
        'va = va + v_n0;\n' ...
        'vb = vb + v_n0;\n' ...
        'vc = vc + v_n0;\n' ...
        '\n' ...
        '%%%% 4. 占空比 [0,1]\n' ...
        'da = va / Vdc + 0.5;\n' ...
        'db = vb / Vdc + 0.5;\n' ...
        'dc = vc / Vdc + 0.5;\n' ...
    ]);
    add_line(path, 'vd_ref/1',  'dq2abc_fcn/1');
    add_line(path, 'vq_ref/1',  'dq2abc_fcn/2');
    add_line(path, 'theta_e/1', 'dq2abc_fcn/3');
    add_line(path, 'Vdc/1',     'dq2abc_fcn/4');
    add_line(path, 'dq2abc_fcn/1', 'da/1');
    add_line(path, 'dq2abc_fcn/2', 'db/1');
    add_line(path, 'dq2abc_fcn/3', 'dc/1');
end
