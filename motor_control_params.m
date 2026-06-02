
%% Motor control baseline parameters for an average-inverter plant
% This script prepares a practical starting point for SPMSM FOC simulation.
% Load it before opening or running the Simulink model.

clear motor inverter control simcfg

%% Simulation configuration
simcfg.stop_time = 0.5;
simcfg.Ts_plant = 25e-6;              % 植物模型步长 (逆变器 + 电机, 25us)
simcfg.Ts_power = simcfg.Ts_plant;    % alias (open-loop model compat)
simcfg.Ts_ctrl  = 50e-6;              % 电流环控制步长 (50us)
simcfg.Ts_speed = 1e-4;               % 速度环控制步长 (100us)

%% Inverter and DC bus
inverter.Vdc = 48;
inverter.modulation_limit = 0.577 * inverter.Vdc;
inverter.current_limit = 15;

%% Surface-mounted PMSM parameters
motor.type = 'SPMSM';
motor.topology = 'Surface Mount PMSM';
motor.pole_pairs = 4;
motor.Rs = 0.35;
motor.Ld = 1.8e-3;
motor.Lq = 1.8e-3;
motor.psi_f = 0.035;
motor.J = 2.5e-4;
motor.B = 1.0e-4;
motor.load_torque = 0;
motor.speed_ref_rpm = 400;

%% Base quantities
motor.speed_ref_mech_rad_s = motor.speed_ref_rpm * 2 * pi / 60;
motor.speed_ref_elec_rad_s = motor.speed_ref_mech_rad_s * motor.pole_pairs;
motor.torque_constant = 1.5 * motor.pole_pairs * motor.psi_f;
motor.saliency_ratio = motor.Lq / motor.Ld;

%% Current loop design
control.id_ref = 0;
control.current_bandwidth_hz = 800;
control.current_bandwidth_rad_s = 2 * pi * control.current_bandwidth_hz;

control.pi_id.Kp = motor.Ld * control.current_bandwidth_rad_s;
control.pi_id.Ki = motor.Rs * control.current_bandwidth_rad_s;
control.pi_iq.Kp = motor.Lq * control.current_bandwidth_rad_s;
control.pi_iq.Ki = motor.Rs * control.current_bandwidth_rad_s;

control.pi_id.output_limit = inverter.modulation_limit;
control.pi_iq.output_limit = inverter.modulation_limit;
control.iq_ref_limit = inverter.current_limit;

%% Speed loop design — 带宽法 (考虑执行延时)
% 等效延时: 电流环闭环时间常数 + 速度环采样/ZOH延时
control.tau_current_cl = 1 / (2 * pi * control.current_bandwidth_hz);  % 电流环闭环时间常数
control.tau_speed_delay = 1.5 * simcfg.Ts_speed;  % ZOH + 计算延时 (1.5 × Ts)
control.tau_sigma = control.tau_current_cl + control.tau_speed_delay;   % 总等效小时间常数
control.speed_feedback_source = 'w_kf';

% 速度环带宽受限于延时: ωbw ≤ 1/(3τσ)
control.speed_bandwidth_hz = 1 / (2 * pi * 3 * control.tau_sigma);
if strcmp(control.speed_feedback_source, 'w_kf')
    control.speed_bandwidth_hz = min(control.speed_bandwidth_hz, 40);
end
control.speed_bandwidth_rad_s = 2 * pi * control.speed_bandwidth_hz;
control.speed_damping = 1.0;

% 标准带宽法公式
control.pi_speed.Kp = ...
    2 * control.speed_damping * control.speed_bandwidth_rad_s * motor.J ...
    / motor.torque_constant;
control.pi_speed.Ki = ...
    control.speed_bandwidth_rad_s^2 * motor.J / motor.torque_constant;
control.pi_speed.output_limit = control.iq_ref_limit;
control.speed_ref_filter_tau = 1 / (2 * pi * control.speed_bandwidth_hz);  % 参考预滤波

%% Position loop design — P 控制器 (速度环提供积分作用)
simcfg.Ts_pos = 1e-3;                % 位置环步长 (1ms)
control.pos_ref_rad = 2*pi;          % 位置指令 (rad, 机械角)
control.pos_step_time = 0.01;        % 位置阶跃时间 (s)
control.pos_ref_mode = 'step';       % 'step' | 'chirp' | 'sine'
control.pos_use_planner = false;      % true: 位置阶跃先经过轨迹规划, false: 直接位置步进
control.pos_chirp.amplitude_rad = deg2rad(2.0);
control.pos_chirp.f0_hz = 0.2;
control.pos_chirp.f1_hz = 25.0;
control.pos_chirp.start_time = 0.05;
control.pos_chirp.duration = 8.0;
control.pos_chirp.offset_rad = 0.0;
control.pos_sine.amplitude_rad = deg2rad(1.0);
control.pos_sine.freq_hz = 1.0;
control.pos_sine.start_time = 0.05;
control.pos_sine.offset_rad = 0.0;
control.pos_scan.start_time = 0.05;
control.pos_scan.hold_time = 0.04;
control.pos_scan.points = 180;
control.pos_scan.theta_table = zeros(360, 1);
% 位置环带宽 ≤ 速度环带宽 / 3~5
control.pos_bandwidth_hz = control.speed_bandwidth_hz / 4;
control.pos_bandwidth_rad_s = 2 * pi * control.pos_bandwidth_hz;
control.pos_controller_mode = 'pid_reg3';
% P 控制器: Kp_pos = ωn_pos, 输出为速度指令 (rad/s)
control.pi_pos.Kp = control.pos_bandwidth_rad_s;
control.pi_pos.output_limit = motor.speed_ref_mech_rad_s * 1;  % 速度限幅
% PIDREG3 位置控制器: 输出为机械角速度指令 (rad/s)
control.pid_pos.damping = 1.0;
control.pid_pos.Kp = 2 * control.pid_pos.damping * control.pos_bandwidth_rad_s;
control.pid_pos.Ki_cont = 0;
control.pid_pos.Ki = 0;
control.pid_pos.Kc = 0.5;
control.pid_pos.output_limit = control.pi_pos.output_limit;
% 梯形速度规划参数 (限速/限加速度, 消除阶跃导致的超调)
control.pos_max_vel = motor.speed_ref_mech_rad_s;      % 最大速度 (rad/s)
control.pos_max_acc = control.pos_max_vel / 0.02;      % 最大加速度 (20ms 加速到满速)

%% Kalman filter speed estimator
control.kf.q_theta = 1e-8;      % 位置过程噪声 (保持很小, 避免位置状态漂移)
control.kf.q_omega = 10;         % 速度过程噪声 (50us 观测下的折中最优点)
control.kf.r_theta = 1e-4;      % 位置测量噪声
control.kf.test_noise_var = 1e-5; % 测试模型注入的位置噪声方差

%% Vibration compensation test parameters
control.vib.mode = 'online';
control.vib.enable_learning = 1;
control.vib.enable_ff = 1;
control.vib.table_points = 72;
control.vib.learning_rate = 0.03;
control.vib.phase_advance_deg = 0;
control.vib.output_limit = 0.35 * control.iq_ref_limit;
control.vib.mean_alpha = 0.002;
control.vib.min_speed_abs = 0.80 * motor.speed_ref_mech_rad_s;
control.vib.speed_err_threshold = 6.0;
control.vib.learn_start_time = 0.15;
control.vib.ff_enable_time = 0.55;
control.vib.test_stop_time = 1.0;
control.vib.ff_table = zeros(360, 1);
control.vib.ff_table_file = 'vib_ff_table.mat';
control = apply_cogging_load_config(control, cogging_load_config);

%% Feedforward and decoupling
control.enable_decoupling = true;
control.enable_speed_loop = true;

control.vd_decoupling = @(id, iq, omega_e) -omega_e * motor.Lq * iq;
control.vq_decoupling = @(id, iq, omega_e) omega_e * (motor.Ld * id + motor.psi_f);

%% Reference ramps
control.speed_ramp_time = 0.05;
control.load_step_time = 0.2;
control.load_step_torque = 0.0;
control.use_periodic_load = false;

%% Notes for Simulink mapping
% motor.*      -> motor plant block or mask parameters
% inverter.*   -> average inverter DC bus and saturation limits
% control.*    -> speed PI, current PI, decoupling, and references
% simcfg.*     -> solver step sizes and stop time
% For SPMSM, Ld and Lq are kept equal by design in this baseline.

%% Open-loop V/F test parameters
% 目标转速 = freq_hz * 60 / pole_pairs (RPM)
% 例: freq_hz=66.7, pole_pairs=4 → 1000 RPM
openloop.vd = 0;            % d轴电压 (V)
openloop.vq = 5;            % q轴电压 (V), 决定电磁力矩大小
openloop.freq_hz = motor.speed_ref_rpm * motor.pole_pairs / 60;  % 电频率 (Hz)