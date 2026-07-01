%% Setup green-joint current-loop digital twin v0 parameters
%
% Run this before opening or simulating green_joint_current_loop_twin_model.
% The model intentionally keeps plant/harness parameters in base workspace so
% users can tune them interactively without editing the generated controller
% dictionary.

gjdt_setup_script_dir = fileparts(mfilename('fullpath'));
gjdt_setup_repo_dir = fileparts(gjdt_setup_script_dir);
gjdt_setup_workspace_dir = fileparts(gjdt_setup_repo_dir);
gjdt_setup_green_joint_fw_dir = fullfile(gjdt_setup_workspace_dir, ...
    'green-joint');
gjdt_setup_green_joint_mbd_dir = fullfile(gjdt_setup_repo_dir, ...
    'green_joint_current_loop_mbd');
if ~contains(path, gjdt_setup_green_joint_mbd_dir)
    addpath(gjdt_setup_green_joint_mbd_dir);
end
gjdt_setup_speed_mbd_dir = fullfile(gjdt_setup_repo_dir, ...
    'motor_speed_pi_mbd');
if ~contains(path, gjdt_setup_speed_mbd_dir)
    addpath(gjdt_setup_speed_mbd_dir);
end
gjdt_setup_speed_estimator_mbd_dir = fullfile(gjdt_setup_repo_dir, ...
    'motor_speed_estimator_mbd');
if ~contains(path, gjdt_setup_speed_estimator_mbd_dir)
    addpath(gjdt_setup_speed_estimator_mbd_dir);
end
gjdt_setup_mit_mbd_dir = fullfile(gjdt_setup_repo_dir, ...
    'green_joint_mit_impedance_mbd');
if ~contains(path, gjdt_setup_mit_mbd_dir)
    addpath(gjdt_setup_mit_mbd_dir);
end

GJDT_Ts = 50e-6;
GJDT_TsSpeed = 100e-6;
GJDT_TsPlant = 5e-6;
GJDT_StopTime = 0.030;

GJDT_MotorType = getenv('GJDT_MOTOR_TYPE');
if isempty(GJDT_MotorType)
    GJDT_MotorType = '1615';
end
GJDT_ModuleConfig = load_green_joint_module_config( ...
    gjdt_setup_green_joint_fw_dir, GJDT_MotorType);
GJDT_GearRatio = GJDT_ModuleConfig.gear_ratio;
GJDT_ControlModeCurrent = int32(0);
GJDT_ControlModeSpeed = int32(1);
GJDT_ControlModeMit = int32(2);
GJDT_ControlMode = GJDT_ControlModeCurrent;
GJDT_UseSpeedLoop = 0;
GJDT_Vbus_V = single(12.0);
GJDT_IdRef_A = single(0.0);
GJDT_IqStepTime_s = 0.001;
GJDT_IqBefore_A = single(0.0);
GJDT_IqAfter_A = single(1.5);
GJDT_SpeedRefStepTime_s = 0.005;
GJDT_SpeedRefBefore_rad_s = single(0.0);
GJDT_SpeedRefAfter_rad_s = single(4.0);
GJDT_SpeedIqLimit_A = single(4.0);
GJDT_MitPosStepTime_s = 0.020;
GJDT_MitPosBefore_Rad = single(0.0);
GJDT_MitPosAfter_Rad = single(0.20);
GJDT_MitVelTarget_RadS = single(0.0);
GJDT_MitFfTorque_Nm = single(0.0);
GJDT_MitBandwidth_Hz = 15.0;
GJDT_MitDampingRatio = 1.0;

line_to_line_resistance_ohm = GJDT_ModuleConfig.line_to_line_resistance_ohm;
line_to_line_inductance_h = GJDT_ModuleConfig.line_to_line_inductance_h;
rotor_inertia_kg_mm2 = GJDT_ModuleConfig.rotor_inertia_kg_m2 * 1e6;

GJDT_Rs_Ohm = GJDT_ModuleConfig.phase_resistance_ohm;
GJDT_Ld_H = GJDT_ModuleConfig.phase_inductance_h;
GJDT_Lq_H = GJDT_ModuleConfig.phase_inductance_h;
GJDT_CurrentLoopRs_Ohm = GJDT_ModuleConfig.current_loop.effective_phase_resistance_ohm;
GJDT_CurrentLoopLd_H = GJDT_ModuleConfig.current_loop.effective_phase_inductance_h;
GJDT_CurrentLoopLq_H = GJDT_ModuleConfig.current_loop.effective_phase_inductance_h;
GJDT_CurrentLoopDesignPhaseMargin_Deg = ...
    GJDT_ModuleConfig.current_loop.design_phase_margin_deg;
GJDT_CurrentLoopDesignDelay_s = GJDT_ModuleConfig.current_loop.design_delay_s;
GJDT_CurrentBandwidth_Hz = GJDT_ModuleConfig.current_loop.reference_bandwidth_hz;
GJDT_CurrentBandwidth_RadPerSec = 2 * pi * GJDT_CurrentBandwidth_Hz;
GJDT_CurDKp_Physical = single(GJDT_CurrentLoopLd_H ...
    * GJDT_CurrentBandwidth_RadPerSec);
GJDT_CurDKi_Physical = single(GJDT_CurrentLoopRs_Ohm ...
    * GJDT_CurrentBandwidth_RadPerSec);
GJDT_CurQKp_Physical = single(GJDT_CurrentLoopLq_H ...
    * GJDT_CurrentBandwidth_RadPerSec);
GJDT_CurQKi_Physical = single(GJDT_CurrentLoopRs_Ohm ...
    * GJDT_CurrentBandwidth_RadPerSec);
GJDT_CurDKp = single(GJDT_ModuleConfig.current_loop.cur_d_kp);
GJDT_CurDKi = single(GJDT_ModuleConfig.current_loop.cur_d_ki);
GJDT_CurQKp = single(GJDT_ModuleConfig.current_loop.cur_q_kp);
GJDT_CurQKi = single(GJDT_ModuleConfig.current_loop.cur_q_ki);

simcfg.stop_time = GJDT_StopTime;
simcfg.Ts_ctrl = GJDT_Ts;
simcfg.Ts_speed = GJDT_TsSpeed;
simcfg.Ts_plant = GJDT_TsPlant;

inverter.Vdc = double(GJDT_Vbus_V);
inverter.load_torque = 0;

motor.type = 'SPMSM';
motor.pole_pairs = GJDT_ModuleConfig.speed_estimator.pole_pairs;
motor.line_to_line_resistance = line_to_line_resistance_ohm;
motor.line_to_line_inductance = line_to_line_inductance_h;
motor.Rs = GJDT_Rs_Ohm;
motor.Ld = GJDT_Ld_H;
motor.Lq = GJDT_Lq_H;
motor.gear_ratio = GJDT_GearRatio;
motor.rotor_inertia_kg_mm2 = rotor_inertia_kg_mm2;
motor.rotor_inertia_kg_m2 = rotor_inertia_kg_mm2 * 1e-6;
motor.output_equivalent_inertia_kg_m2 = ...
    GJDT_ModuleConfig.mechanics.output_equivalent_inertia_kg_m2;
motor.output_viscous_damping_nm_s_per_rad = ...
    GJDT_ModuleConfig.mechanics.output_viscous_damping_nm_s_per_rad;
motor.output_coulomb_friction_nm = ...
    GJDT_ModuleConfig.mechanics.output_coulomb_friction_nm;
motor.output_torque_bias_nm = ...
    GJDT_ModuleConfig.mechanics.output_torque_bias_nm;
motor.output_load_inertia_kg_m2 = max(0.0, ...
    motor.output_equivalent_inertia_kg_m2 - ...
    motor.rotor_inertia_kg_m2 * motor.gear_ratio ^ 2);
motor.rated_current_a = 0.4949;
motor.rated_torque_nm = 2.56e-3;
motor.peak_current_a = 1.4847;
motor.peak_torque_nm = 7.33e-3;
motor.kt_rated_nm_per_a = motor.rated_torque_nm / motor.rated_current_a;
motor.kt_peak_nm_per_a = motor.peak_torque_nm / motor.peak_current_a;
motor.torque_constant = GJDT_ModuleConfig.torque_constant_nm_per_a;
motor.psi_f = motor.torque_constant / (1.5 * motor.pole_pairs);
motor.J = motor.output_equivalent_inertia_kg_m2 / ...
    (motor.gear_ratio ^ 2);
motor.B = motor.output_viscous_damping_nm_s_per_rad / ...
    (motor.gear_ratio ^ 2);
motor.speed_loop_equiv_inertia_kg_m2 = ...
    motor.output_equivalent_inertia_kg_m2 / motor.gear_ratio;
motor.speed_loop_equiv_damping_nm_s_per_rad = ...
    motor.output_viscous_damping_nm_s_per_rad / motor.gear_ratio;

mit_wn_rad_s = 2 * pi * GJDT_MitBandwidth_Hz;
mit_kp_nm_per_rad = motor.output_equivalent_inertia_kg_m2 ...
    * mit_wn_rad_s ^ 2;
mit_kd_nm_s_per_rad = max(0.0, ...
    2 * GJDT_MitDampingRatio * motor.output_equivalent_inertia_kg_m2 ...
    * mit_wn_rad_s - motor.output_viscous_damping_nm_s_per_rad);
GJDT_MitKtOutput_NmPerA = single(motor.torque_constant * motor.gear_ratio);
GJDT_MitTorqueToIq_APerNm = single(1.0 / ...
    double(GJDT_MitKtOutput_NmPerA));
GJDT_MitKp_NmPerRad = single(mit_kp_nm_per_rad);
GJDT_MitKd_NmSPerRad = single(mit_kd_nm_s_per_rad);
GJDT_MitKp_APerRad = single(mit_kp_nm_per_rad / ...
    double(GJDT_MitKtOutput_NmPerA));
GJDT_MitKd_APerRadS = single(mit_kd_nm_s_per_rad / ...
    double(GJDT_MitKtOutput_NmPerA));
GJDT_MitIqLimit_A = single(GJDT_ModuleConfig.defaults.torque_limit_a);

GJDT_OutputEquivalentInertia_kg_m2 = motor.output_equivalent_inertia_kg_m2;
GJDT_OutputViscousDamping_Nm_s_per_rad = ...
    motor.output_viscous_damping_nm_s_per_rad;
GJDT_OutputCoulombFriction_Nm = motor.output_coulomb_friction_nm;
GJDT_OutputTorqueBias_Nm = motor.output_torque_bias_nm;
GJDT_MotorShaftEquivalentInertia_kg_m2 = motor.J;
GJDT_MotorShaftViscousDamping_Nm_s_per_rad = motor.B;

clear line_to_line_resistance_ohm line_to_line_inductance_h rotor_inertia_kg_mm2;
clear mit_wn_rad_s mit_kp_nm_per_rad mit_kd_nm_s_per_rad;
clear gjdt_setup_script_dir gjdt_setup_repo_dir gjdt_setup_green_joint_mbd_dir;
clear gjdt_setup_speed_mbd_dir gjdt_setup_speed_estimator_mbd_dir;
clear gjdt_setup_mit_mbd_dir;
clear gjdt_setup_workspace_dir gjdt_setup_green_joint_fw_dir;

function cfg = load_green_joint_module_config(fw_dir, motor_type)
config_file = fullfile(fw_dir, 'Module', 'Config', ...
    ['green_joint_' char(motor_type) '_config.json']);
if ~exist(config_file, 'file')
    error('Missing green-joint module config: %s', config_file);
end

cfg = jsondecode(fileread(config_file));

expected_phase_r = cfg.line_to_line_resistance_ohm / 2.0;
expected_phase_l = cfg.line_to_line_inductance_h / 2.0;
if abs(cfg.phase_resistance_ohm - expected_phase_r) > 1e-9
    error('Invalid phase resistance in %s.', config_file);
end
if abs(cfg.phase_inductance_h - expected_phase_l) > 1e-12
    error('Invalid phase inductance in %s.', config_file);
end
if ~isfield(cfg.current_loop, 'effective_phase_resistance_ohm')
    cfg.current_loop.effective_phase_resistance_ohm = cfg.phase_resistance_ohm;
end
if ~isfield(cfg.current_loop, 'effective_phase_inductance_h')
    cfg.current_loop.effective_phase_inductance_h = cfg.phase_inductance_h;
end
if ~isfield(cfg.current_loop, 'design_phase_margin_deg')
    cfg.current_loop.design_phase_margin_deg = 60.0;
end
if ~isfield(cfg.current_loop, 'design_delay_s')
    cfg.current_loop.design_delay_s = 75e-6;
end
if cfg.current_loop.effective_phase_resistance_ohm <= 0
    error('Invalid effective current-loop resistance in %s.', config_file);
end
if cfg.current_loop.effective_phase_inductance_h <= 0
    error('Invalid effective current-loop inductance in %s.', config_file);
end
end
