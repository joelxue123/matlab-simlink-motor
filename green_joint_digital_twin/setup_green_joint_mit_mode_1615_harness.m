%% Setup green-joint 1615 MIT mode harness parameters
%
% Run this before opening or simulating green_joint_mit_mode_1615_harness.

gjdt_mit_setup_script_dir = fileparts(mfilename('fullpath'));
run(fullfile(gjdt_mit_setup_script_dir, 'setup_green_joint_current_loop_twin.m'));

GJDT_MIT_StopTime_s = 0.45;
GJDT_MIT_PosStepTime_s = 0.020;
GJDT_MIT_PosTarget_Rad = single(0.20);
GJDT_MIT_VelTarget_RadS = single(0.0);
GJDT_MIT_FfTorque_Nm = single(0.0);

GJDT_MIT_KtOutput_NmPerA = single(motor.torque_constant * motor.gear_ratio);
GJDT_MIT_TorqueToIq_APerNm = single(1.0 / ...
    double(GJDT_MIT_KtOutput_NmPerA));

GJDT_MIT_JOutput_KgM2 = single(motor.output_equivalent_inertia_kg_m2);
GJDT_MIT_BOutput_NmSPerRad = single( ...
    motor.output_viscous_damping_nm_s_per_rad);
GJDT_MIT_TcOutput_Nm = single(motor.output_coulomb_friction_nm);
GJDT_MIT_TbiasOutput_Nm = single(motor.output_torque_bias_nm);
GJDT_MIT_FrictionSmoothing_RadS = single(0.02);

bandwidth_hz = 15.0;
zeta = 1.0;
wn = 2 * pi * bandwidth_hz;
j_output = double(GJDT_MIT_JOutput_KgM2);
b_output = double(GJDT_MIT_BOutput_NmSPerRad);
kt_output = double(GJDT_MIT_KtOutput_NmPerA);
kp_nm_per_rad = j_output * wn ^ 2;
kd_nm_s_per_rad = max(0.0, 2 * zeta * j_output * wn - b_output);

GJDT_MIT_Kp_APerRad = single(kp_nm_per_rad / kt_output);
GJDT_MIT_Kd_APerRadS = single(kd_nm_s_per_rad / kt_output);
GJDT_MIT_IqLimit_A = single(GJDT_ModuleConfig.defaults.torque_limit_a);

GJDT_MIT_CurrentTau_s = 1.0 / (2 * pi * GJDT_CurrentBandwidth_Hz);
GJDT_MIT_CurrentAlpha = single(1.0 - exp(-GJDT_Ts / ...
    GJDT_MIT_CurrentTau_s));

clear bandwidth_hz zeta wn j_output b_output kt_output;
clear kp_nm_per_rad kd_nm_s_per_rad gjdt_mit_setup_script_dir;
