function fxp = init_controller_fixed_point_types(cfg)
%INIT_CONTROLLER_FIXED_POINT_TYPES Define fixed-point types and constants.
% The generated Simulink models use these names in block data type fields and
% MATLAB Function code. They are also assigned into the base workspace so that
% Simulink can resolve them during update/build.

fxp.T_angle = fixdt(1, 16, 12);
fxp.T_speed = fixdt(1, 16, 8);
fxp.T_accel = fixdt(1, 16, 4);
fxp.T_torque = fixdt(1, 16, 13);
fxp.T_torque_wide = fixdt(1, 16, 12);
fxp.T_gain = fixdt(1, 16, 10);
fxp.T_small_gain = fixdt(1, 16, 14);

fxp.Ts_fxp = fi(cfg.sim.Ts, 1, 16, 15);
fxp.tau_limit_fxp = fi(cfg.sim.tau_limit, 1, 16, 13);
fxp.integral_limit_fxp = fi(cfg.pid.integral_limit, 1, 16, 13);
fxp.tau_hat_limit_fxp = fi(cfg.dob_pd.tau_hat_limit, 1, 16, 13);

fxp.Kp_pid = fi(cfg.pid.Kp, 1, 16, 10);
fxp.Ki_pid = fi(cfg.pid.Ki, 1, 16, 10);
fxp.Kd_pid = fi(cfg.pid.Kd, 1, 16, 14);
fxp.Kp_dob = fi(cfg.dob_pd.Kp, 1, 16, 10);
fxp.Kd_dob = fi(cfg.dob_pd.Kd, 1, 16, 14);
fxp.Jn_dob = fi(cfg.dob_pd.Jn, 1, 16, 15);
fxp.bn_dob = fi(cfg.dob_pd.bn, 1, 16, 15);
fxp.alpha_dob = fi(1 - exp(-2*pi*cfg.dob_pd.observer_bw_hz*cfg.sim.Ts), 1, 16, 14);
fxp.K_imp = fi(cfg.impedance.K, 1, 16, 10);
fxp.D_imp = fi(cfg.impedance.D, 1, 16, 14);

names = fieldnames(fxp);
for i = 1:numel(names)
    assignin('base', names{i}, fxp.(names{i}));
end
end