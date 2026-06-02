function [tau, state] = controller_dob_pd(cfg, state, ref, measurement)
%CONTROLLER_DOB_PD PD position control plus a load-torque disturbance observer.
%
% Plant convention:
%   J*qddot + b*qdot = tau - tau_load
%
% Therefore:
%   tau_load = tau - J*qddot - b*qdot

Ts = cfg.sim.Ts;
omega_o = 2*pi*cfg.dob_pd.observer_bw_hz;
alpha = 1 - exp(-omega_o*Ts);

raw_load = measurement.prev_tau_applied ...
    - cfg.dob_pd.Jn*measurement.qddot ...
    - cfg.dob_pd.bn*measurement.qdot;

state.tau_load_hat = state.tau_load_hat + alpha*(raw_load - state.tau_load_hat);
state.tau_load_hat = clamp(state.tau_load_hat, ...
    -cfg.dob_pd.tau_hat_limit, cfg.dob_pd.tau_hat_limit);

e = ref.q - measurement.q;
tau_pd = cfg.dob_pd.Kp*e - cfg.dob_pd.Kd*measurement.qdot;

tau = tau_pd + state.tau_load_hat;
end

function y = clamp(x, lower, upper)
    y = min(max(x, lower), upper);
end
