function cfg = config_default()
%CONFIG_DEFAULT Default parameters for the single-joint dynamics lab.

cfg.plant.J = 0.01;              % kg*m^2
cfg.plant.b = 0.02;              % N*m*s/rad
cfg.plant.Kt = 0.08;             % N*m/A
cfg.plant.R = 1.2;               % Ohm

cfg.sim.Ts = 0.001;              % s, 1 kHz control sample
cfg.sim.T_end = 2.0;             % s
cfg.sim.q0 = 0.0;                % rad
cfg.sim.qdot0 = 0.0;             % rad/s
cfg.sim.tau_limit = 1.5;         % N*m

cfg.ref.step_time = 0.1;         % s
cfg.ref.q_initial = 0.0;         % rad
cfg.ref.q_final = 1.0;           % rad

cfg.load.step_time = 0.8;        % s
cfg.load.tau_value = 0.25;       % N*m, positive load opposes positive motion

% Position PID. D term is measured-velocity feedback, not derivative of error.
cfg.pid.Kp = 8.0;
cfg.pid.Ki = 6.0;
cfg.pid.Kd = 0.45;
cfg.pid.integral_limit = 0.4;

% DOB + PD. The observer estimates tau_load from nominal dynamics.
cfg.dob_pd.Kp = 8.0;
cfg.dob_pd.Kd = 0.45;
cfg.dob_pd.Jn = cfg.plant.J;
cfg.dob_pd.bn = cfg.plant.b;
cfg.dob_pd.observer_bw_hz = 35.0;
cfg.dob_pd.tau_hat_limit = 0.8;

% Impedance controller in joint space.
cfg.impedance.K = 4.0;           % N*m/rad
cfg.impedance.zeta = 1.0;
cfg.impedance.D = 2*cfg.impedance.zeta*sqrt(cfg.plant.J*cfg.impedance.K) - cfg.plant.b;

cfg.controllers = {'pid', 'dob_pd', 'impedance'};
cfg.delay_scan_samples = 0:5;
end
