function result = run_customer_example()
% Customer-facing template for the Iq/DC-bus-current efficiency study.
%
% Edit the input block below, then run:
%   result = run_customer_example;

cfg = default_config();

%% Output label
cfg.output_tag = 'customer_motor_68V';

%% Inverter and drive inputs
cfg.inverter.Vdc = 68;                 % DC bus voltage (V)
cfg.inverter.current_limit = 20;        % current limit used by the controller (A)
cfg.inverter.drive_efficiency = 0.95;   % drive efficiency, 0.95 means 95%

%% Motor inputs
cfg.motor.pole_pairs = 10;
cfg.motor.kv_vrms_per_krpm = 17.03;            % line-line RMS back-EMF, per mechanical krpm
cfg.motor.line_to_line_resistance = 0.4267;    % ohm, measured phase-to-phase
cfg.motor.line_to_line_inductance = 0.53e-3;   % H, measured phase-to-phase
cfg.motor.J = 2.5e-4;                           % kg*m^2
cfg.motor.B = 1.0e-4;                           % N*m/(rad/s)
cfg.motor.max_speed_rpm = 4900;

%% Sweep inputs
cfg.speed_rpm_list = [500 1000 2000 2500];
cfg.iq_target_a_list = [0.5 1:1:10 12:2:20];

%% Simulation window
cfg.stop_time_s = 0.35;
cfg.eval_start_s = 0.25;
cfg.eval_end_s = [];

result = run_study(cfg);
disp(result.output_files);
end
