function cfg = default_config()
% Default configuration for speed/Iq vs DC-bus-current efficiency study.

cfg = struct();
cfg.model_name = 'speedloop_kf_test';
cfg.output_tag = '';
cfg.speed_rpm_list = [500];
cfg.iq_target_a_list = [0.5 1:1:10];
cfg.stop_time_s = 0.35;
cfg.speed_step_time_s = 0.02;
cfg.eval_start_s = 0.25;
cfg.eval_end_s = [];
cfg.speed_tolerance_pct = 5.0;
cfg.iq_tolerance_a = 0.25;
cfg.iq_tolerance_pct = 8.0;
cfg.include_negative_iq = false;
cfg.drive_efficiency = [];

% Customer motor inputs. The study treats resistance/inductance as
% phase-to-phase measured values and converts them to phase values inside
% run_study.
cfg.motor = struct();
cfg.motor.pole_pairs = 10;
cfg.motor.kv_vrms_per_krpm = 8.74;          % line-line RMS back-EMF, per mechanical krpm
cfg.motor.line_to_line_resistance = 0.1067;  % ohm, measured phase-to-phase
cfg.motor.line_to_line_inductance = 0.145e-3; % H, measured phase-to-phase
cfg.motor.J = 2.5e-4;                        % kg*m^2
cfg.motor.B = 1.0e-4;                        % N*m/(rad/s)
cfg.motor.max_speed_rpm = 2500;

cfg.inverter = struct();
cfg.control = struct();
cfg.simcfg = struct();
cfg.plot_results = true;
cfg.save_outputs = true;
cfg.close_model = true;
end
