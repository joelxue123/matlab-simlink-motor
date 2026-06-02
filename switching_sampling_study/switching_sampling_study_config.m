
%% Switching sampling study configuration

clear ss_cfg

ss_cfg.model_name = 'switching_sampling_study_model';

% DC bus and PWM settings
ss_cfg.Vdc = 48;
ss_cfg.f_pwm = 20e3;
ss_cfg.T_pwm = 1 / ss_cfg.f_pwm;
ss_cfg.dead_time_s = 100e-9;

% Reference settings
ss_cfg.modulation_ratio = 0.1;
ss_cfg.electrical_freq_hz = 80;
ss_cfg.theta_e_deg = 60;
ss_cfg.samples_per_pwm_model = 4000;
ss_cfg.sim_stop_time = 2 * ss_cfg.T_pwm;

% Sampling study assumptions
ss_cfg.adc_settle_time_s = 1.0e-6;
ss_cfg.adc_trigger_offset_s = 2.0e-6;
ss_cfg.min_valid_window_s = 2.5e-6;
ss_cfg.duty_sample_limit = 0.95;

% RL-load study settings
ss_cfg.rl_load_R_ohm = 10;
ss_cfg.rl_load_L_h = 200e-6;
ss_cfg.rl_study_periods = 40;
ss_cfg.rl_rotating_cycles = 1;

% SPS power-stage settings
ss_cfg.powergui_sample_time_s = 12.5e-9;
ss_cfg.pmsm_speed_ref_rad_s = 0.0;
ss_cfg.model_fixed_step_s = min(ss_cfg.T_pwm / ss_cfg.samples_per_pwm_model, ss_cfg.powergui_sample_time_s);
% First-stage window study uses only dead time and settling time. Trigger
% offset is reserved for the next step when explicit ADC timing is added.

% Zero-vector allocation study cases
ss_cfg.v0v7_splits = [0.0, 0.5, 1.0];
ss_cfg.case_names = {'All V0', 'Symmetric', 'All V7'};