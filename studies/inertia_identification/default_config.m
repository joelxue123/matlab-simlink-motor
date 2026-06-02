function cfg = default_config()
% Default configuration for the inertia identification study.

cfg = struct();
cfg.model_name = 'speedloop_kf_test';
cfg.inertia_scale = 1.0;
cfg.inertia_scale_list = [];
cfg.speed_source = 'wkf';
cfg.redesign_speed_pi = false;
cfg.step_time_s = 0.02;
cfg.step_down_time_s = 0.30;
cfg.stop_time_s = 0.5;
cfg.plot_results = true;
cfg.save_outputs = true;
cfg.close_model = true;
end