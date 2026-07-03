function mdl = run_switching_sampling_study()
%RUN_SWITCHING_SAMPLING_STUDY Load config and build the study scaffold.

switching_sampling_study_config;
mdl = build_switching_sampling_study_model(ss_cfg);

fprintf('Built model: %s\n', mdl);
fprintf('PWM frequency: %.1f kHz\n', ss_cfg.f_pwm / 1e3);
fprintf('Zero-vector cases: %s\n', strjoin(ss_cfg.case_names, ', '));

open_system(mdl);
end
