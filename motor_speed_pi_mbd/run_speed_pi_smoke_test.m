%% Smoke test for speed_pi_model
%
% Builds and simulates the speed PI controller with a simple mechanical plant.
% The test checks finite outputs, iq_ref limiting, and speed tracking.

clear; clc;

script_dir = fileparts(mfilename('fullpath'));
run(fullfile(script_dir, 'build_speed_pi_model.m'));

cfg = evalin('base', 'speed_pi_config');
model = cfg.modelName;
model_file = fullfile(script_dir, [model '.slx']);

load_system(model_file);

sim_out = sim(model, ...
    'ReturnWorkspaceOutputs', 'on', ...
    'SaveOutput', 'off');

wm_ref = sim_out.get('log_wm_ref').Data;
wm_meas = sim_out.get('log_wm_meas').Data;
iq_ref = sim_out.get('log_iq_ref').Data;

defaults = evalin('base', 'speed_pi_defaults');
iq_limit = double(defaults.test.iq_limit);
final_wm_ref = double(wm_ref(end));
final_wm_meas = double(wm_meas(end));
final_speed_error = final_wm_ref - final_wm_meas;
iq_values = double(iq_ref(:));
all_values = [double(wm_ref(:)); double(wm_meas(:)); iq_values];

fprintf('\nSpeed PI smoke test result:\n');
fprintf('  wm_ref final  = %.6g rad/s\n', final_wm_ref);
fprintf('  wm_meas final = %.6g rad/s\n', final_wm_meas);
fprintf('  speed error   = %.6g rad/s\n', final_speed_error);
fprintf('  iq_ref range  = [%.6g, %.6g] A\n', min(iq_values), max(iq_values));
fprintf('  iq_limit      = %.6g A\n', iq_limit);

if any(~isfinite(all_values))
    error('Speed PI smoke test failed: non-finite value detected.');
end

if any(abs(iq_values) > iq_limit + 1e-4)
    error('Speed PI smoke test failed: iq_ref exceeded iq_limit.');
end

if abs(final_speed_error) > 0.5
    error('Speed PI smoke test failed: final speed error is too large.');
end

if max(iq_values) < 0.1
    error('Speed PI smoke test failed: iq_ref did not respond.');
end

fprintf('Speed PI smoke test passed.\n');
