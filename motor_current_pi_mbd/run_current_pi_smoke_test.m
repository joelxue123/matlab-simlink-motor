%% Smoke test for current_pi_model
%
% Builds and simulates the block-diagram current PI controller. The test checks
% that outputs are finite and respect the dynamic DC-bus voltage limit.

clear; clc;

script_dir = fileparts(mfilename('fullpath'));
run(fullfile(script_dir, 'build_current_pi_model.m'));

cfg = evalin('base', 'current_pi_config');
model = cfg.modelName;
model_file = fullfile(script_dir, [model '.slx']);

load_system(model_file);

sim_out = sim(model, ...
    'ReturnWorkspaceOutputs', 'on', ...
    'SaveOutput', 'off');

vd = sim_out.get('log_vd_ref').Data;
vq = sim_out.get('log_vq_ref').Data;

defaults = evalin('base', 'current_pi_defaults');
test_input = evalin('base', 'current_pi_test');
v_limit = double(defaults.VLimitRatio) * double(test_input.vdc);
limit_tolerance = 1e-4;

values = [double(vd(:)); double(vq(:))];

fprintf('\nCurrent PI smoke test result:\n');
fprintf('  vd_ref range = [%.6g, %.6g] V\n', min(double(vd(:))), max(double(vd(:))));
fprintf('  vq_ref range = [%.6g, %.6g] V\n', min(double(vq(:))), max(double(vq(:))));
fprintf('  v_limit      = %.6g V\n', v_limit);

if any(~isfinite(values))
    error('Current PI smoke test failed: non-finite output detected.');
end

if any(abs(values) > v_limit + limit_tolerance)
    error('Current PI smoke test failed: output exceeded voltage limit.');
end

if max(abs(double(vq(:)))) < 0.5 * v_limit
    error('Current PI smoke test failed: q-axis output did not respond as expected.');
end

fprintf('Current PI smoke test passed.\n');
