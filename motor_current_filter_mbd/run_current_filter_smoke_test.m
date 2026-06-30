%% Smoke test for current_filter_model
%
% Builds and simulates the dq current feedback filter. The test checks the
% low-voltage alpha clamp, a mid-range adaptive alpha point, and the basic
% step-response direction.

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

run(fullfile(script_dir, 'build_current_filter_model.m'));

model = 'current_filter_model';
model_file = fullfile(script_dir, [model '.slx']);

load_system(model_file);

low_case = run_case(model, single(0.1));
mid_case = run_case(model, single(0.65));

fprintf('\nCurrent filter smoke test result:\n');
fprintf('  low-v alpha = %.6g\n', low_case.final_alpha);
fprintf('  mid-v alpha = %.6g\n', mid_case.final_alpha);
fprintf('  mid-v id_f  = %.6g A\n', mid_case.final_id);
fprintf('  mid-v iq_f  = %.6g A\n', mid_case.final_iq);

assert_case(low_case, 0.95, false);
assert_case(mid_case, 0.95 - 1.125 * (0.65 - 0.5), true);

fprintf('Current filter smoke test passed.\n');

function result = run_case(model, v_mag_norm_value)
assignin('base', 'current_filter_test', struct( ...
    'id_raw', single(0), ...
    'iq_raw', single(10), ...
    'v_mag_norm', single(v_mag_norm_value)));

sim_out = sim(model, ...
    'ReturnWorkspaceOutputs', 'on', ...
    'SaveOutput', 'off');

id_f = logged_values(sim_out.get('log_id_f'));
iq_f = logged_values(sim_out.get('log_iq_f'));
alpha = logged_values(sim_out.get('log_alpha'));

result.final_id = double(id_f(end));
result.final_iq = double(iq_f(end));
result.final_alpha = double(alpha(end));
result.values = [double(id_f(:)); double(iq_f(:)); double(alpha(:))];
end

function assert_case(result, expected_alpha, check_tracking)
if any(~isfinite(result.values))
    error('Current filter smoke test failed: non-finite output detected.');
end

if abs(result.final_alpha - expected_alpha) > 1e-6
    error('Current filter smoke test failed: expected alpha %.6g, got %.6g.', ...
        expected_alpha, result.final_alpha);
end

if ~check_tracking
    return;
end

if abs(result.final_id) > 1e-6
    error('Current filter smoke test failed: id_f should stay near zero.');
end

if result.final_iq < 9.9 || result.final_iq > 10.1
    error('Current filter smoke test failed: iq_f did not converge toward 10 A.');
end
end

function values = logged_values(signal)
if isa(signal, 'timeseries')
    values = signal.Data;
    return;
end

if isstruct(signal) && isfield(signal, 'signals') && isfield(signal.signals, 'values')
    values = signal.signals.values;
    return;
end

error('Unsupported logged signal format.');
end
