%% Functional test for pwm_deadtime_sampling_model

clear; clc;

script_dir = fileparts(mfilename('fullpath'));
previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

run(fullfile(script_dir, 'build_pwm_deadtime_sampling_model.m'));

model = 'pwm_deadtime_sampling_model';
sim_out = sim(model, 'ReturnWorkspaceOutputs', 'on');

result.usable_low_a_s = get_timeseries_last(sim_out, 'usable_low_a_s');
result.usable_low_b_s = get_timeseries_last(sim_out, 'usable_low_b_s');
result.usable_low_c_s = get_timeseries_last(sim_out, 'usable_low_c_s');
result.sample_valid_a = logical(get_timeseries_last(sim_out, 'sample_valid_a'));
result.sample_valid_b = logical(get_timeseries_last(sim_out, 'sample_valid_b'));
result.sample_valid_c = logical(get_timeseries_last(sim_out, 'sample_valid_c'));
result.all_samples_valid = logical(get_timeseries_last(sim_out, 'all_samples_valid'));
result.min_usable_low_s = get_timeseries_last(sim_out, 'min_usable_low_s');

expected.usable_low_a_s = 45.5e-6;
expected.usable_low_b_s = 0.5e-6;
expected.usable_low_c_s = 23.0e-6;
expected.sample_valid_a = true;
expected.sample_valid_b = false;
expected.sample_valid_c = true;
expected.all_samples_valid = false;
expected.min_usable_low_s = 0.5e-6;

tol_s = 0.02e-6;
assert_close(result.usable_low_a_s, expected.usable_low_a_s, tol_s, 'usable_low_a_s');
assert_close(result.usable_low_b_s, expected.usable_low_b_s, tol_s, 'usable_low_b_s');
assert_close(result.usable_low_c_s, expected.usable_low_c_s, tol_s, 'usable_low_c_s');
assert_close(result.min_usable_low_s, expected.min_usable_low_s, tol_s, 'min_usable_low_s');

assert(result.sample_valid_a == expected.sample_valid_a, 'sample_valid_a mismatch.');
assert(result.sample_valid_b == expected.sample_valid_b, 'sample_valid_b mismatch.');
assert(result.sample_valid_c == expected.sample_valid_c, 'sample_valid_c mismatch.');
assert(result.all_samples_valid == expected.all_samples_valid, 'all_samples_valid mismatch.');

results_dir = fullfile(script_dir, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

report_file = fullfile(results_dir, 'pwm_deadtime_sampling_window_test_report.txt');
fid = fopen(report_file, 'w');
cleanup_file = onCleanup(@() fclose(fid));

fprintf(fid, 'PWM dead-time sampling-window functional test\n');
fprintf(fid, 'usable_low_a_s = %.9g s\n', result.usable_low_a_s);
fprintf(fid, 'usable_low_b_s = %.9g s\n', result.usable_low_b_s);
fprintf(fid, 'usable_low_c_s = %.9g s\n', result.usable_low_c_s);
fprintf(fid, 'sample_valid_a = %d\n', result.sample_valid_a);
fprintf(fid, 'sample_valid_b = %d\n', result.sample_valid_b);
fprintf(fid, 'sample_valid_c = %d\n', result.sample_valid_c);
fprintf(fid, 'all_samples_valid = %d\n', result.all_samples_valid);
fprintf(fid, 'min_usable_low_s = %.9g s\n', result.min_usable_low_s);
fprintf(fid, 'Result: PASS\n');

fprintf('\nPWM dead-time sampling-window functional test passed.\n');
fprintf('usable_low = [%.3f %.3f %.3f] us\n', ...
    result.usable_low_a_s * 1e6, result.usable_low_b_s * 1e6, ...
    result.usable_low_c_s * 1e6);
fprintf('sample_valid = [%d %d %d], all = %d\n', ...
    result.sample_valid_a, result.sample_valid_b, ...
    result.sample_valid_c, result.all_samples_valid);
fprintf('Saved report:\n  %s\n', report_file);

function value = get_timeseries_last(sim_out, name)
ts = sim_out.get(name);
if isa(ts, 'timeseries')
    value = ts.Data(end);
elseif isstruct(ts) && isfield(ts, 'signals') && isfield(ts.signals, 'values')
    values = ts.signals.values;
    value = values(end);
elseif isnumeric(ts) || islogical(ts)
    value = ts(end);
else
    error('Unsupported logged value for "%s": %s', name, class(ts));
end
end

function assert_close(actual, expected, tolerance, label)
if abs(double(actual) - double(expected)) > tolerance
    error('%s mismatch: actual %.9g, expected %.9g, tolerance %.9g', ...
        label, actual, expected, tolerance);
end
end
