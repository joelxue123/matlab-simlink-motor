%% Functional test for pwm_deadtime_compensation_model

clear; clc;

script_dir = fileparts(mfilename('fullpath'));
previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

run(fullfile(script_dir, 'build_pwm_deadtime_compensation_model.m'));

model = 'pwm_deadtime_compensation_model';
sim_out = sim(model, 'ReturnWorkspaceOutputs', 'on');

result.da = get_logged_last(sim_out, 'da');
result.db = get_logged_last(sim_out, 'db');
result.dc = get_logged_last(sim_out, 'dc');
result.comp_a = get_logged_last(sim_out, 'comp_a');
result.comp_b = get_logged_last(sim_out, 'comp_b');
result.comp_c = get_logged_last(sim_out, 'comp_c');
result.active_a = logical(get_logged_last(sim_out, 'active_a'));
result.active_b = logical(get_logged_last(sim_out, 'active_b'));
result.active_c = logical(get_logged_last(sim_out, 'active_c'));

tol = 1e-6;
assert_close(result.da, 0.05, tol, 'da');
assert_close(result.db, 0.94, tol, 'db');
assert_close(result.dc, 0.51, tol, 'dc');
assert_close(result.comp_a, 0.00, tol, 'comp_a');
assert_close(result.comp_b, -0.01, tol, 'comp_b');
assert_close(result.comp_c, 0.01, tol, 'comp_c');
assert(result.active_a == false, 'active_a mismatch.');
assert(result.active_b == true, 'active_b mismatch.');
assert(result.active_c == true, 'active_c mismatch.');

results_dir = fullfile(script_dir, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

report_file = fullfile(results_dir, 'pwm_deadtime_compensation_test_report.txt');
fid = fopen(report_file, 'w');
cleanup_file = onCleanup(@() fclose(fid));

fprintf(fid, 'PWM dead-time compensation functional test\n');
fprintf(fid, 'da/db/dc = [%.9g %.9g %.9g]\n', result.da, result.db, result.dc);
fprintf(fid, 'comp = [%.9g %.9g %.9g]\n', result.comp_a, result.comp_b, result.comp_c);
fprintf(fid, 'active = [%d %d %d]\n', result.active_a, result.active_b, result.active_c);
fprintf(fid, 'Result: PASS\n');

fprintf('\nPWM dead-time compensation functional test passed.\n');
fprintf('duty_out = [%.5f %.5f %.5f]\n', result.da, result.db, result.dc);
fprintf('comp = [%.5f %.5f %.5f], active = [%d %d %d]\n', ...
    result.comp_a, result.comp_b, result.comp_c, result.active_a, result.active_b, result.active_c);
fprintf('Saved report:\n  %s\n', report_file);

function value = get_logged_last(sim_out, name)
data = sim_out.get(name);
if isstruct(data) && isfield(data, 'signals') && isfield(data.signals, 'values')
    values = data.signals.values;
    value = values(end);
elseif isa(data, 'timeseries')
    value = data.Data(end);
else
    error('Unsupported logged value format for "%s": %s', name, class(data));
end
end

function assert_close(actual, expected, tolerance, label)
if abs(double(actual) - double(expected)) > tolerance
    error('%s mismatch: actual %.9g, expected %.9g, tolerance %.9g', ...
        label, actual, expected, tolerance);
end
end
