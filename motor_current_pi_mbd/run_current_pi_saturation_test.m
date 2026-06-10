%% Compare current PI saturation behavior with and without anti-windup
%
% This test runs the same saturation/release scenario twice:
%   1. Kaw_iq = default value
%   2. Kaw_iq = 0
%
% The reusable PI model is not changed. The script temporarily edits the
% dictionary parameter and restores the default model afterward.

clear; clc;

script_dir = fileparts(mfilename('fullpath'));
run(fullfile(script_dir, 'build_current_pi_saturation_test_model.m'));

cfg = evalin('base', 'current_pi_saturation_test_config');
sat_test = evalin('base', 'current_pi_sat_test');
model = cfg.modelName;
model_file = fullfile(script_dir, [model '.slx']);

default_kaw = get_dictionary_parameter(script_dir, 'Kaw_iq');

cleanup = onCleanup(@() restore_default_model(script_dir));

set_dictionary_parameter(script_dir, 'Kaw_iq', default_kaw);
with_aw = run_case(model_file, model, sat_test);

set_dictionary_parameter(script_dir, 'Kaw_iq', single(0));
without_aw = run_case(model_file, model, sat_test);

print_case('With anti-windup', with_aw);
print_case('Without anti-windup', without_aw);

if with_aw.release_recovery_time >= without_aw.release_recovery_time
    error(['Anti-windup saturation test failed: expected the default Kaw_iq ' ...
        'case to recover faster after release.']);
end

if with_aw.release_vq_area >= without_aw.release_vq_area
    error(['Anti-windup saturation test failed: expected less post-release ' ...
        'saturated voltage area with anti-windup.']);
end

fprintf('\nCurrent PI saturation anti-windup test passed.\n');

function result = run_case(model_file, model, sat_test)
load_system(model_file);
set_param(model, 'SimulationCommand', 'update');

sim_out = sim(model, ...
    'ReturnWorkspaceOutputs', 'on', ...
    'SaveOutput', 'off');

iq_ref = sim_out.get('log_iq_ref_sat');
iq_meas = sim_out.get('log_iq_meas_sat');
vq_ref = sim_out.get('log_vq_ref_sat');

t = iq_ref.Time;
ref = double(iq_ref.Data(:));
meas = double(iq_meas.Data(:));
vq = double(vq_ref.Data(:));

v_limit = double(sat_test.vdc) * double(sat_test.v_limit_ratio);
release_time = sat_test.ref.release_time;
post_release = t >= release_time;
release_index = find(post_release, 1, 'first');

result.final_iq = meas(end);
result.max_iq = max(meas);
result.min_iq_after_release = min(meas(post_release));
result.max_vq = max(vq);
result.min_vq = min(vq);
result.release_vq_area = trapz(t(post_release), abs(vq(post_release)));
result.release_recovery_time = calculate_recovery_time(t, meas, release_index);
result.saturation_count = nnz(abs(vq) > 0.99 * v_limit);
result.release_saturation_count = nnz(abs(vq(post_release)) > 0.99 * v_limit);
result.ref_final = ref(end);

close_system(model, 0);
end

function recovery_time = calculate_recovery_time(t, meas, release_index)
threshold = 0.5;
idx = find(abs(meas(release_index:end)) <= threshold, 1, 'first');

if isempty(idx)
    recovery_time = inf;
else
    recovery_time = t(release_index + idx - 1) - t(release_index);
end
end

function print_case(label, result)
fprintf('\n%s:\n', label);
fprintf('  final iq                 = %.6g A\n', result.final_iq);
fprintf('  max iq                   = %.6g A\n', result.max_iq);
fprintf('  min iq after release     = %.6g A\n', result.min_iq_after_release);
fprintf('  vq range                 = [%.6g, %.6g] V\n', ...
    result.min_vq, result.max_vq);
fprintf('  saturation samples       = %d\n', result.saturation_count);
fprintf('  release saturation count = %d\n', result.release_saturation_count);
fprintf('  release recovery time    = %.6g s\n', result.release_recovery_time);
fprintf('  release |vq| area        = %.6g V*s\n', result.release_vq_area);
end

function value = get_dictionary_parameter(script_dir, name)
dd = Simulink.data.dictionary.open(fullfile(script_dir, 'current_pi_interface.sldd'));
cleanup = onCleanup(@() close(dd));
section = getSection(dd, 'Design Data');
entry = getEntry(section, name);
parameter = getValue(entry);
value = single(parameter.Value);
end

function set_dictionary_parameter(script_dir, name, value)
dd = Simulink.data.dictionary.open(fullfile(script_dir, 'current_pi_interface.sldd'));
cleanup = onCleanup(@() close(dd));
section = getSection(dd, 'Design Data');
entry = getEntry(section, name);
parameter = getValue(entry);
parameter.Value = double(value);
setValue(entry, parameter);
saveChanges(dd);
end

function restore_default_model(script_dir)
try
    run(fullfile(script_dir, 'build_current_pi_model.m'));
catch err
    warning('Could not restore default current PI model after saturation test: %s', ...
        err.message);
end
end
