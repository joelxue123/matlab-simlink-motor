%% Analyze current PI output ripple during saturation and release

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

fprintf('\nSaturation ripple analysis:\n');
print_case('With anti-windup', with_aw);
print_case('Without anti-windup', without_aw);

fprintf('\nInterpretation:\n');
fprintf('  Saturated vq_ref ripple is essentially zero in both cases.\n');
fprintf('  Anti-windup mainly changes post-release recovery, not high-frequency output ripple during saturation.\n');

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
meas = double(iq_meas.Data(:));
vq = double(vq_ref.Data(:));

on_time = sat_test.ref.on_time;
release_time = sat_test.ref.release_time;
sat_window = t >= on_time & t < release_time;
sat_settled_window = t >= (on_time + 0.003) & t < release_time;
release_window = t >= release_time;

result.sat_vq_min = min(vq(sat_window));
result.sat_vq_max = max(vq(sat_window));
result.sat_vq_pp = result.sat_vq_max - result.sat_vq_min;
result.sat_vq_std = std(vq(sat_window));

result.settled_vq_min = min(vq(sat_settled_window));
result.settled_vq_max = max(vq(sat_settled_window));
result.settled_vq_pp = result.settled_vq_max - result.settled_vq_min;
result.settled_vq_std = std(vq(sat_settled_window));

result.sat_iq_min = min(meas(sat_window));
result.sat_iq_max = max(meas(sat_window));
result.sat_iq_pp = result.sat_iq_max - result.sat_iq_min;
result.settled_iq_std = std(meas(sat_settled_window));

result.release_vq_min = min(vq(release_window));
result.release_vq_max = max(vq(release_window));
result.release_vq_pp = result.release_vq_max - result.release_vq_min;
result.release_iq_min = min(meas(release_window));
result.release_iq_max = max(meas(release_window));
result.release_iq_pp = result.release_iq_max - result.release_iq_min;

close_system(model, 0);
end

function print_case(label, result)
fprintf('\n%s:\n', label);
fprintf('  saturation vq_ref range       = [%.9g, %.9g] V\n', ...
    result.sat_vq_min, result.sat_vq_max);
fprintf('  saturation vq_ref p-p/std     = %.9g / %.9g V\n', ...
    result.sat_vq_pp, result.sat_vq_std);
fprintf('  settled saturation vq p-p/std = %.9g / %.9g V\n', ...
    result.settled_vq_pp, result.settled_vq_std);
fprintf('  saturation iq range           = [%.9g, %.9g] A\n', ...
    result.sat_iq_min, result.sat_iq_max);
fprintf('  saturation iq p-p             = %.9g A\n', result.sat_iq_pp);
fprintf('  settled saturation iq std     = %.9g A\n', result.settled_iq_std);
fprintf('  release vq_ref range/p-p      = [%.9g, %.9g] / %.9g V\n', ...
    result.release_vq_min, result.release_vq_max, result.release_vq_pp);
fprintf('  release iq range/p-p          = [%.9g, %.9g] / %.9g A\n', ...
    result.release_iq_min, result.release_iq_max, result.release_iq_pp);
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
    warning('Could not restore default current PI model after ripple analysis: %s', ...
        err.message);
end
end
