%% Smoke test for green_joint_current_loop_twin_model
%
% Verifies that the implemented GreenJointCurrentLoopStep can close a current
% loop against a dq average-voltage plant.

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

run(fullfile(script_dir, 'setup_green_joint_current_loop_twin.m'));

model = 'green_joint_current_loop_twin_model';
model_file = fullfile(script_dir, [model '.slx']);
if ~exist(model_file, 'file') || strcmp(getenv('GJ_DT_REBUILD_BEFORE_TEST'), '1')
    run(fullfile(script_dir, 'build_green_joint_current_loop_twin_model.m'));
else
    fprintf('Using existing green-joint current-loop digital twin model:\n  %s\n', ...
        model_file);
end

load_system(fullfile(script_dir, [model '.slx']));

sim_result = sim(model, 'ReturnWorkspaceOutputs', 'on');

iq_ref = read_last(sim_result, 'gjdt_iq_ref');
id = read_last(sim_result, 'gjdt_id');
iq = read_last(sim_result, 'gjdt_iq');
vd = read_last(sim_result, 'gjdt_vd');
vq = read_last(sim_result, 'gjdt_vq');
voltage_mag_norm = read_last(sim_result, 'gjdt_voltage_mag_norm');

iq_error = iq_ref - iq;

if abs(iq_error) > 0.25
    error('Current-loop twin iq did not track. iq_ref=%g A, iq=%g A.', ...
        iq_ref, iq);
end

if abs(id) > 0.20
    error('Current-loop twin generated too much d-axis current. id=%g A.', id);
end

if max(read_values(sim_result, 'gjdt_voltage_mag_norm')) > 1.0005
    error('Voltage command exceeded normalized circular limit.');
end

fprintf('\nGreen-joint current-loop digital twin smoke test result:\n');
fprintf('  iq_ref final          = %.6g A\n', iq_ref);
fprintf('  iq final              = %.6g A\n', iq);
fprintf('  iq tracking error     = %.6g A\n', iq_error);
fprintf('  id final              = %.6g A\n', id);
fprintf('  vd final              = %.6g V\n', vd);
fprintf('  vq final              = %.6g V\n', vq);
fprintf('  voltage_mag_norm max  = %.6g\n', ...
    max(read_values(sim_result, 'gjdt_voltage_mag_norm')));
fprintf('Green-joint current-loop digital twin smoke test passed.\n');

function value = read_last(sim_result, variable_name)
values = read_values(sim_result, variable_name);
value = values(end);
end

function values = read_values(sim_result, variable_name)
if ~isprop(sim_result, variable_name) && ~has_variable(sim_result, variable_name)
    error('Expected simulation output variable "%s" was not created.', ...
        variable_name);
end
logged = sim_result.get(variable_name);
values = logged.signals.values;
values = values(:);
end

function result = has_variable(sim_result, variable_name)
try
    sim_result.get(variable_name);
    result = true;
catch
    result = false;
end
end
