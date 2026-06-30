%% Smoke test for green_joint_average_motor_twin_model
%
% Verifies the green-joint current PI MBD controller can run with the existing
% average-voltage inverter + Surface Mount PMSM plant pattern.

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

run(fullfile(script_dir, 'setup_green_joint_current_loop_twin.m'));

% Keep this smoke test focused on current-loop wiring. With an unloaded,
% low-inertia micro motor, a long positive Iq step quickly becomes a
% high-speed voltage-limit test instead of a basic current-loop sanity test.
GJDT_StopTime = 0.005;
GJDT_IqAfter_A = single(0.3);
GJDT_UseSpeedLoop = 0;

model = 'green_joint_average_motor_twin_model';
model_file = fullfile(script_dir, [model '.slx']);
if ~exist(model_file, 'file') || strcmp(getenv('GJ_DT_REBUILD_BEFORE_TEST'), '1')
    run(fullfile(script_dir, 'build_green_joint_average_motor_twin_model.m'));
else
    fprintf('Using existing green-joint average motor twin model:\n  %s\n', ...
        model_file);
end

load_system(model_file);
sim_result = sim(model, 'ReturnWorkspaceOutputs', 'on');

iq_ref = read_last(sim_result, 'gjavg_iq_ref');
id = read_last(sim_result, 'gjavg_id');
iq = read_last(sim_result, 'gjavg_iq');
vd = read_last(sim_result, 'gjavg_vd');
vq = read_last(sim_result, 'gjavg_vq');
wm = read_last(sim_result, 'gjavg_wm');
voltage_mag_norm_values = read_values(sim_result, 'gjavg_voltage_mag_norm');

must_be_finite('iq_ref', iq_ref);
must_be_finite('id', id);
must_be_finite('iq', iq);
must_be_finite('vd', vd);
must_be_finite('vq', vq);
must_be_finite('wm', wm);

if max(voltage_mag_norm_values) > 1.0005
    error('Voltage command exceeded normalized circular limit.');
end

fprintf('\nGreen-joint average motor twin smoke test result:\n');
fprintf('  iq_ref final          = %.6g A\n', iq_ref);
fprintf('  id final              = %.6g A\n', id);
fprintf('  iq final              = %.6g A\n', iq);
fprintf('  vd final              = %.6g V\n', vd);
fprintf('  vq final              = %.6g V\n', vq);
fprintf('  wm final              = %.6g rad/s\n', wm);
fprintf('  voltage_mag_norm max  = %.6g\n', max(voltage_mag_norm_values));
fprintf('Green-joint average motor twin smoke test passed.\n');

function must_be_finite(name, value)
if ~isfinite(value)
    error('Expected finite %s, got %g.', name, value);
end
end

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
