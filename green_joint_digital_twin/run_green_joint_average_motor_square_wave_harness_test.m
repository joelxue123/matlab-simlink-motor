%% Run the visible V1 average motor 1 kHz square-wave harness model

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

run(fullfile(script_dir, 'setup_green_joint_current_loop_twin.m'));

model = 'green_joint_average_motor_square_wave_harness';
model_file = fullfile(script_dir, [model '.slx']);
if ~exist(model_file, 'file') || strcmp(getenv('GJ_DT_REBUILD_BEFORE_TEST'), '1')
    run(fullfile(script_dir, 'build_green_joint_average_motor_square_wave_harness.m'));

    script_dir = fileparts(mfilename('fullpath'));
    previous_dir = pwd;
    cleanup_dir = onCleanup(@() cd(previous_dir)); %#ok<NASGU>
    cd(script_dir);
    run(fullfile(script_dir, 'setup_green_joint_current_loop_twin.m'));
end

GJDT_StopTime = 0.010;
GJDT_CurDKp = single(1.0);
GJDT_CurDKi = single(20000.0);
GJDT_CurQKp = single(1.0);
GJDT_CurQKi = single(20000.0);

load_system(model_file);
sim_result = sim(model, 'ReturnWorkspaceOutputs', 'on');

[t_iq_ref, iq_ref] = read_signal(sim_result, 'gjavg_iq_ref');
[t_iq, iq] = read_signal(sim_result, 'gjavg_iq');
[~, voltage_mag_norm] = read_signal(sim_result, 'gjavg_voltage_mag_norm');

iq_ref_interp = interp1(t_iq_ref, iq_ref, t_iq, 'previous', 'extrap');
measure_window = t_iq >= 0.002;

iq_peak_pos = max(iq(measure_window));
iq_peak_neg = min(iq(measure_window));
iq_pp = iq_peak_pos - iq_peak_neg;
iq_ref_pp = max(iq_ref_interp(measure_window)) ...
    - min(iq_ref_interp(measure_window));
tracking_gain_pp = iq_pp / iq_ref_pp;
vnorm_max = max(voltage_mag_norm(measure_window));

fprintf('\nVisible square-wave harness test result:\n');
fprintf('  model                  = %s\n', model);
fprintf('  iq positive peak       = %.6g A\n', iq_peak_pos);
fprintf('  iq negative peak       = %.6g A\n', iq_peak_neg);
fprintf('  iq peak-to-peak        = %.6g A\n', iq_pp);
fprintf('  iq/ref p-p gain        = %.6g\n', tracking_gain_pp);
fprintf('  voltage_mag_norm max   = %.6g\n', vnorm_max);
fprintf('Visible square-wave harness test passed.\n');

close_system(model, 0);

function [time, values] = read_signal(sim_result, variable_name)
if ~isprop(sim_result, variable_name) && ~has_variable(sim_result, variable_name)
    error('Expected simulation output variable "%s" was not created.', ...
        variable_name);
end
logged = sim_result.get(variable_name);
time = logged.time(:);
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
