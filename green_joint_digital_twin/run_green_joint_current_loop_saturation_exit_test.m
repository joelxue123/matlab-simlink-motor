%% Saturation exit test for green_joint_current_loop_twin_model
%
% Drives the v0 dq plant into voltage saturation with an unreachable iq
% command, then drops the command back to a reachable current and measures
% how quickly the controller exits saturation and settles.

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

% Test scenario:
% 4 A needs about 11.6 V steady-state on a 2.9 ohm phase resistance, so a
% 12 V bus with 0.577*0.9 voltage headroom cannot hold it without saturation.
GJDT_StopTime = 0.025;
GJDT_IqStepTime_s = 0.008;
GJDT_IqBefore_A = single(4.0);
GJDT_IqAfter_A = single(1.5);

release_time_s = GJDT_IqStepTime_s;
settling_band_a = 0.15;
sustained_time_s = 0.0005;
exit_saturation_threshold = 0.98;
enter_saturation_threshold = 0.995;

load_system(model_file);
sim_result = sim(model, 'ReturnWorkspaceOutputs', 'on');

[t_iq_ref, iq_ref] = read_signal(sim_result, 'gjdt_iq_ref');
[t_iq, iq] = read_signal(sim_result, 'gjdt_iq');
[t_vq, vq] = read_signal(sim_result, 'gjdt_vq');
[t_vnorm, voltage_mag_norm] = read_signal(sim_result, 'gjdt_voltage_mag_norm');

pre_release = t_vnorm >= (release_time_s - 0.001) & t_vnorm < release_time_s;
if ~any(pre_release)
    error('No pre-release samples were available for saturation check.');
end

pre_release_vnorm_max = max(voltage_mag_norm(pre_release));
pre_release_iq = interp1(t_iq, iq, release_time_s, 'previous', 'extrap');
if pre_release_vnorm_max < enter_saturation_threshold
    error(['Test did not enter voltage saturation before release. ' ...
        'max voltage_mag_norm=%g.'], pre_release_vnorm_max);
end

post_release = t_iq >= release_time_s;
iq_peak_after_release = max(iq(post_release));
iq_min_after_release = min(iq(post_release));
iq_final = iq(end);
vq_final = vq(end);

exit_saturation_time = first_sustained_time(t_vnorm, ...
    voltage_mag_norm <= exit_saturation_threshold, release_time_s, ...
    sustained_time_s);
settling_time = first_sustained_time(t_iq, ...
    abs(iq - double(GJDT_IqAfter_A)) <= settling_band_a, ...
    release_time_s, sustained_time_s);

if isnan(exit_saturation_time)
    error('Controller did not exit voltage saturation within the simulation.');
end

if isnan(settling_time)
    error('Current did not settle within +/-%.3g A by StopTime %.3g s.', ...
        settling_band_a, GJDT_StopTime);
end

fprintf('\nGreen-joint current-loop saturation exit test result:\n');
fprintf('  high iq_ref before release = %.6g A\n', GJDT_IqBefore_A);
fprintf('  low iq_ref after release   = %.6g A\n', GJDT_IqAfter_A);
fprintf('  release time               = %.6g ms\n', release_time_s * 1e3);
fprintf('  pre-release iq             = %.6g A\n', pre_release_iq);
fprintf('  pre-release vnorm max      = %.6g\n', pre_release_vnorm_max);
fprintf('  saturation exit threshold  = %.6g\n', exit_saturation_threshold);
fprintf('  saturation exit time       = %.6g ms after release\n', ...
    (exit_saturation_time - release_time_s) * 1e3);
fprintf('  settling band              = +/-%.6g A\n', settling_band_a);
fprintf('  iq settling time           = %.6g ms after release\n', ...
    (settling_time - release_time_s) * 1e3);
fprintf('  iq peak after release      = %.6g A\n', iq_peak_after_release);
fprintf('  iq min after release       = %.6g A\n', iq_min_after_release);
fprintf('  iq final                   = %.6g A\n', iq_final);
fprintf('  vq final                   = %.6g V\n', vq_final);
fprintf('Green-joint current-loop saturation exit test passed.\n');

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

function first_time = first_sustained_time(time, condition, start_time, hold_time)
first_time = NaN;
start_index = find(time >= start_time, 1, 'first');
if isempty(start_index)
    return;
end

for i = start_index:numel(time)
    if ~condition(i)
        continue;
    end

    hold_indices = time >= time(i) & time <= (time(i) + hold_time);
    if any(hold_indices) && all(condition(hold_indices))
        first_time = time(i);
        return;
    end
end
end
