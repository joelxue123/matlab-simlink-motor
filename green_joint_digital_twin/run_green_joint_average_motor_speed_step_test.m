%% Speed-loop step test with the V1 average motor digital twin
%
% This is a mainline scenario: SpeedPiStep drives GreenJointCurrentLoopStep,
% DqToAbcDutyStep, Average-Value Inverter, and Surface Mount PMSM through the
% saved green_joint_average_motor_twin_model.slx.

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

run(fullfile(script_dir, 'sync_green_joint_speed_loop_twin_parameters.m'));

% sync_* and design_* scripts are script-style and clear caller workspace.
script_dir = fileparts(mfilename('fullpath'));
previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir)); %#ok<NASGU>
cd(script_dir);

model = 'green_joint_average_motor_twin_model';
model_file = fullfile(script_dir, [model '.slx']);
if ~exist(model_file, 'file') || strcmp(getenv('GJ_DT_REBUILD_BEFORE_TEST'), '1')
    run(fullfile(script_dir, 'build_green_joint_average_motor_twin_model.m'));

    script_dir = fileparts(mfilename('fullpath'));
    previous_dir = pwd;
    cleanup_dir = onCleanup(@() cd(previous_dir)); %#ok<NASGU>
    cd(script_dir);
    run(fullfile(script_dir, 'setup_green_joint_current_loop_twin.m'));
    run(fullfile(script_dir, 'design_green_joint_speed_loop.m'));
else
    fprintf('Using existing green-joint average motor twin model:\n  %s\n', ...
        model_file);
end

scenario.name = 'speed_step_0_to_4radps_joint_average_motor_v1';
scenario.stop_time_s = 0.250;
scenario.step_time_s = 0.005;
scenario.speed_ref_before_rad_s = single(0.0);
scenario.speed_ref_after_rad_s = single(4.0);
scenario.iq_limit_a = single(4.0);
scenario.settling_band_rad_s = 0.05 * ...
    max(1.0, abs(double(scenario.speed_ref_after_rad_s)));

GJDT_UseSpeedLoop = 1;
GJDT_StopTime = scenario.stop_time_s;
GJDT_SpeedRefStepTime_s = scenario.step_time_s;
GJDT_SpeedRefBefore_rad_s = scenario.speed_ref_before_rad_s;
GJDT_SpeedRefAfter_rad_s = scenario.speed_ref_after_rad_s;
GJDT_SpeedIqLimit_A = scenario.iq_limit_a;

model = 'green_joint_average_motor_twin_model';
model_file = fullfile(script_dir, [model '.slx']);

load_system(model_file);
cleanup_model = onCleanup(@() close_model_without_saving(model));
set_param(model, 'StopTime', 'GJDT_StopTime');

sim_result = sim(model, 'ReturnWorkspaceOutputs', 'on');

[t_speed_ref, speed_ref] = read_signal(sim_result, 'gjavg_speed_ref'); %#ok<ASGLU>
[t_joint_speed, joint_speed] = read_signal(sim_result, 'gjavg_joint_speed');
[t_iq_ref, iq_ref] = read_signal(sim_result, 'gjavg_iq_ref');
[t_iq, iq] = read_signal(sim_result, 'gjavg_iq');
[t_vnorm, voltage_mag_norm] = read_signal(sim_result, ...
    'gjavg_voltage_mag_norm');
[t_wm, motor_speed] = read_signal(sim_result, 'gjavg_wm');
[t_joint_speed_ideal, joint_speed_ideal] = read_signal(sim_result, ...
    'gjavg_joint_speed_ideal');

speed_ref_interp = reference_step_at_time(t_joint_speed, scenario);
iq_ref_interp = interp_signal(t_iq_ref, iq_ref, t_joint_speed);
iq_interp = interp_signal(t_iq, iq, t_joint_speed);
vnorm_interp = interp_signal(t_vnorm, voltage_mag_norm, t_joint_speed);
motor_speed_interp = interp_signal(t_wm, motor_speed, t_joint_speed);
joint_speed_ideal_interp = interp_signal(t_joint_speed_ideal, ...
    joint_speed_ideal, t_joint_speed);
speed_est_error_interp = joint_speed - joint_speed_ideal_interp;

post_step = t_joint_speed >= scenario.step_time_s;
final_ref = double(speed_ref_interp(end));
final_speed = double(joint_speed(end));
final_ideal_speed = double(joint_speed_ideal_interp(end));
final_error = final_ref - final_speed;
final_estimator_error = double(speed_est_error_interp(end));
overshoot_rad_s = max(joint_speed(post_step)) - final_ref;
if final_ref ~= 0
    overshoot_pct = max(0, overshoot_rad_s) / abs(final_ref) * 100;
else
    overshoot_pct = 0;
end

rise_time_s = first_crossing_time(t_joint_speed, joint_speed, ...
    scenario.step_time_s, 0.9 * final_ref);
settling_time_s = settling_time(t_joint_speed, joint_speed, final_ref, ...
    scenario.step_time_s, scenario.settling_band_rad_s);

results_dir = fullfile(script_dir, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

csv_file = fullfile(results_dir, [scenario.name '.csv']);
summary_file = fullfile(results_dir, [scenario.name '_summary.csv']);

result_table = table(t_joint_speed, speed_ref_interp, joint_speed, ...
    joint_speed_ideal_interp, speed_est_error_interp, ...
    motor_speed_interp, iq_ref_interp, iq_interp, vnorm_interp, ...
    'VariableNames', {'time_s', 'joint_speed_ref_rad_s', ...
    'joint_speed_est_rad_s', 'joint_speed_ideal_rad_s', ...
    'speed_est_error_rad_s', 'motor_speed_rad_s', 'iq_ref_a', 'iq_a', ...
    'voltage_mag_norm'});
writetable(result_table, csv_file);

summary_table = table( ...
    string(scenario.name), ...
    string(GJDT_MotorType), ...
    double(GJDT_GearRatio), ...
    double(GJDT_SpeedLoopEquivalentInertia_kg_m2), ...
    double(GJDT_SpeedKp), ...
    double(GJDT_SpeedKi), ...
    double(GJDT_SpeedKaw), ...
    double(scenario.iq_limit_a), ...
    final_ref, ...
    final_speed, ...
    final_ideal_speed, ...
    final_error, ...
    final_estimator_error, ...
    rise_time_s, ...
    settling_time_s, ...
    overshoot_pct, ...
    max(abs(speed_est_error_interp)), ...
    max(abs(iq_ref_interp)), ...
    max(abs(iq_interp)), ...
    max(vnorm_interp), ...
    'VariableNames', {'scenario', 'motor_type', 'gear_ratio', ...
    'speed_loop_equiv_inertia_kg_m2', 'Kp_speed_A_per_radps', ...
    'Ki_speed_A_per_rad', 'Kaw_speed_1_per_s', 'iq_limit_a', ...
    'final_ref_rad_s', 'final_joint_speed_rad_s', ...
    'final_joint_speed_ideal_rad_s', 'final_error_rad_s', ...
    'final_speed_est_error_rad_s', 'rise_time_s', 'settling_time_s', ...
    'overshoot_pct', 'speed_est_error_abs_max_rad_s', ...
    'iq_ref_abs_max_a', 'iq_abs_max_a', 'voltage_mag_norm_max'});
writetable(summary_table, summary_file);

fprintf('\nGreen-joint V1 average-motor speed step test result:\n');
fprintf('  scenario               = %s\n', scenario.name);
fprintf('  motor / gear ratio     = %s / %.6g\n', ...
    string(GJDT_MotorType), GJDT_GearRatio);
fprintf('  speed-loop equiv J     = %.9g kg*m^2\n', ...
    GJDT_SpeedLoopEquivalentInertia_kg_m2);
fprintf('  speed ref              = %.6g -> %.6g rad/s joint-side\n', ...
    double(scenario.speed_ref_before_rad_s), ...
    double(scenario.speed_ref_after_rad_s));
fprintf('  Kp/Ki/Kaw              = %.9g / %.9g / %.9g\n', ...
    double(GJDT_SpeedKp), double(GJDT_SpeedKi), ...
    double(GJDT_SpeedKaw));
fprintf('  iq_limit               = %.6g A\n', double(scenario.iq_limit_a));
fprintf('  final joint speed      = %.6g rad/s\n', final_speed);
fprintf('  final ideal speed      = %.6g rad/s\n', final_ideal_speed);
fprintf('  final speed error      = %.6g rad/s\n', final_error);
fprintf('  final estimator error  = %.6g rad/s\n', final_estimator_error);
fprintf('  rise time to 90%%       = %.6g ms\n', rise_time_s * 1e3);
fprintf('  settling time          = %.6g ms\n', settling_time_s * 1e3);
fprintf('  overshoot              = %.6g %%\n', overshoot_pct);
fprintf('  estimator |error| max  = %.6g rad/s\n', ...
    max(abs(speed_est_error_interp)));
fprintf('  |iq_ref| max           = %.6g A\n', max(abs(iq_ref_interp)));
fprintf('  |iq| max               = %.6g A\n', max(abs(iq_interp)));
fprintf('  voltage_mag_norm max   = %.6g\n', max(vnorm_interp));
fprintf('  csv                    = %s\n', csv_file);
fprintf('  summary                = %s\n', summary_file);

if any(~isfinite(result_table{:, 2:end}), 'all')
    error('Speed step test failed: non-finite signal detected.');
end

if max(vnorm_interp) > 1.0005
    error('Speed step test failed: voltage command exceeded circular limit.');
end

function close_model_without_saving(model)
if bdIsLoaded(model)
    set_param(model, 'Dirty', 'off');
    close_system(model, 0);
end
end

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

function values = interp_signal(time, values, query_time)
if numel(time) < 2
    values = repmat(values(end), size(query_time));
    values = values(:);
    return;
end

values = interp1(time, values, query_time, 'previous', 'extrap');
values = values(:);
end

function values = reference_step_at_time(time, scenario)
values = double(scenario.speed_ref_before_rad_s) * ones(size(time));
values(time >= scenario.step_time_s) = ...
    double(scenario.speed_ref_after_rad_s);
values = values(:);
end

function result = first_crossing_time(time, values, start_time, threshold)
idx = find(time >= start_time & values >= threshold, 1);
if isempty(idx)
    result = NaN;
else
    result = time(idx) - start_time;
end
end

function result = settling_time(time, values, final_value, start_time, band)
idx_start = find(time >= start_time, 1);
if isempty(idx_start)
    result = NaN;
    return;
end

error_abs = abs(values - final_value);
idx_last_outside = find(error_abs(idx_start:end) > band, 1, 'last');
if isempty(idx_last_outside)
    result = 0;
else
    absolute_idx = idx_start + idx_last_outside - 1;
    if absolute_idx >= numel(time)
        result = NaN;
    else
        result = time(absolute_idx + 1) - start_time;
    end
end
end
