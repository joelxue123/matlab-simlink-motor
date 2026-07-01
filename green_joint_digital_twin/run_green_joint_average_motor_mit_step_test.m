%% MIT step test with the V1 average motor digital twin
%
% Mainline path:
%   GreenJointMitImpedanceStep
%     -> GreenJointCurrentLoopStep
%     -> DqToAbcDutyStep
%     -> Average-Value Inverter
%     -> Surface Mount PMSM
%     -> SpeedEstimatorPllStep feedback

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

run(fullfile(script_dir, 'setup_green_joint_current_loop_twin.m'));

scenario.name = 'mit_step_0p2rad_average_motor_v1';
scenario.stop_time_s = 0.120;
scenario.step_time_s = 0.020;
scenario.pos_before_rad = single(0.0);
scenario.pos_after_rad = single(0.20);
scenario.vel_target_rad_s = single(0.0);
scenario.ff_torque_nm = single(0.0);
scenario.settling_band_rad = 0.005;

GJDT_ControlMode = GJDT_ControlModeMit;
GJDT_UseSpeedLoop = 0;
GJDT_StopTime = scenario.stop_time_s;
GJDT_MitPosStepTime_s = scenario.step_time_s;
GJDT_MitPosBefore_Rad = scenario.pos_before_rad;
GJDT_MitPosAfter_Rad = scenario.pos_after_rad;
GJDT_MitVelTarget_RadS = scenario.vel_target_rad_s;
GJDT_MitFfTorque_Nm = scenario.ff_torque_nm;

model = 'green_joint_average_motor_twin_model';
model_file = fullfile(script_dir, [model '.slx']);
if ~exist(model_file, 'file') || strcmp(getenv('GJ_DT_REBUILD_BEFORE_TEST'), '1')
    run(fullfile(script_dir, 'build_green_joint_average_motor_twin_model.m'));
    script_dir = fileparts(mfilename('fullpath'));
    previous_dir = pwd;
    cleanup_dir = onCleanup(@() cd(previous_dir)); %#ok<NASGU>
    cd(script_dir);
    run(fullfile(script_dir, 'setup_green_joint_current_loop_twin.m'));
    GJDT_ControlMode = GJDT_ControlModeMit;
    GJDT_UseSpeedLoop = 0;
    GJDT_StopTime = scenario.stop_time_s;
    GJDT_MitPosStepTime_s = scenario.step_time_s;
    GJDT_MitPosBefore_Rad = scenario.pos_before_rad;
    GJDT_MitPosAfter_Rad = scenario.pos_after_rad;
    GJDT_MitVelTarget_RadS = scenario.vel_target_rad_s;
    GJDT_MitFfTorque_Nm = scenario.ff_torque_nm;
else
    fprintf('Using existing green-joint average motor twin model:\n  %s\n', ...
        model_file);
end

load_system(model_file);
cleanup_model = onCleanup(@() close_model_without_saving(model));
set_param(model, 'StopTime', 'GJDT_StopTime');

sim_result = sim(model, 'ReturnWorkspaceOutputs', 'on');

[t_pos_ref, pos_ref] = read_signal(sim_result, 'gjavg_mit_pos_ref');
[t_pos, joint_pos] = read_signal(sim_result, 'gjavg_joint_pos');
[t_speed, joint_speed] = read_signal(sim_result, 'gjavg_joint_speed');
[t_iq_ref, iq_ref] = read_signal(sim_result, 'gjavg_iq_ref');
[t_iq, iq] = read_signal(sim_result, 'gjavg_iq');
[t_vnorm, voltage_mag_norm] = read_signal(sim_result, ...
    'gjavg_voltage_mag_norm');

query_time = t_pos(:);
pos_ref_interp = interp_signal(t_pos_ref, pos_ref, query_time);
joint_speed_interp = interp_signal(t_speed, joint_speed, query_time);
iq_ref_interp = interp_signal(t_iq_ref, iq_ref, query_time);
iq_interp = interp_signal(t_iq, iq, query_time);
vnorm_interp = interp_signal(t_vnorm, voltage_mag_norm, query_time);
pos_error = wrap_pi_loop(pos_ref_interp - joint_pos);

post_step = query_time >= scenario.step_time_s;
final_ref = double(pos_ref_interp(end));
final_pos = double(joint_pos(end));
final_error = double(pos_error(end));
overshoot_rad = max(joint_pos(post_step)) - final_ref;
if abs(final_ref) > 1e-9
    overshoot_pct = max(0.0, overshoot_rad) / abs(final_ref) * 100.0;
else
    overshoot_pct = 0.0;
end
settling_time_s = settling_time(query_time, joint_pos, final_ref, ...
    scenario.step_time_s, scenario.settling_band_rad);

results_dir = fullfile(script_dir, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

csv_file = fullfile(results_dir, [scenario.name '.csv']);
summary_file = fullfile(results_dir, [scenario.name '_summary.csv']);

result_table = table(query_time, pos_ref_interp, joint_pos, pos_error, ...
    joint_speed_interp, iq_ref_interp, iq_interp, vnorm_interp, ...
    'VariableNames', {'time_s', 'joint_pos_ref_rad', ...
    'joint_pos_rad', 'joint_pos_error_rad', 'joint_speed_rad_s', ...
    'iq_ref_a', 'iq_a', 'voltage_mag_norm'});
writetable(result_table, csv_file);

summary_table = table( ...
    string(scenario.name), ...
    string(GJDT_MotorType), ...
    double(GJDT_GearRatio), ...
    double(GJDT_MitBandwidth_Hz), ...
    double(GJDT_MitDampingRatio), ...
    double(GJDT_MitKp_NmPerRad), ...
    double(GJDT_MitKd_NmSPerRad), ...
    double(GJDT_MitKp_APerRad), ...
    double(GJDT_MitKd_APerRadS), ...
    double(GJDT_MitIqLimit_A), ...
    final_ref, ...
    final_pos, ...
    final_error, ...
    overshoot_pct, ...
    settling_time_s, ...
    max(abs(iq_ref_interp)), ...
    max(abs(iq_interp)), ...
    max(vnorm_interp), ...
    'VariableNames', {'scenario', 'motor_type', 'gear_ratio', ...
    'mit_bandwidth_hz', 'mit_damping_ratio', ...
    'mit_kp_nm_per_rad', 'mit_kd_nm_s_per_rad', ...
    'mit_kp_a_per_rad', 'mit_kd_a_per_rad_s', 'iq_limit_a', ...
    'final_ref_rad', 'final_joint_pos_rad', 'final_error_rad', ...
    'overshoot_pct', 'settling_time_s', 'iq_ref_abs_max_a', ...
    'iq_abs_max_a', 'voltage_mag_norm_max'});
writetable(summary_table, summary_file);

fprintf('\nGreen-joint V1 average-motor MIT step test result:\n');
fprintf('  scenario              = %s\n', scenario.name);
fprintf('  motor / gear ratio    = %s / %.6g\n', ...
    string(GJDT_MotorType), GJDT_GearRatio);
fprintf('  control mode          = %d (MIT)\n', int32(GJDT_ControlMode));
fprintf('  MIT sample time       = %.6g us\n', GJDT_Ts * 1e6);
fprintf('  MIT bandwidth/zeta    = %.6g Hz / %.6g\n', ...
    GJDT_MitBandwidth_Hz, GJDT_MitDampingRatio);
fprintf('  MIT Kp/Kd physical    = %.9g / %.9g\n', ...
    double(GJDT_MitKp_NmPerRad), double(GJDT_MitKd_NmSPerRad));
fprintf('  MIT Kp/Kd legacy      = %.9g / %.9g\n', ...
    double(GJDT_MitKp_APerRad), double(GJDT_MitKd_APerRadS));
fprintf('  pos ref final         = %.6g rad\n', final_ref);
fprintf('  joint pos final       = %.6g rad\n', final_pos);
fprintf('  final error           = %.6g rad\n', final_error);
fprintf('  overshoot             = %.6g %%\n', overshoot_pct);
fprintf('  settling time         = %.6g ms\n', settling_time_s * 1e3);
fprintf('  |iq_ref| max          = %.6g A\n', max(abs(iq_ref_interp)));
fprintf('  |iq| max              = %.6g A\n', max(abs(iq_interp)));
fprintf('  voltage_mag_norm max  = %.6g\n', max(vnorm_interp));
fprintf('  csv                   = %s\n', csv_file);
fprintf('  summary               = %s\n', summary_file);

if any(~isfinite(result_table{:, 2:end}), 'all')
    error('MIT step test failed: non-finite signal detected.');
end

if max(vnorm_interp) > 1.0005
    error('MIT step test failed: voltage command exceeded circular limit.');
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

values = interp1(time(:), values(:), query_time(:), 'previous', 'extrap');
values = values(:);
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

function wrapped = wrap_pi_loop(value)
wrapped = value;
wrapped(wrapped > pi) = wrapped(wrapped > pi) - 2 * pi;
wrapped(wrapped < -pi) = wrapped(wrapped < -pi) + 2 * pi;
end
