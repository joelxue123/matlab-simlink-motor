%% Sweep 100us speed-loop and aggressive PLL candidates on V1 digital twin
%
% This script keeps the mainline chain intact:
%   SpeedEstimatorPllStep -> SpeedPiStep -> GreenJointCurrentLoopStep
%   -> DqToAbcDutyStep -> Average-Value Inverter -> Surface Mount PMSM
%
% It updates shared .sldd dictionaries, so run it serially.

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
repo_dir = fileparts(script_dir);
speed_pi_dir = fullfile(repo_dir, 'motor_speed_pi_mbd');
speed_estimator_dir = fullfile(repo_dir, 'motor_speed_estimator_mbd');
results_dir = fullfile(script_dir, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

run(fullfile(script_dir, 'setup_green_joint_current_loop_twin.m'));

if abs(GJDT_TsSpeed - 100e-6) > 1e-12
    error('Expected GJDT_TsSpeed = 100us, got %.12g.', GJDT_TsSpeed);
end

model = 'green_joint_average_motor_twin_model';
model_file = fullfile(script_dir, [model '.slx']);
if ~exist(model_file, 'file') || strcmp(getenv('GJ_DT_REBUILD_BEFORE_TEST'), '1')
    run(fullfile(script_dir, 'build_green_joint_average_motor_twin_model.m'));
    cd(script_dir);
end

scenario.name = 'speed_loop_100us_pll_aggressive_sweep_v1';
scenario.stop_time_s = 0.180;
scenario.step_time_s = 0.005;
scenario.speed_ref_before_rad_s = single(0.0);
scenario.speed_ref_after_rad_s = single(4.0);
scenario.iq_limit_a = single(4.0);
scenario.settling_band_rad_s = 0.05 * ...
    max(1.0, abs(double(scenario.speed_ref_after_rad_s)));

speed_bandwidth_hz_list = [20 40 60 80];
pll_bandwidth_hz_list = [120 240 360 480 600 720];
pll_damping = 1.0;

rows = struct([]);
for speed_bandwidth_hz = speed_bandwidth_hz_list
    speed_tuning = design_speed_tuning_from_workspace(speed_bandwidth_hz);
    sync_speed_pi_dictionary(fullfile(speed_pi_dir, 'speed_pi_interface.sldd'), ...
        speed_tuning);

    for pll_bandwidth_hz = pll_bandwidth_hz_list
        pll_tuning = design_pll_tuning(pll_bandwidth_hz, pll_damping);
        sync_speed_estimator_dictionary( ...
            fullfile(speed_estimator_dir, 'speed_estimator_pll_interface.sldd'), ...
            pll_tuning);

        result = run_case(model_file, scenario, speed_tuning, pll_tuning);
        rows = [rows; result]; %#ok<AGROW>
    end
end

summary_table = struct2table(rows);
summary_file = fullfile(results_dir, ...
    'green_joint_speed_loop_100us_pll_aggressive_sweep.csv');
writetable(summary_table, summary_file);

plot_file = fullfile(results_dir, ...
    'green_joint_speed_loop_100us_pll_aggressive_sweep.png');
plot_summary(summary_table, plot_file);

fprintf('\nGreen-joint V1 100us speed-loop + aggressive PLL sweep\n');
fprintf('  scenario = %s\n', scenario.name);
fprintf('  model    = %s\n', model_file);
fprintf('  summary  = %s\n', summary_file);
fprintf('  plot     = %s\n\n', plot_file);
disp(sortrows(summary_table, ...
    {'speed_bandwidth_hz', 'overshoot_pct', 'settling_time_ms'}));

function tuning = design_speed_tuning_from_workspace(speed_bandwidth_hz)
motor = evalin('base', 'motor');
ts_speed = evalin('base', 'GJDT_TsSpeed');
wc = 2 * pi * speed_bandwidth_hz;
zeta = 1.0;

tuning.speed_bandwidth_hz = speed_bandwidth_hz;
tuning.ts_speed_s = ts_speed;
tuning.kp = single(2 * zeta * wc * ...
    motor.speed_loop_equiv_inertia_kg_m2 / motor.torque_constant);
tuning.ki = single(wc ^ 2 * ...
    motor.speed_loop_equiv_inertia_kg_m2 / motor.torque_constant);
tuning.kaw = single(wc);
tuning.iq_limit_a = single(4.0);
end

function tuning = design_pll_tuning(pll_bandwidth_hz, damping)
ts = evalin('base', 'GJDT_Ts');
module_config = evalin('base', 'GJDT_ModuleConfig');
omega_n = 2 * pi * pll_bandwidth_hz;

tuning.pll_bandwidth_hz = pll_bandwidth_hz;
tuning.damping = damping;
tuning.ts_s = ts;
tuning.kp = single(2 * damping * omega_n);
tuning.ki = single(omega_n ^ 2);
tuning.zero_speed_threshold_rad_s = single( ...
    0.5 * ts * double(tuning.ki) * ...
    (2 * pi / double(module_config.speed_estimator.encoder_counts)));
tuning.speed_minus3db_hz = speed_estimator_minus3db_hz( ...
    pll_bandwidth_hz, damping);
end

function hz = speed_estimator_minus3db_hz(pll_bandwidth_hz, damping)
z = damping;
omega_n = 2 * pi * pll_bandwidth_hz;
b = -2 + 4 * z * z;
x = (-b + sqrt(b * b + 4)) / 2;
hz = omega_n * sqrt(x) / (2 * pi);
end

function sync_speed_pi_dictionary(dictionary_file, tuning)
dd = Simulink.data.dictionary.open(dictionary_file);
cleanup_dd = onCleanup(@() close(dd));
section = getSection(dd, 'Design Data');
upsert_parameter(section, 'Kp_speed', tuning.kp, 'T_SpeedPiGain');
upsert_parameter(section, 'Ki_speed', tuning.ki, 'T_SpeedPiGain');
upsert_parameter(section, 'Kaw_speed', tuning.kaw, 'T_SpeedPiGain');
upsert_parameter(section, 'IqLimitDefault', tuning.iq_limit_a, ...
    'T_SpeedPiCurrent');
saveChanges(dd);
end

function sync_speed_estimator_dictionary(dictionary_file, tuning)
dd = Simulink.data.dictionary.open(dictionary_file);
cleanup_dd = onCleanup(@() close(dd));
section = getSection(dd, 'Design Data');
upsert_parameter(section, 'PllKp', tuning.kp, 'T_SpeedEstimatorGain');
upsert_parameter(section, 'PllKi', tuning.ki, 'T_SpeedEstimatorGain');
upsert_parameter(section, 'SpeedEstimatorSampleTime', tuning.ts_s, ...
    'T_SpeedEstimatorFloat');
upsert_parameter(section, 'ZeroSpeedThresholdRadS', ...
    tuning.zero_speed_threshold_rad_s, 'T_SpeedEstimatorSpeed');
saveChanges(dd);
end

function upsert_parameter(section, name, value, data_type)
parameter = Simulink.Parameter(double(value));
parameter.DataType = data_type;
parameter.CoderInfo.StorageClass = 'Auto';
entry = find(section, 'Name', name);
if isempty(entry)
    addEntry(section, name, parameter);
else
    setValue(entry(1), parameter);
end
end

function result = run_case(model_file, scenario, speed_tuning, pll_tuning)
model = 'green_joint_average_motor_twin_model';
load_system(model_file);
cleanup_model = onCleanup(@() close_model_without_saving(model));

assignin('base', 'GJDT_UseSpeedLoop', 1);
assignin('base', 'GJDT_StopTime', scenario.stop_time_s);
assignin('base', 'GJDT_SpeedRefStepTime_s', scenario.step_time_s);
assignin('base', 'GJDT_SpeedRefBefore_rad_s', scenario.speed_ref_before_rad_s);
assignin('base', 'GJDT_SpeedRefAfter_rad_s', scenario.speed_ref_after_rad_s);
assignin('base', 'GJDT_SpeedIqLimit_A', scenario.iq_limit_a);
set_param(model, 'StopTime', 'GJDT_StopTime');

sim_result = sim(model, 'ReturnWorkspaceOutputs', 'on');

[t_joint_speed, joint_speed] = read_signal(sim_result, 'gjavg_joint_speed');
[t_iq_ref, iq_ref] = read_signal(sim_result, 'gjavg_iq_ref');
[t_iq, iq] = read_signal(sim_result, 'gjavg_iq');
[t_vnorm, voltage_mag_norm] = read_signal(sim_result, ...
    'gjavg_voltage_mag_norm');
[t_wm, motor_speed] = read_signal(sim_result, 'gjavg_wm');
[t_joint_speed_ideal, joint_speed_ideal] = read_signal(sim_result, ...
    'gjavg_joint_speed_ideal');

speed_ref = reference_step_at_time(t_joint_speed, scenario);
iq_ref_interp = interp_signal(t_iq_ref, iq_ref, t_joint_speed);
iq_interp = interp_signal(t_iq, iq, t_joint_speed);
vnorm_interp = interp_signal(t_vnorm, voltage_mag_norm, t_joint_speed);
motor_speed_interp = interp_signal(t_wm, motor_speed, t_joint_speed);
joint_speed_ideal_interp = interp_signal(t_joint_speed_ideal, ...
    joint_speed_ideal, t_joint_speed);
speed_est_error = joint_speed - joint_speed_ideal_interp;

post_step = t_joint_speed >= scenario.step_time_s;
final_ref = double(speed_ref(end));
final_speed = double(joint_speed(end));
final_ideal_speed = double(joint_speed_ideal_interp(end));
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

result = struct( ...
    'speed_sample_time_us', speed_tuning.ts_speed_s * 1e6, ...
    'speed_bandwidth_hz', speed_tuning.speed_bandwidth_hz, ...
    'speed_kp', double(speed_tuning.kp), ...
    'speed_ki', double(speed_tuning.ki), ...
    'speed_kaw', double(speed_tuning.kaw), ...
    'pll_bandwidth_hz', pll_tuning.pll_bandwidth_hz, ...
    'pll_speed_minus3db_hz', pll_tuning.speed_minus3db_hz, ...
    'pll_kp', double(pll_tuning.kp), ...
    'pll_ki', double(pll_tuning.ki), ...
    'pll_zero_speed_threshold_rad_s', ...
        double(pll_tuning.zero_speed_threshold_rad_s), ...
    'final_ref_rad_s', final_ref, ...
    'final_joint_speed_rad_s', final_speed, ...
    'final_joint_speed_ideal_rad_s', final_ideal_speed, ...
    'final_error_rad_s', final_ref - final_speed, ...
    'final_speed_est_error_rad_s', double(speed_est_error(end)), ...
    'rise_time_ms', rise_time_s * 1e3, ...
    'settling_time_ms', settling_time_s * 1e3, ...
    'overshoot_pct', overshoot_pct, ...
    'speed_est_error_abs_max_rad_s', max(abs(speed_est_error)), ...
    'iq_ref_abs_max_a', max(abs(iq_ref_interp)), ...
    'iq_abs_max_a', max(abs(iq_interp)), ...
    'voltage_mag_norm_max', max(vnorm_interp), ...
    'motor_speed_rad_s_max', max(abs(motor_speed_interp)), ...
    'nonfinite_detected', any(~isfinite([joint_speed; iq_ref_interp; ...
        iq_interp; vnorm_interp; motor_speed_interp; speed_est_error])));
end

function close_model_without_saving(model)
if bdIsLoaded(model)
    set_param(model, 'Dirty', 'off');
    close_system(model, 0);
end
end

function [time, values] = read_signal(sim_result, variable_name)
logged = sim_result.get(variable_name);
time = logged.time(:);
values = logged.signals.values;
values = values(:);
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

function plot_summary(summary_table, plot_file)
figure_handle = figure('Visible', 'off');
tiledlayout(2, 1);

nexttile;
hold on;
speed_bandwidths = unique(summary_table.speed_bandwidth_hz);
for i = 1:numel(speed_bandwidths)
    rows = summary_table.speed_bandwidth_hz == speed_bandwidths(i);
    plot(summary_table.pll_bandwidth_hz(rows), ...
        summary_table.overshoot_pct(rows), '-o', ...
        'DisplayName', sprintf('speed %.0f Hz', speed_bandwidths(i)));
end
grid on;
xlabel('PLL design bandwidth (Hz)');
ylabel('Speed overshoot (%)');
legend('Location', 'best');
title('100us speed-loop PLL sweep');

nexttile;
hold on;
for i = 1:numel(speed_bandwidths)
    rows = summary_table.speed_bandwidth_hz == speed_bandwidths(i);
    plot(summary_table.pll_bandwidth_hz(rows), ...
        summary_table.speed_est_error_abs_max_rad_s(rows), '-o', ...
        'DisplayName', sprintf('speed %.0f Hz', speed_bandwidths(i)));
end
grid on;
xlabel('PLL design bandwidth (Hz)');
ylabel('Max estimator error (joint rad/s)');
legend('Location', 'best');

saveas(figure_handle, plot_file);
close(figure_handle);
end
