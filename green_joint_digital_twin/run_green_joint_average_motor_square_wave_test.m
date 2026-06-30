%% 1 kHz square-wave current-loop test with the V1 average motor plant
%
% Scenario:
%   current_square_1khz_0p3A_average_motor_v1
%
% This is a test-harness script. It keeps the saved .slx unchanged and
% replaces the iq_ref source in memory only.

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

run(fullfile(script_dir, 'setup_green_joint_current_loop_twin.m'));

model = 'green_joint_average_motor_twin_model';
model_file = fullfile(script_dir, [model '.slx']);
if ~exist(model_file, 'file') || strcmp(getenv('GJ_DT_REBUILD_BEFORE_TEST'), '1')
    run(fullfile(script_dir, 'build_green_joint_average_motor_twin_model.m'));

    % build_* scripts are script-style and intentionally clear the caller
    % workspace, so restore the test script context before defining scenario.
    script_dir = fileparts(mfilename('fullpath'));
    previous_dir = pwd;
    cleanup_dir = onCleanup(@() cd(previous_dir)); %#ok<NASGU>
    cd(script_dir);
    run(fullfile(script_dir, 'setup_green_joint_current_loop_twin.m'));
else
    fprintf('Using existing green-joint average motor twin model:\n  %s\n', ...
        model_file);
end

scenario.name = char(sprintf('current_square_1khz_0p3A_average_motor_v1_%s', ...
    string(GJDT_MotorType)));
scenario.stop_time_s = 0.010;
scenario.square_frequency_hz = 1000;
scenario.square_period_s = 1 / scenario.square_frequency_hz;
scenario.square_half_period_s = scenario.square_period_s / 2;
scenario.square_amplitude_a = single(0.3);
scenario.measure_start_s = 0.002;

scenario.tuning_case = getenv('GJDT_CURRENT_TUNING_CASE');
if isempty(scenario.tuning_case)
    scenario.tuning_case = 'variant_default';
end
scenario.name = [scenario.name '_' sanitize_name(scenario.tuning_case)];

apply_current_tuning_case(scenario.tuning_case);
GJDT_StopTime = scenario.stop_time_s;

sync_average_twin_current_loop_parameters(script_dir);

time = (0:GJDT_Ts:GJDT_StopTime)';
phase = mod(time, scenario.square_period_s);
iq_ref_values = -double(scenario.square_amplitude_a) * ones(size(time));
iq_ref_values(phase >= scenario.square_half_period_s) = ...
    double(scenario.square_amplitude_a);
GJDT_IqRefTimeseries = timeseries(single(iq_ref_values), time);

load_system(model_file);
cleanup_model = onCleanup(@() restore_iq_step_source_without_saving(model));
set_param(model, 'StopTime', 'GJDT_StopTime');
replace_iq_step_with_workspace_source(model);

sim_result = sim(model, 'ReturnWorkspaceOutputs', 'on');

[t_iq_ref, iq_ref] = read_signal(sim_result, 'gjavg_iq_ref');
[t_iq, iq] = read_signal(sim_result, 'gjavg_iq');
[t_id, id_raw] = read_signal(sim_result, 'gjavg_id');
[t_vd, vd_raw] = read_signal(sim_result, 'gjavg_vd');
[t_vq, vq_raw] = read_signal(sim_result, 'gjavg_vq');
[t_wm, wm_raw] = read_signal(sim_result, 'gjavg_wm');
[t_voltage_mag_norm, voltage_mag_norm_raw] = read_signal(sim_result, ...
    'gjavg_voltage_mag_norm');

iq_ref_interp = interp1(t_iq_ref, iq_ref, t_iq, 'previous', 'extrap');
id = interp_signal(t_id, id_raw, t_iq);
vd = interp_signal(t_vd, vd_raw, t_iq);
vq = interp_signal(t_vq, vq_raw, t_iq);
wm = interp_signal(t_wm, wm_raw, t_iq);
voltage_mag_norm = interp_signal(t_voltage_mag_norm, voltage_mag_norm_raw, ...
    t_iq);
measure_window = t_iq >= scenario.measure_start_s;
iq_error = iq_ref_interp - iq;

fundamental = fundamental_metrics(t_iq(measure_window), ...
    iq_ref_interp(measure_window), iq(measure_window), ...
    scenario.square_frequency_hz);

iq_peak_pos = max(iq(measure_window));
iq_peak_neg = min(iq(measure_window));
iq_pp = iq_peak_pos - iq_peak_neg;
iq_ref_pp = max(iq_ref_interp(measure_window)) ...
    - min(iq_ref_interp(measure_window));
tracking_gain_pp = iq_pp / iq_ref_pp;
rmse_a = sqrt(mean(iq_error(measure_window).^2));
max_abs_error_a = max(abs(iq_error(measure_window)));
id_abs_max = max(abs(id(measure_window)));
vq_abs_max = max(abs(vq(measure_window)));
vd_abs_max = max(abs(vd(measure_window)));
vnorm_max = max(voltage_mag_norm(measure_window));
wm_abs_max = max(abs(wm(measure_window)));

results_dir = fullfile(script_dir, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

plot_file = fullfile(results_dir, [scenario.name '.png']);
csv_file = fullfile(results_dir, [scenario.name '.csv']);

plot_square_wave_result(t_iq_ref, iq_ref, t_iq, iq, vd, vq, ...
    voltage_mag_norm, wm, scenario, plot_file);

result_table = table(t_iq, iq_ref_interp, id, iq, vd, vq, ...
    voltage_mag_norm, wm, ...
    'VariableNames', {'time_s', 'iq_ref_a', 'id_a', 'iq_a', ...
    'vd_v', 'vq_v', 'voltage_mag_norm', 'wm_rad_per_s'});
writetable(result_table, csv_file);

fprintf('\nGreen-joint V1 average-motor current square-wave test result:\n');
fprintf('  scenario               = %s\n', scenario.name);
fprintf('  iq_ref amplitude       = +/-%.6g A\n', scenario.square_amplitude_a);
fprintf('  square frequency       = %.6g Hz\n', scenario.square_frequency_hz);
fprintf('  square period          = %.6g ms\n', scenario.square_period_s * 1e3);
fprintf('  square half-period     = %.6g ms / %.6g control ticks\n', ...
    scenario.square_half_period_s * 1e3, scenario.square_half_period_s / GJDT_Ts);
fprintf('  controller/plant Ts    = %.6g us / %.6g us\n', ...
    GJDT_Ts * 1e6, GJDT_TsPlant * 1e6);
fprintf('  motor variant          = %s\n', string(GJDT_MotorType));
fprintf('  tuning case            = %s\n', scenario.tuning_case);
fprintf('  Kp/Ki                  = %.6g / %.6g\n', ...
    double(GJDT_CurQKp), double(GJDT_CurQKi));
fprintf('  iq positive peak       = %.6g A\n', iq_peak_pos);
fprintf('  iq negative peak       = %.6g A\n', iq_peak_neg);
fprintf('  iq peak-to-peak        = %.6g A\n', iq_pp);
fprintf('  iq/ref p-p gain        = %.6g\n', tracking_gain_pp);
fprintf('  gain@1kHz fundamental  = %.6g\n', fundamental.gain);
fprintf('  lag@1kHz fundamental   = %.6g deg / %.6g us\n', ...
    fundamental.phase_lag_deg, fundamental.lag_s * 1e6);
fprintf('  iq RMSE after 2 ms     = %.6g A\n', rmse_a);
fprintf('  iq max abs error       = %.6g A\n', max_abs_error_a);
fprintf('  |id| max               = %.6g A\n', id_abs_max);
fprintf('  |vd| max               = %.6g V\n', vd_abs_max);
fprintf('  |vq| max               = %.6g V\n', vq_abs_max);
fprintf('  voltage_mag_norm max   = %.6g\n', vnorm_max);
fprintf('  |wm| max               = %.6g rad/s\n', wm_abs_max);
fprintf('  plot                   = %s\n', plot_file);
fprintf('  csv                    = %s\n', csv_file);

restore_iq_step_source_without_saving(model);
clear cleanup_model;

if vnorm_max > 1.0005
    error('Voltage command exceeded normalized circular limit.');
end

function apply_current_tuning_case(tuning_case)
switch string(tuning_case)
    case "variant_default"
        % setup_green_joint_current_loop_twin.m already loaded the variant
        % default current-loop parameters from green-joint Module/Config.
    case "kp1_ki20000"
        assignin('base', 'GJDT_CurDKp', single(1.0));
        assignin('base', 'GJDT_CurDKi', single(20000.0));
        assignin('base', 'GJDT_CurQKp', single(1.0));
        assignin('base', 'GJDT_CurQKi', single(20000.0));
    otherwise
        if ~apply_delay_pm_tuning_case(tuning_case)
            error('Unsupported GJDT_CURRENT_TUNING_CASE: %s', tuning_case);
        end
end
end

function applied = apply_delay_pm_tuning_case(tuning_case)
applied = false;
tokens = regexp(char(tuning_case), ...
    '^(\d+)_(\d+(?:p\d+)?)hz_pm(\d+(?:p\d+)?)_td(\d+)us$', ...
    'tokens', 'once');
if isempty(tokens)
    return;
end

module_id = tokens{1};
if ~strcmp(module_id, char(evalin('base', 'GJDT_MotorType')))
    error('Tuning case %s does not match active motor variant %s.', ...
        tuning_case, char(evalin('base', 'GJDT_MotorType')));
end

bandwidth_hz = token_to_number(tokens{2});
phase_margin_deg = token_to_number(tokens{3});
delay_s = str2double(tokens{4}) * 1e-6;
module_cfg = evalin('base', 'GJDT_ModuleConfig');
[kp, ki] = design_delay_pm_pi(module_cfg.phase_resistance_ohm, ...
    module_cfg.phase_inductance_h, bandwidth_hz, phase_margin_deg, delay_s);

assignin('base', 'GJDT_CurDKp', single(kp));
assignin('base', 'GJDT_CurDKi', single(ki));
assignin('base', 'GJDT_CurQKp', single(kp));
assignin('base', 'GJDT_CurQKi', single(ki));
applied = true;
end

function value = token_to_number(token)
value = str2double(strrep(char(token), 'p', '.'));
if isnan(value)
    error('Invalid numeric token: %s', char(token));
end
end

function [kp, ki] = design_delay_pm_pi(resistance_ohm, inductance_h, ...
    bandwidth_hz, phase_margin_deg, delay_s)
omega_c = 2 * pi * bandwidth_hz;
plant_phase_lag_rad = atan2(omega_c * inductance_h, resistance_ohm);
delay_phase_lag_rad = omega_c * delay_s;
pi_phase_lag_rad = pi - deg2rad(phase_margin_deg) ...
    - plant_phase_lag_rad - delay_phase_lag_rad;
if pi_phase_lag_rad <= 0 || pi_phase_lag_rad >= (pi / 2)
    error(['Infeasible current-loop design: bandwidth %.6g Hz, ', ...
        'PM %.6g deg, delay %.6g us.'], ...
        bandwidth_hz, phase_margin_deg, delay_s * 1e6);
end

plant_den_mag = sqrt(resistance_ohm^2 + (omega_c * inductance_h)^2);
kp = plant_den_mag * cos(pi_phase_lag_rad);
ki = kp * omega_c * tan(pi_phase_lag_rad);
end

function clean = sanitize_name(value)
clean = regexprep(char(value), '[^a-zA-Z0-9_]+', '_');
end

function sync_average_twin_current_loop_parameters(script_dir)
dictionary_file = fullfile(script_dir, ...
    'green_joint_average_motor_twin_interface.sldd');
if ~exist(dictionary_file, 'file')
    return;
end

dd = Simulink.data.dictionary.open(dictionary_file);
cleanup_dd = onCleanup(@() close(dd));
section = getSection(dd, 'Design Data');

upsert_parameter(section, 'CurDKp', double(evalin('base', 'GJDT_CurDKp')), ...
    'T_GJFloat', 'ExportedGlobal');
upsert_parameter(section, 'CurDKi', double(evalin('base', 'GJDT_CurDKi')), ...
    'T_GJFloat', 'ExportedGlobal');
upsert_parameter(section, 'CurQKp', double(evalin('base', 'GJDT_CurQKp')), ...
    'T_GJFloat', 'ExportedGlobal');
upsert_parameter(section, 'CurQKi', double(evalin('base', 'GJDT_CurQKi')), ...
    'T_GJFloat', 'ExportedGlobal');
upsert_parameter(section, 'VoltageLimitRatio', 0.577, ...
    'T_GJFloat', 'ExportedGlobal');
upsert_parameter(section, 'VoltageModulationRatio', 0.9, ...
    'T_GJFloat', 'ExportedGlobal');
upsert_parameter(section, 'VoltageEpsilon', 0.001, ...
    'T_GJVoltage', 'ExportedGlobal');

saveChanges(dd);
end

function upsert_parameter(section, name, value, data_type, storage_class)
parameter = Simulink.Parameter(value);
parameter.DataType = data_type;
parameter.CoderInfo.StorageClass = storage_class;

entry = find(section, 'Name', name);
if isempty(entry)
    addEntry(section, name, parameter);
else
    setValue(entry(1), parameter);
end
end

function replace_iq_step_with_workspace_source(model)
line_handles = get_param([model '/iq_ref_step'], 'LineHandles');
if line_handles.Outport ~= -1
    delete_line(line_handles.Outport);
end
delete_block([model '/iq_ref_step']);

add_block('simulink/Sources/From Workspace', [model '/iq_ref_step'], ...
    'Position', [45 120 105 150], ...
    'VariableName', 'GJDT_IqRefTimeseries', ...
    'SampleTime', 'GJDT_Ts');
add_line(model, 'iq_ref_step/1', 'iq_ref_to_current/1', 'autorouting', 'on');
end

function restore_iq_step_source_without_saving(model)
if bdIsLoaded(model)
    restore_iq_step_source(model);
    set_param(model, 'Dirty', 'off');
    close_system(model, 0);
end
end

function restore_iq_step_source(model)
block_path = [model '/iq_ref_step'];
if getSimulinkBlockHandle(block_path) ~= -1
    line_handles = get_param(block_path, 'LineHandles');
    if line_handles.Outport ~= -1
        delete_line(line_handles.Outport);
    end
    delete_block(block_path);
end

add_block('simulink/Sources/Step', block_path, ...
    'Position', [45 120 105 150], ...
    'Time', 'GJDT_IqStepTime_s', ...
    'Before', 'GJDT_IqBefore_A', ...
    'After', 'GJDT_IqAfter_A', ...
    'SampleTime', 'GJDT_Ts');
add_line(model, 'iq_ref_step/1', 'iq_ref_to_current/1', 'autorouting', 'on');
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
time = time(:);
values = values(:);
if numel(time) == numel(query_time) && all(abs(time - query_time) < 1e-12)
    return;
end
values = interp1(time, values, query_time, 'linear', 'extrap');
end

function metrics = fundamental_metrics(t, ref, y, freq_hz)
omega = 2 * pi * freq_hz;
ref_phasor = mean(ref(:) .* exp(-1j * omega * t(:)));
y_phasor = mean(y(:) .* exp(-1j * omega * t(:)));
metrics.gain = abs(y_phasor) / max(abs(ref_phasor), eps);
phase = angle(y_phasor / ref_phasor);
metrics.phase_lag_deg = mod(-rad2deg(phase) + 180, 360) - 180;
metrics.lag_s = metrics.phase_lag_deg / 360 / freq_hz;
end

function plot_square_wave_result(t_ref, iq_ref, t, iq, vd, vq, ...
        voltage_mag_norm, wm, scenario, plot_file)
figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1200 820]);

subplot(3, 1, 1);
plot(t_ref * 1e3, iq_ref * 1e3, 'k-', 'LineWidth', 1.0);
hold on;
plot(t * 1e3, iq * 1e3, 'Color', [0.08 0.40 0.75], 'LineWidth', 1.2);
grid on;
xlabel('Time (ms)');
ylabel('Current (mA)');
title('V1 average motor plant: 1 kHz iq square-wave');
legend('Iq Ref', 'Iq', 'Location', 'best');
xlim([0 scenario.stop_time_s * 1e3]);

subplot(3, 1, 2);
plot(t * 1e3, vd, 'Color', [0.10 0.55 0.25], 'LineWidth', 1.1);
hold on;
plot(t * 1e3, vq, 'Color', [0.85 0.20 0.15], 'LineWidth', 1.1);
grid on;
xlabel('Time (ms)');
ylabel('Voltage (V)');
title('D/Q voltage command');
legend('Vd', 'Vq', 'Location', 'best');
xlim([0 scenario.stop_time_s * 1e3]);

subplot(3, 1, 3);
plot(t * 1e3, voltage_mag_norm, 'Color', [0.50 0.25 0.65], ...
    'LineWidth', 1.1);
hold on;
plot(t * 1e3, wm, 'Color', [0.80 0.45 0.05], 'LineWidth', 1.1);
yline(1.0, 'k--');
grid on;
xlabel('Time (ms)');
ylabel('norm / rad/s');
title('Voltage utilization and mechanical speed');
legend('voltage mag norm', 'wm', 'limit', 'Location', 'best');
xlim([0 scenario.stop_time_s * 1e3]);

exportgraphics(gcf, plot_file, 'Resolution', 160);
close(gcf);
end
