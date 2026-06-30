%% Square-wave test for green_joint_current_loop_twin_model
%
% Runs the v0 dq average-voltage plant with a fast small-signal iq_ref
% square wave. The saved .slx remains unchanged: this script replaces the
% reference source in memory only.

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

% Test scenario. "1 ms square wave" is treated as a full 1 ms period.
GJDT_StopTime = 0.010;
square_period_s = 0.001;
square_half_period_s = square_period_s / 2;
square_amplitude_a = single(0.3);

time = (0:GJDT_Ts:GJDT_StopTime)';
phase = mod(time, square_period_s);
iq_ref_values = -double(square_amplitude_a) * ones(size(time));
iq_ref_values(phase >= square_half_period_s) = double(square_amplitude_a);
GJDT_IqRefTimeseries = timeseries(single(iq_ref_values), time);

load_system(model_file);
cleanup_model = onCleanup(@() close_changed_model_without_saving(model));
replace_iq_step_with_workspace_source(model);

sim_result = sim(model, 'ReturnWorkspaceOutputs', 'on');

[t_iq_ref, iq_ref] = read_signal(sim_result, 'gjdt_iq_ref');
[t_iq, iq] = read_signal(sim_result, 'gjdt_iq');
[~, vq] = read_signal(sim_result, 'gjdt_vq');
[~, voltage_mag_norm] = read_signal(sim_result, 'gjdt_voltage_mag_norm');

iq_ref_interp = interp1(t_iq_ref, iq_ref, t_iq, 'previous', 'extrap');
iq_error = iq_ref_interp - iq;

ignore_initial_s = square_period_s;
measure_window = t_iq >= ignore_initial_s;
iq_peak_pos = max(iq(measure_window));
iq_peak_neg = min(iq(measure_window));
iq_pp = iq_peak_pos - iq_peak_neg;
iq_ref_pp = max(iq_ref_interp(measure_window)) - min(iq_ref_interp(measure_window));
tracking_gain = iq_pp / iq_ref_pp;
rmse_a = sqrt(mean(iq_error(measure_window).^2));
max_abs_error_a = max(abs(iq_error(measure_window)));
vnorm_max = max(voltage_mag_norm);
vq_max_abs = max(abs(vq));

figure('Visible', 'off', 'Color', 'w');
plot(t_iq_ref * 1e3, iq_ref * 1e3, 'LineWidth', 1.2);
hold on;
plot(t_iq * 1e3, iq * 1e3, 'LineWidth', 1.2);
grid on;
xlabel('Time (ms)');
ylabel('Current (mA)');
title('green-joint current-loop v0: 1 ms square-wave iq test');
legend('Iq Ref', 'Iq', 'Location', 'best');

output_png = fullfile(script_dir, 'current_loop_square_wave_1ms.png');
exportgraphics(gcf, output_png, 'Resolution', 160);
close(gcf);

fprintf('\nGreen-joint current-loop 1 ms square-wave test result:\n');
fprintf('  iq_ref amplitude       = +/-%.6g A\n', square_amplitude_a);
fprintf('  square period          = %.6g ms\n', square_period_s * 1e3);
fprintf('  square half-period     = %.6g ms\n', square_half_period_s * 1e3);
fprintf('  stop time              = %.6g ms\n', GJDT_StopTime * 1e3);
fprintf('  iq positive peak       = %.6g A\n', iq_peak_pos);
fprintf('  iq negative peak       = %.6g A\n', iq_peak_neg);
fprintf('  iq peak-to-peak        = %.6g A\n', iq_pp);
fprintf('  iq/ref p-p gain        = %.6g\n', tracking_gain);
fprintf('  iq RMSE after 1 period = %.6g A\n', rmse_a);
fprintf('  iq max abs error       = %.6g A\n', max_abs_error_a);
fprintf('  |vq| max               = %.6g V\n', vq_max_abs);
fprintf('  voltage_mag_norm max   = %.6g\n', vnorm_max);
fprintf('  plot                   = %s\n', output_png);

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

function close_changed_model_without_saving(model)
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
