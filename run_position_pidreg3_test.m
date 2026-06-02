function result = run_position_pidreg3_test(overrides)
% Run a position-loop simulation using the PIDREG3 controller in the
% existing average-inverter FOC project.

if nargin < 1
    overrides = struct();
end

motor_control_params;

control.pos_controller_mode = 'pid_reg3';
simcfg.stop_time = max(simcfg.stop_time, 0.5);

if isfield(overrides, 'pid_pos_Kp')
    control.pid_pos.Kp = overrides.pid_pos_Kp;
end
if isfield(overrides, 'pid_pos_Ki')
    control.pid_pos.Ki = overrides.pid_pos_Ki;
end
if isfield(overrides, 'pid_pos_Kc')
    control.pid_pos.Kc = overrides.pid_pos_Kc;
end
if isfield(overrides, 'pid_pos_output_limit')
    control.pid_pos.output_limit = overrides.pid_pos_output_limit;
end
if isfield(overrides, 'pos_use_planner')
    control.pos_use_planner = logical(overrides.pos_use_planner);
end
if isfield(overrides, 'pos_ref_mode')
    control.pos_ref_mode = overrides.pos_ref_mode;
end
if isfield(overrides, 'pos_sine_amplitude_rad')
    control.pos_sine.amplitude_rad = overrides.pos_sine_amplitude_rad;
end
if isfield(overrides, 'pos_sine_freq_hz')
    control.pos_sine.freq_hz = overrides.pos_sine_freq_hz;
end
if isfield(overrides, 'pos_sine_start_time')
    control.pos_sine.start_time = overrides.pos_sine_start_time;
end
if isfield(overrides, 'pos_sine_offset_rad')
    control.pos_sine.offset_rad = overrides.pos_sine_offset_rad;
end
if isfield(overrides, 'pos_chirp_amplitude_rad')
    control.pos_chirp.amplitude_rad = overrides.pos_chirp_amplitude_rad;
end
if isfield(overrides, 'pos_chirp_f0_hz')
    control.pos_chirp.f0_hz = overrides.pos_chirp_f0_hz;
end
if isfield(overrides, 'pos_chirp_f1_hz')
    control.pos_chirp.f1_hz = overrides.pos_chirp_f1_hz;
end
if isfield(overrides, 'pos_chirp_start_time')
    control.pos_chirp.start_time = overrides.pos_chirp_start_time;
end
if isfield(overrides, 'pos_chirp_duration')
    control.pos_chirp.duration = overrides.pos_chirp_duration;
end
if isfield(overrides, 'pos_chirp_offset_rad')
    control.pos_chirp.offset_rad = overrides.pos_chirp_offset_rad;
end
if isfield(overrides, 'stop_time')
    simcfg.stop_time = overrides.stop_time;
end

plot_results = true;
if isfield(overrides, 'plot_results')
    plot_results = logical(overrides.plot_results);
end

assignin('base', 'motor', motor);
assignin('base', 'inverter', inverter);
assignin('base', 'control', control);
assignin('base', 'simcfg', simcfg);

build_average_inverter_foc_model;
set_param('average_inverter_foc', 'InitFcn', '');
sim_out = sim('average_inverter_foc', 'ReturnWorkspaceOutputs', 'on');

pos_ref_ts = sim_out.get('log_pos_ref');
pos_ts = sim_out.get('log_pos');
wref_ts = sim_out.get('log_wref');
wkf_ts = sim_out.get('log_wkf');
ui_pos_ts = [];
if isprop(sim_out, 'SimulationMetadata') || true
    try
        ui_pos_ts = sim_out.get('log_ui_pos');
    catch
        ui_pos_ts = [];
    end
end

result = local_collect_results(pos_ref_ts, pos_ts, wref_ts, wkf_ts, ui_pos_ts, control);
local_print_summary(result);
if plot_results
    local_plot_results(pos_ref_ts, pos_ts, wref_ts, wkf_ts, ui_pos_ts);
end
end

function result = local_collect_results(pos_ref_ts, pos_ts, wref_ts, wkf_ts, ui_pos_ts, control)
time = pos_ts.Time(:);
pos = pos_ts.Data(:);
pos_ref = interp1(pos_ref_ts.Time(:), pos_ref_ts.Data(:), time, 'previous', 'extrap');
w_ref = interp1(wref_ts.Time(:), wref_ts.Data(:), time, 'previous', 'extrap');
w_kf = interp1(wkf_ts.Time(:), wkf_ts.Data(:), time, 'previous', 'extrap');
ui_pos = [];
if ~isempty(ui_pos_ts)
    ui_pos = interp1(ui_pos_ts.Time(:), ui_pos_ts.Data(:), time, 'previous', 'extrap');
end

step_time = control.pos_step_time;
final_ref = pos_ref(end);
final_pos = pos(end);
err = final_ref - pos;
final_err = final_ref - final_pos;

step_mask = time >= step_time;
pos_after_step = pos(step_mask);
time_after_step = time(step_mask);
overshoot = max(pos_after_step - final_ref);
undershoot = min(pos_after_step - final_ref);
band = max(0.02 * max(abs(final_ref), 1e-6), 1e-3);
settling_time = NaN;
for idx = 1:numel(time_after_step)
    if all(abs(err(step_mask)) <= band)
        settling_time = time_after_step(1) - step_time;
        break;
    end
    tail_err = abs(final_ref - pos_after_step(idx:end));
    if all(tail_err <= band)
        settling_time = time_after_step(idx) - step_time;
        break;
    end
end

result = struct();
result.time = time;
result.pos_ref = pos_ref;
result.pos = pos;
result.w_ref = w_ref;
result.w_kf = w_kf;
result.ui_pos = ui_pos;
result.pos_ref_mode = control.pos_ref_mode;
result.final_position_error = final_err;
result.max_abs_position_error = max(abs(final_ref - pos_after_step));
result.overshoot_rad = max(overshoot, 0);
result.undershoot_rad = min(undershoot, 0);
result.max_abs_speed_ref = max(abs(w_ref));
result.max_abs_speed_meas = max(abs(w_kf));
result.settling_time_s = settling_time;
result.position_tolerance_rad = band;
end

function local_print_summary(result)
fprintf('\nPosition PIDREG3 simulation summary\n');
fprintf('  ref mode              : %s\n', result.pos_ref_mode);
if ~strcmpi(result.pos_ref_mode, 'step')
    fprintf('  final position error   = %.6f rad\n', result.final_position_error);
    fprintf('  max |w_ref|            = %.6f rad/s\n', result.max_abs_speed_ref);
    fprintf('  max |w_kf|             = %.6f rad/s\n', result.max_abs_speed_meas);
    return;
end
fprintf('  final position error   = %.6f rad\n', result.final_position_error);
fprintf('  max abs position error = %.6f rad\n', result.max_abs_position_error);
fprintf('  overshoot              = %.6f rad\n', result.overshoot_rad);
fprintf('  undershoot             = %.6f rad\n', result.undershoot_rad);
fprintf('  max |w_ref|            = %.6f rad/s\n', result.max_abs_speed_ref);
fprintf('  max |w_kf|             = %.6f rad/s\n', result.max_abs_speed_meas);
if isnan(result.settling_time_s)
    fprintf('  settling time          = not settled within simulation window\n');
else
    fprintf('  settling time          = %.6f s (%.6f rad band)\n', ...
        result.settling_time_s, result.position_tolerance_rad);
end
end

function local_plot_results(pos_ref_ts, pos_ts, wref_ts, wkf_ts, ui_pos_ts)
figure('Name', 'Position PIDREG3 Test', 'Color', 'w');

subplot(3, 1, 1);
plot(pos_ref_ts.Time, pos_ref_ts.Data, '--', 'LineWidth', 1.2); hold on;
plot(pos_ts.Time, pos_ts.Data, 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Position (rad)');
title('Position response');
legend('pos ref', 'theta meas', 'Location', 'best');

subplot(3, 1, 2);
plot(wref_ts.Time, wref_ts.Data, '--', 'LineWidth', 1.2); hold on;
plot(wkf_ts.Time, wkf_ts.Data, 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Speed (rad/s)');
title('Speed command and measured speed');
legend('w ref', 'w kf', 'Location', 'best');

subplot(3, 1, 3);
if ~isempty(ui_pos_ts)
    plot(ui_pos_ts.Time, ui_pos_ts.Data, 'LineWidth', 1.2);
    legend('ui pos', 'Location', 'best');
else
    plot(0, 0);
    legend('ui pos unavailable', 'Location', 'best');
end
grid on;
xlabel('Time (s)');
ylabel('Ui');
title('Position-loop Ui');
end