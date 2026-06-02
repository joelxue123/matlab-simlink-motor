function result = analyze_position_pidreg3_ki0_effect()
% Compare the effect of Ui generated only by Kc when Ki = 0.
% The outer-loop controller is simulated with project parameters and a
% reduced speed-loop plant derived from the existing bandwidth settings.

motor_control_params;

Ts = simcfg.Ts_pos;
total_time = 0.5;
time = (0:Ts:total_time).';
sample_count = numel(time);

pos_cmd = zeros(sample_count, 1);
pos_cmd(time >= control.pos_step_time) = control.pos_ref_rad;

tau_speed = max(1 / control.speed_bandwidth_rad_s, simcfg.Ts_speed);

base_params = struct();
base_params.Kp = control.pid_pos.Kp;
base_params.Ki = 0.0;
base_params.Kc = control.pid_pos.Kc;
base_params.OutMax = control.pid_pos.output_limit;
base_params.OutMin = -control.pid_pos.output_limit;
base_params.TauSpeed = tau_speed;
base_params.Ts = Ts;

cases = [
    struct('name', 'Ki0_KcActive', 'mode', 'pidreg3', 'params', base_params), ...
    struct('name', 'Ki0_UiForcedZero', 'mode', 'force_zero', 'params', base_params), ...
    struct('name', 'Ki0_KcZero', 'mode', 'pidreg3', 'params', setfield(base_params, 'Kc', 0.0)) ...
    ];

result = struct();
for case_index = 1:numel(cases)
    case_def = cases(case_index);
    case_result = local_simulate_case(pos_cmd, time, case_def.params, case_def.mode);
    case_result.metrics = local_metrics(case_result, control.pos_step_time, control.pos_ref_rad);
    result.(case_def.name) = case_result;
end

local_print_summary(result);
local_plot_results(time, pos_cmd, result);
end

function case_result = local_simulate_case(pos_cmd, time, params, mode)
sample_count = numel(time);
theta = zeros(sample_count, 1);
w = zeros(sample_count, 1);
err = zeros(sample_count, 1);
up = zeros(sample_count, 1);
ui = zeros(sample_count, 1);
out_pre_sat = zeros(sample_count, 1);
w_ref = zeros(sample_count, 1);

state_ui = 0;
state_out_pre_sat = 0;
Ts = params.Ts;

for sample_index = 1:(sample_count - 1)
    err(sample_index) = pos_cmd(sample_index) - theta(sample_index);
    up(sample_index) = params.Kp * err(sample_index);

    switch mode
        case 'pidreg3'
            [state_ui, state_out_pre_sat, w_ref(sample_index)] = local_pidreg3_step( ...
                err(sample_index), up(sample_index), state_ui, state_out_pre_sat, params);
        case 'force_zero'
            state_ui = 0;
            state_out_pre_sat = up(sample_index);
            w_ref(sample_index) = min(params.OutMax, max(params.OutMin, state_out_pre_sat));
        otherwise
            error('Unknown mode: %s', mode);
    end

    ui(sample_index) = state_ui;
    out_pre_sat(sample_index) = state_out_pre_sat;

    w_dot = (w_ref(sample_index) - w(sample_index)) / params.TauSpeed;
    w(sample_index + 1) = w(sample_index) + Ts * w_dot;
    theta(sample_index + 1) = theta(sample_index) + Ts * w(sample_index + 1);
end

err(end) = pos_cmd(end) - theta(end);
up(end) = params.Kp * err(end);
ui(end) = state_ui;
out_pre_sat(end) = state_out_pre_sat;
w_ref(end) = w_ref(end - 1);

case_result = struct();
case_result.time = time;
case_result.pos_ref = pos_cmd;
case_result.theta = theta;
case_result.w = w;
case_result.w_ref = w_ref;
case_result.err = err;
case_result.up = up;
case_result.ui = ui;
case_result.out_pre_sat = out_pre_sat;
end

function [ui, out_pre_sat, out] = local_pidreg3_step(err, up, ui, out_pre_sat, params)
if out_pre_sat > params.OutMax
    if err < 0
        ui = ui + params.Ki * err;
    else
        ui = ui + params.Kc * (params.OutMax - out_pre_sat);
    end
elseif out_pre_sat < params.OutMin
    if err > 0
        ui = ui + params.Ki * err;
    else
        ui = ui + params.Kc * (params.OutMin - out_pre_sat);
    end
else
    ui = ui + params.Ki * err;
end

ui = min(params.OutMax, max(params.OutMin, ui));
out_pre_sat = up + ui;
out = min(params.OutMax, max(params.OutMin, out_pre_sat));
end

function metrics = local_metrics(case_result, step_time, final_ref)
time = case_result.time;
theta = case_result.theta;
ui = case_result.ui;
err = final_ref - theta;
step_mask = time >= step_time;
time_after = time(step_mask);
theta_after = theta(step_mask);

band = max(0.02 * max(abs(final_ref), 1e-6), 1e-3);
settling_time = NaN;
for idx = 1:numel(time_after)
    if all(abs(final_ref - theta_after(idx:end)) <= band)
        settling_time = time_after(idx) - step_time;
        break;
    end
end

metrics = struct();
metrics.final_position_error = err(end);
metrics.overshoot_rad = max(theta_after - final_ref);
metrics.max_abs_ui = max(abs(ui));
metrics.final_ui = ui(end);
metrics.max_abs_w_ref = max(abs(case_result.w_ref));
metrics.settling_time_s = settling_time;
metrics.ui_nonzero_samples = nnz(abs(ui) > 1e-10);
end

function local_print_summary(result)
case_names = fieldnames(result);
fprintf('\nKi = 0, Ui-from-Kc effect analysis\n');
for case_index = 1:numel(case_names)
    name = case_names{case_index};
    metrics = result.(name).metrics;
    fprintf('\nCase: %s\n', name);
    fprintf('  final position error = %.6f rad\n', metrics.final_position_error);
    fprintf('  overshoot            = %.6f rad\n', metrics.overshoot_rad);
    fprintf('  max |Ui|             = %.6f\n', metrics.max_abs_ui);
    fprintf('  final Ui             = %.6f\n', metrics.final_ui);
    fprintf('  max |w_ref|          = %.6f rad/s\n', metrics.max_abs_w_ref);
    fprintf('  Ui nonzero samples   = %d\n', metrics.ui_nonzero_samples);
    if isnan(metrics.settling_time_s)
        fprintf('  settling time        = not settled\n');
    else
        fprintf('  settling time        = %.6f s\n', metrics.settling_time_s);
    end
end

delta_overshoot = result.Ki0_KcActive.metrics.overshoot_rad - result.Ki0_UiForcedZero.metrics.overshoot_rad;
delta_settling = result.Ki0_KcActive.metrics.settling_time_s - result.Ki0_UiForcedZero.metrics.settling_time_s;
fprintf('\nImpact of allowing Ui from Kc when Ki = 0\n');
fprintf('  delta overshoot (Kc-active - Ui-forced-zero) = %.6f rad\n', delta_overshoot);
fprintf('  delta settling time                          = %.6f s\n', delta_settling);
end

function local_plot_results(time, pos_cmd, result)
figure('Name', 'Ki0 Ui Effect Analysis', 'Color', 'w');

subplot(3, 1, 1);
plot(time, pos_cmd, 'k--', 'LineWidth', 1.1); hold on;
plot(time, result.Ki0_KcActive.theta, 'LineWidth', 1.2);
plot(time, result.Ki0_UiForcedZero.theta, 'LineWidth', 1.2);
plot(time, result.Ki0_KcZero.theta, 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Position (rad)');
title('Position response');
legend('pos ref', 'Ki0 Kc active', 'Ki0 Ui forced zero', 'Ki0 Kc zero', 'Location', 'best');

subplot(3, 1, 2);
plot(time, result.Ki0_KcActive.w_ref, 'LineWidth', 1.2); hold on;
plot(time, result.Ki0_UiForcedZero.w_ref, 'LineWidth', 1.2);
plot(time, result.Ki0_KcZero.w_ref, 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('w_{ref} (rad/s)');
title('Speed command');
legend('Ki0 Kc active', 'Ki0 Ui forced zero', 'Ki0 Kc zero', 'Location', 'best');

subplot(3, 1, 3);
plot(time, result.Ki0_KcActive.ui, 'LineWidth', 1.2); hold on;
plot(time, result.Ki0_UiForcedZero.ui, 'LineWidth', 1.2);
plot(time, result.Ki0_KcZero.ui, 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Ui');
title('Ui trajectory');
legend('Ki0 Kc active', 'Ki0 Ui forced zero', 'Ki0 Kc zero', 'Location', 'best');
end