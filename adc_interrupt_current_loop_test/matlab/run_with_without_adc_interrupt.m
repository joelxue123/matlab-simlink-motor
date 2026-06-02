%% Compare current-loop simulation with and without ADC interrupt timing
% Plant:
%   L di/dt + R i = v
%
% Case A: without ADC interrupt
%   The current loop runs at each PWM boundary and updates voltage immediately.
%
% Case B: with virtual ADC interrupt
%   PWM period midpoint triggers ADC sampling.
%   ADC EOC occurs after conversion delay and runs the current loop.
%   Computed voltage is written to PWM shadow and takes effect next period.

clear; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
root_dir = fileparts(script_dir);
results_dir = fullfile(root_dir, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

cfg = default_config();

without_adc = simulate_current_loop(cfg, "without_adc_interrupt");
with_adc = simulate_current_loop(cfg, "with_adc_interrupt");

metrics_without = compute_metrics(without_adc, cfg);
metrics_with = compute_metrics(with_adc, cfg);

fprintf('Current loop ADC trigger timing test\n');
fprintf('PWM frequency = %.1f kHz, dt = %.1f us\n', 1/cfg.Tpwm/1000, cfg.dt*1e6);
fprintf('ADC sample phase = %.2f PWM period, ADC conversion delay = %.1f us\n\n', ...
    cfg.adc_sample_phase, cfg.adc_conversion_delay*1e6);

print_metrics('without ADC interrupt', metrics_without);
print_metrics('with ADC interrupt', metrics_with);

save(fullfile(results_dir, 'with_without_adc_interrupt.mat'), ...
    'cfg', 'without_adc', 'with_adc', 'metrics_without', 'metrics_with');

write_result_csv(fullfile(results_dir, 'without_adc_interrupt.csv'), without_adc);
write_result_csv(fullfile(results_dir, 'with_adc_interrupt.csv'), with_adc);

plot_results(without_adc, with_adc, cfg, results_dir);

fprintf('\nSaved results to:\n  %s\n', results_dir);

%% Local functions
function cfg = default_config()
    cfg.R = 0.5;                    % Ohm
    cfg.L = 1.0e-3;                 % H
    cfg.V_limit = 24.0;             % V

    cfg.Tpwm = 100e-6;              % s, 10 kHz PWM/current-loop base rate
    cfg.dt = 1e-6;                  % s, plant integration step
    cfg.T_end = 20e-3;              % s

    cfg.ref_step_time = 2e-3;       % s
    cfg.i_ref_initial = 0.0;        % A
    cfg.i_ref_final = 10.0;         % A

    % PI gains from approximate current-loop bandwidth.
    cfg.current_loop_bw_hz = 800.0;
    wc = 2*pi*cfg.current_loop_bw_hz;
    cfg.Kp = cfg.L * wc;
    cfg.Ki = cfg.R * wc;
    cfg.integrator_limit = cfg.V_limit;

    % Virtual ADC timing.
    cfg.adc_sample_phase = 0.5;       % sample at PWM midpoint
    cfg.adc_conversion_delay = 5e-6;  % s
end

function out = simulate_current_loop(cfg, mode)
    n = round(cfg.T_end / cfg.dt) + 1;
    steps_per_pwm = round(cfg.Tpwm / cfg.dt);
    adc_sample_offset = round(cfg.adc_sample_phase * steps_per_pwm);
    adc_delay_steps = round(cfg.adc_conversion_delay / cfg.dt);

    t = (0:n-1)' * cfg.dt;
    i = zeros(n, 1);
    i_ref = zeros(n, 1);
    i_meas = zeros(n, 1);
    v_cmd = zeros(n, 1);
    v_applied = zeros(n, 1);
    adc_eoc = zeros(n, 1);
    pwm_update = zeros(n, 1);

    pi_state.integrator = 0.0;
    v_hold = 0.0;
    v_shadow = 0.0;
    adc_sample_value = 0.0;

    for k = 1:n
        tk = t(k);
        i_ref(k) = current_reference(tk, cfg);

        period_index = mod(k - 1, steps_per_pwm);
        is_pwm_boundary = (period_index == 0);

        if mode == "without_adc_interrupt"
            if is_pwm_boundary
                pwm_update(k) = 1.0;
                i_meas(k) = i(k);
                [v_hold, pi_state] = pi_current_controller(i_ref(k), i_meas(k), pi_state, cfg);
            elseif k > 1
                i_meas(k) = i_meas(k - 1);
            end
        elseif mode == "with_adc_interrupt"
            if is_pwm_boundary
                pwm_update(k) = 1.0;
                v_hold = v_shadow;
            end

            if period_index == adc_sample_offset
                adc_sample_value = i(k);
            end

            if period_index == adc_sample_offset + adc_delay_steps
                adc_eoc(k) = 1.0;
                i_meas(k) = adc_sample_value;
                [v_shadow, pi_state] = pi_current_controller(i_ref(k), i_meas(k), pi_state, cfg);
            elseif k > 1
                i_meas(k) = i_meas(k - 1);
            end
        else
            error('Unknown simulation mode: %s', mode);
        end

        v_cmd(k) = v_hold;
        v_applied(k) = v_hold;

        if k < n
            di = (v_hold - cfg.R*i(k)) / cfg.L;
            i(k + 1) = i(k) + cfg.dt * di;
        end
    end

    out.mode = mode;
    out.t = t;
    out.i = i;
    out.i_ref = i_ref;
    out.i_meas = i_meas;
    out.v_cmd = v_cmd;
    out.v_applied = v_applied;
    out.adc_eoc = adc_eoc;
    out.pwm_update = pwm_update;
    out.error = i_ref - i;
end

function i_ref = current_reference(t, cfg)
    if t < cfg.ref_step_time
        i_ref = cfg.i_ref_initial;
    else
        i_ref = cfg.i_ref_final;
    end
end

function [v, state] = pi_current_controller(i_ref, i_meas, state, cfg)
    e = i_ref - i_meas;
    p_term = cfg.Kp * e;
    state.integrator = state.integrator + cfg.Ki * cfg.Tpwm * e;
    state.integrator = clamp(state.integrator, -cfg.integrator_limit, cfg.integrator_limit);

    v_unsat = p_term + state.integrator;
    v = clamp(v_unsat, -cfg.V_limit, cfg.V_limit);

    % Simple anti-windup: do not keep integrating into saturation.
    if v ~= v_unsat
        state.integrator = v - p_term;
        state.integrator = clamp(state.integrator, -cfg.integrator_limit, cfg.integrator_limit);
    end
end

function y = clamp(x, lower, upper)
    y = min(max(x, lower), upper);
end

function metrics = compute_metrics(out, cfg)
    idx_step = find(out.t >= cfg.ref_step_time, 1, 'first');
    final_ref = cfg.i_ref_final;
    band = 0.02 * abs(final_ref - cfg.i_ref_initial);

    metrics.overshoot_pct = max(0, (max(out.i(idx_step:end)) - final_ref) / final_ref * 100);
    metrics.final_error = out.i_ref(end) - out.i(end);
    metrics.error_rms_after_step = sqrt(mean(out.error(idx_step:end).^2));
    metrics.v_rms_after_step = sqrt(mean(out.v_applied(idx_step:end).^2));
    metrics.v_peak = max(abs(out.v_applied));

    settling_time = NaN;
    for k = idx_step:numel(out.t)
        if all(abs(out.i(k:end) - final_ref) <= band)
            settling_time = out.t(k) - cfg.ref_step_time;
            break;
        end
    end
    metrics.settling_time = settling_time;

    idx_90 = find(out.t >= cfg.ref_step_time & out.i >= 0.9*final_ref, 1, 'first');
    if isempty(idx_90)
        metrics.rise_time_90 = NaN;
    else
        metrics.rise_time_90 = out.t(idx_90) - cfg.ref_step_time;
    end
end

function print_metrics(name, metrics)
    fprintf('%s\n', name);
    fprintf('  rise_time_90         = %.6f s\n', metrics.rise_time_90);
    fprintf('  settling_time_2pct   = %.6f s\n', metrics.settling_time);
    fprintf('  overshoot            = %.3f %%\n', metrics.overshoot_pct);
    fprintf('  final_error          = %.6f A\n', metrics.final_error);
    fprintf('  error_rms_after_step = %.6f A\n', metrics.error_rms_after_step);
    fprintf('  v_rms_after_step     = %.6f V\n', metrics.v_rms_after_step);
    fprintf('  v_peak               = %.6f V\n\n', metrics.v_peak);
end

function write_result_csv(filename, out)
    header = 't,i_ref,i,i_meas,error,v_cmd,v_applied,pwm_update,adc_eoc';
    data = [out.t, out.i_ref, out.i, out.i_meas, out.error, ...
        out.v_cmd, out.v_applied, out.pwm_update, out.adc_eoc];

    fid = fopen(filename, 'w');
    if fid < 0
        error('Cannot open CSV file: %s', filename);
    end
    fprintf(fid, '%s\n', header);
    fclose(fid);
    dlmwrite(filename, data, '-append');
end

function plot_results(without_adc, with_adc, cfg, results_dir)
    fig = figure('Name', 'With vs without ADC interrupt', 'Color', 'w');
    tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    plot(without_adc.t*1000, without_adc.i_ref, 'k--', 'LineWidth', 1.1); hold on;
    plot(without_adc.t*1000, without_adc.i, 'b', 'LineWidth', 1.3);
    plot(with_adc.t*1000, with_adc.i, 'r', 'LineWidth', 1.3);
    grid on;
    ylabel('current / A');
    title('Current response');
    legend('i ref', 'without ADC interrupt', 'with ADC interrupt', 'Location', 'best');

    nexttile;
    plot(without_adc.t*1000, without_adc.error, 'b', 'LineWidth', 1.2); hold on;
    plot(with_adc.t*1000, with_adc.error, 'r', 'LineWidth', 1.2);
    grid on;
    ylabel('error / A');
    title('Current error');

    nexttile;
    plot(without_adc.t*1000, without_adc.v_applied, 'b', 'LineWidth', 1.2); hold on;
    plot(with_adc.t*1000, with_adc.v_applied, 'r', 'LineWidth', 1.2);
    grid on;
    ylabel('voltage / V');
    title('Applied voltage');

    nexttile;
    plot(with_adc.t*1000, with_adc.pwm_update, 'Color', [0.1 0.5 0.1], 'LineWidth', 1.0); hold on;
    plot(with_adc.t*1000, with_adc.adc_eoc, 'm', 'LineWidth', 1.0);
    xlim([cfg.ref_step_time*1000-0.2, cfg.ref_step_time*1000+0.8]);
    ylim([-0.1, 1.2]);
    grid on;
    ylabel('event');
    xlabel('time / ms');
    title('Virtual ADC/PWM events near step');
    legend('PWM update', 'ADC EOC', 'Location', 'best');

    exportgraphics(fig, fullfile(results_dir, 'with_without_adc_interrupt.png'), 'Resolution', 160);
end
