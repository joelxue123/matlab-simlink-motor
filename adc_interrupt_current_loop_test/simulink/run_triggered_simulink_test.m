%% Run the triggered Simulink current-loop model and save results

clear; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
root_dir = fileparts(script_dir);
results_dir = fullfile(root_dir, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

run(fullfile(script_dir, 'build_adc_interrupt_current_loop_triggered_model.m'));

model = 'adc_interrupt_current_loop_triggered';
simOut = sim(model);
yout = simOut.yout;

data = struct();
data.i_ref = yout.get('i_ref').Values;
data.i_without_adc_delay = yout.get('i_without_adc_delay').Values;
data.i_with_adc_delay = yout.get('i_with_adc_delay').Values;
data.i_adc_sampled = yout.get('i_adc_sampled').Values;
data.v_without_adc_delay = yout.get('v_without_adc_delay').Values;
data.v_adc_shadow = yout.get('v_adc_shadow').Values;
data.v_with_adc_applied = yout.get('v_with_adc_applied').Values;
data.pi_integrator_without = yout.get('pi_integrator_without').Values;
data.pi_integrator_with = yout.get('pi_integrator_with').Values;

cfg = default_config_for_run();
metrics_without = compute_metrics(data.i_ref, data.i_without_adc_delay, ...
    data.v_without_adc_delay, cfg);
metrics_with = compute_metrics(data.i_ref, data.i_with_adc_delay, ...
    data.v_with_adc_applied, cfg);

fprintf('Triggered Simulink ADC current-loop test\n');
print_metrics('Fixed 50 us timer ISR, direct feedback', metrics_without);
print_metrics('Fixed 50 us HW_INT ISR, sample-hold feedback', metrics_with);

save(fullfile(results_dir, 'triggered_simulink_current_loop.mat'), ...
    'cfg', 'data', 'metrics_without', 'metrics_with');

write_result_csv(fullfile(results_dir, 'triggered_simulink_current_loop.csv'), data);
plot_results(data, cfg, results_dir);

fprintf('\nSaved triggered Simulink results to:\n  %s\n', results_dir);

%% Local functions
function cfg = default_config_for_run()
    cfg.Tint = 50e-6;
    cfg.Tpwm = cfg.Tint;
    cfg.dt = 1e-6;
    cfg.T_end = 20e-3;
    cfg.ref_step_time = 2e-3;
    cfg.i_ref_initial = 0.0;
    cfg.i_ref_final = 10.0;
end

function metrics = compute_metrics(i_ref_ts, i_ts, v_ts, cfg)
    t = i_ts.Time;
    i_ref = i_ref_ts.Data;
    i = i_ts.Data;
    v = align_to_time(v_ts, t);

    idx_step = find(t >= cfg.ref_step_time, 1, 'first');
    final_ref = cfg.i_ref_final;
    band = 0.02 * abs(final_ref - cfg.i_ref_initial);
    err = i_ref - i;

    metrics.overshoot_pct = max(0, (max(i(idx_step:end)) - final_ref) / final_ref * 100);
    metrics.final_error = i_ref(end) - i(end);
    metrics.error_rms_after_step = sqrt(mean(err(idx_step:end).^2));
    v_after_step = v(idx_step:end);
    v_after_step = v_after_step(~isnan(v_after_step));
    metrics.v_rms_after_step = sqrt(mean(v_after_step.^2));
    metrics.v_peak = max(abs(v(~isnan(v))));

    metrics.settling_time = NaN;
    for k = idx_step:numel(t)
        if all(abs(i(k:end) - final_ref) <= band)
            metrics.settling_time = t(k) - cfg.ref_step_time;
            break;
        end
    end

    idx_90 = find(t >= cfg.ref_step_time & i >= 0.9*final_ref, 1, 'first');
    if isempty(idx_90)
        metrics.rise_time_90 = NaN;
    else
        metrics.rise_time_90 = t(idx_90) - cfg.ref_step_time;
    end
end

function print_metrics(name, metrics)
    fprintf('\n%s\n', name);
    fprintf('  rise_time_90         = %.6f s\n', metrics.rise_time_90);
    fprintf('  settling_time_2pct   = %.6f s\n', metrics.settling_time);
    fprintf('  overshoot            = %.3f %%\n', metrics.overshoot_pct);
    fprintf('  final_error          = %.6f A\n', metrics.final_error);
    fprintf('  error_rms_after_step = %.6f A\n', metrics.error_rms_after_step);
    fprintf('  v_rms_after_step     = %.6f V\n', metrics.v_rms_after_step);
    fprintf('  v_peak               = %.6f V\n', metrics.v_peak);
end

function write_result_csv(filename, data)
    t = data.i_ref.Time;
    i_ref = align_to_time(data.i_ref, t);
    i_without_adc_delay = align_to_time(data.i_without_adc_delay, t);
    i_with_adc_delay = align_to_time(data.i_with_adc_delay, t);
    i_adc_sampled = align_to_time(data.i_adc_sampled, t);
    v_without_adc_delay = align_to_time(data.v_without_adc_delay, t);
    v_adc_shadow = align_to_time(data.v_adc_shadow, t);
    v_with_adc_applied = align_to_time(data.v_with_adc_applied, t);
    pi_integrator_without = align_to_time(data.pi_integrator_without, t);
    pi_integrator_with = align_to_time(data.pi_integrator_with, t);

    table_data = [ ...
        t, ...
        i_ref, ...
        i_without_adc_delay, ...
        i_with_adc_delay, ...
        i_adc_sampled, ...
        v_without_adc_delay, ...
        v_adc_shadow, ...
        v_with_adc_applied, ...
        pi_integrator_without, ...
        pi_integrator_with];

    header = ['t,i_ref,i_without_adc_delay,i_with_adc_delay,i_adc_sampled,' ...
        'v_without_adc_delay,v_adc_shadow,v_with_adc_applied,' ...
        'pi_integrator_without,pi_integrator_with'];

    fid = fopen(filename, 'w');
    if fid < 0
        error('Cannot open CSV file: %s', filename);
    end
    fprintf(fid, '%s\n', header);
    fclose(fid);
    dlmwrite(filename, table_data, '-append');
end

function plot_results(data, cfg, results_dir)
    t = data.i_ref.Time;
    v_without_adc_delay = align_to_time(data.v_without_adc_delay, t);
    v_adc_shadow = align_to_time(data.v_adc_shadow, t);
    v_with_adc_applied = align_to_time(data.v_with_adc_applied, t);

    fig = figure('Name', 'Triggered Simulink current loop', 'Color', 'w');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    plot(data.i_ref.Time*1000, data.i_ref.Data, 'k--', 'LineWidth', 1.1); hold on;
    plot(data.i_without_adc_delay.Time*1000, data.i_without_adc_delay.Data, 'b', 'LineWidth', 1.3);
    plot(data.i_with_adc_delay.Time*1000, data.i_with_adc_delay.Data, 'r', 'LineWidth', 1.3);
    grid on;
    ylabel('Current (A)');
    legend('i ref', '50 us timer ISR', '50 us HW_INT ISR', 'Location', 'southeast');
    title('Current response');

    nexttile;
    plot(t*1000, v_without_adc_delay, 'b', 'LineWidth', 1.2); hold on;
    plot(t*1000, v_adc_shadow, 'Color', [0.85 0.25 0.1], 'LineWidth', 1.0);
    plot(t*1000, v_with_adc_applied, 'r', 'LineWidth', 1.2);
    grid on;
    ylabel('Voltage (V)');
    legend('timer ISR cmd', 'HW_INT ISR shadow', 'HW_INT path applied', 'Location', 'northeast');
    title('PI output and PWM shadow load');

    nexttile;
    plot(data.i_adc_sampled.Time*1000, data.i_adc_sampled.Data, 'm', 'LineWidth', 1.0); hold on;
    plot(data.i_with_adc_delay.Time*1000, data.i_with_adc_delay.Data, 'r', 'LineWidth', 1.1);
    xline(cfg.ref_step_time*1000, 'k--', 'Step');
    grid on;
    xlabel('Time (ms)');
    ylabel('Current (A)');
    legend('ADC sampled current', 'Plant current', 'Location', 'southeast');
    title('ADC sample hold');

    exportgraphics(fig, fullfile(results_dir, 'triggered_simulink_current_loop.png'), ...
        'Resolution', 160);
end

function xq = align_to_time(ts, tq)
    t = ts.Time(:);
    x = ts.Data(:);

    valid = ~isnan(x);
    if ~any(valid)
        xq = zeros(numel(tq), 1);
        return;
    end

    t = t(valid);
    x = x(valid);
    [t, unique_idx] = unique(t, 'stable');
    x = x(unique_idx);

    if numel(t) == 1
        xq = x(1) * ones(numel(tq), 1);
    else
        xq = interp1(t, x, tq(:), 'previous', 'extrap');
    end

    xq(tq(:) < t(1)) = 0.0;
end
