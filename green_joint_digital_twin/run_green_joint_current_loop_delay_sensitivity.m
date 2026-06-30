%% Delay sensitivity study for green-joint current-loop square-wave test
%
% Models the timing that matters for the hardware scope:
%   1) ADC/Park/current feedback is available at the start of the ISR.
%   2) Scope currently records iq_ref/iq before the current-test square-wave
%      setpoint is updated and before foc_output() is called.
%   3) The PWM voltage command affects the plant after the ISR.
%
% This script therefore compares both the real controller reference and the
% scope-visible reference.

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

run(fullfile(script_dir, 'setup_green_joint_current_loop_twin.m'));

cfg.ts_ctrl = GJDT_Ts;
cfg.ts_plant = GJDT_TsPlant;
cfg.stop_time = 0.010;
cfg.vbus = double(GJDT_Vbus_V);
cfg.rs = GJDT_Rs_Ohm;
cfg.lq = GJDT_Lq_H;
cfg.voltage_limit = cfg.vbus * 0.577 * 0.9;
cfg.pi_correction_gain = 400.0;
cfg.square_period = 0.001;
cfg.square_amplitude = 0.3;
cfg.measure_start = 0.002;

kp = 1.0;
ki = 20000.0;

cases = [ ...
    struct('name', 'ideal_no_extra_delay', 'alpha', 0.95, 'feedback_delay_ticks', 0, 'scope_ref_lag_ticks', 0); ...
    struct('name', 'scope_ref_lag_1tick', 'alpha', 0.95, 'feedback_delay_ticks', 0, 'scope_ref_lag_ticks', 1); ...
    struct('name', 'feedback_delay_1tick', 'alpha', 0.95, 'feedback_delay_ticks', 1, 'scope_ref_lag_ticks', 0); ...
    struct('name', 'feedback_delay_1tick_scope_lag_1tick', 'alpha', 0.95, 'feedback_delay_ticks', 1, 'scope_ref_lag_ticks', 1); ...
    struct('name', 'stronger_filter_scope_lag_1tick', 'alpha', 0.50, 'feedback_delay_ticks', 0, 'scope_ref_lag_ticks', 1)];

summary_rows = [];
waves = cell(numel(cases), 1);
for i = 1:numel(cases)
    waves{i} = simulate_delay_case(cfg, kp, ki, cases(i));
    summary_rows = [summary_rows; waves{i}.summary]; %#ok<AGROW>
end

summary = array2table(summary_rows, 'VariableNames', { ...
    'case_index', 'alpha', 'feedback_delay_ticks', 'scope_ref_lag_ticks', ...
    'gain_vs_control_ref', 'lag_us_vs_control_ref', ...
    'gain_vs_scope_ref', 'lag_us_vs_scope_ref', ...
    'rmse_vs_scope_ref_a', 'iq_peak_pos_a', 'iq_peak_neg_a', ...
    'vq_abs_max_v', 'voltage_norm_max'});
summary.case_name = string({cases.name})';
summary = movevars(summary, 'case_name', 'After', 'case_index');

fprintf('\nGreen-joint current-loop delay sensitivity, Kp=%.6g Ki=%.6g\n', kp, ki);
disp(summary(:, {'case_name', 'alpha', 'feedback_delay_ticks', ...
    'scope_ref_lag_ticks', 'gain_vs_scope_ref', 'lag_us_vs_scope_ref', ...
    'rmse_vs_scope_ref_a', 'iq_peak_pos_a', 'iq_peak_neg_a', ...
    'voltage_norm_max'}));

plot_file = fullfile(script_dir, 'current_loop_delay_sensitivity_kp1_ki20000.png');
plot_delay_cases(cfg, waves, cases, plot_file);

csv_file = fullfile(script_dir, 'current_loop_delay_sensitivity_kp1_ki20000.csv');
writetable(summary, csv_file);

fprintf('\nArtifacts:\n');
fprintf('  plot = %s\n', plot_file);
fprintf('  csv  = %s\n', csv_file);

function result = simulate_delay_case(cfg, kp, ki, case_cfg)
t = (0:cfg.ts_plant:cfg.stop_time)';
control_ref = square_ref(t, cfg.square_period, cfg.square_amplitude);

ctrl_steps = round(cfg.ts_ctrl / cfg.ts_plant);
feedback_delay_samples = case_cfg.feedback_delay_ticks * ctrl_steps;
scope_ref_delay_samples = case_cfg.scope_ref_lag_ticks * ctrl_steps;

iq = zeros(size(t));
iq_fbk = zeros(size(t));
vq = zeros(size(t));
integrator = 0.0;
last_vq = 0.0;

for k = 2:numel(t)
    if mod(k - 2, ctrl_steps) == 0
        feedback_index = max(1, k - 1 - feedback_delay_samples);
        err = control_ref(k - 1) - iq_fbk(feedback_index);
        pre_sat = kp * err + integrator;
        cmd = min(max(pre_sat, -cfg.voltage_limit), cfg.voltage_limit);
        integrator = integrator + ...
            (ki * err + cfg.pi_correction_gain * (cmd - pre_sat)) * cfg.ts_ctrl;
        last_vq = cmd;
    end

    vq(k) = last_vq;
    iq(k) = iq(k - 1) + cfg.ts_plant * (vq(k) - cfg.rs * iq(k - 1)) / cfg.lq;
    iq_fbk(k) = iq_fbk(k - 1) + case_cfg.alpha * (iq(k) - iq_fbk(k - 1));
end

scope_ref = delay_signal(control_ref, scope_ref_delay_samples);
measure = t >= cfg.measure_start;
metrics_control = fundamental_metrics(t(measure), control_ref(measure), ...
    iq_fbk(measure), 1 / cfg.square_period);
metrics_scope = fundamental_metrics(t(measure), scope_ref(measure), ...
    iq_fbk(measure), 1 / cfg.square_period);
err_scope = scope_ref(measure) - iq_fbk(measure);

result.t = t;
result.control_ref = control_ref;
result.scope_ref = scope_ref;
result.iq = iq_fbk;
result.vq = vq;
result.summary = [case_cfg_to_index(case_cfg), case_cfg.alpha, ...
    case_cfg.feedback_delay_ticks, case_cfg.scope_ref_lag_ticks, ...
    metrics_control.gain, metrics_control.lag_s * 1e6, ...
    metrics_scope.gain, metrics_scope.lag_s * 1e6, ...
    sqrt(mean(err_scope.^2)), max(iq_fbk(measure)), min(iq_fbk(measure)), ...
    max(abs(vq)), max(abs(vq)) / cfg.voltage_limit];
end

function index = case_cfg_to_index(~)
persistent case_index;
if isempty(case_index)
    case_index = 0;
end
case_index = case_index + 1;
index = case_index;
end

function y = delay_signal(x, samples)
y = x;
if samples <= 0
    return;
end
y((samples + 1):end) = x(1:(end - samples));
y(1:samples) = x(1);
end

function ref = square_ref(t, period, amplitude)
phase = mod(t, period);
ref = -amplitude * ones(size(t));
ref(phase >= period / 2) = amplitude;
end

function metrics = fundamental_metrics(t, ref, y, freq_hz)
omega = 2 * pi * freq_hz;
ref_phasor = mean(ref(:) .* exp(-1j * omega * t(:)));
y_phasor = mean(y(:) .* exp(-1j * omega * t(:)));
gain = abs(y_phasor) / max(abs(ref_phasor), eps);
phase = angle(y_phasor / ref_phasor);
phase_lag_deg = -rad2deg(phase);
phase_lag_deg = mod(phase_lag_deg + 180, 360) - 180;

metrics.gain = gain;
metrics.lag_s = phase_lag_deg / 360 / freq_hz;
end

function plot_delay_cases(cfg, waves, cases, plot_file)
figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1200 760]);

subplot(2, 1, 1);
plot(waves{1}.t * 1e3, waves{1}.scope_ref * 1e3, 'k-', 'LineWidth', 1.0);
hold on;
for i = 1:numel(waves)
    plot(waves{i}.t * 1e3, waves{i}.iq * 1e3, 'LineWidth', 1.1);
end
grid on;
xlabel('Time (ms)');
ylabel('Current (mA)');
title('Kp=1 Ki=20000: scope-visible reference and delayed feedback cases');
legend_entries = [{'Scope Iq Ref'}, {cases.name}];
legend(legend_entries, 'Interpreter', 'none', 'Location', 'best');
xlim([2 cfg.stop_time * 1e3]);

subplot(2, 1, 2);
for i = 1:numel(waves)
    plot(waves{i}.t * 1e3, waves{i}.vq, 'LineWidth', 1.1);
    hold on;
end
yline(cfg.voltage_limit, 'k--');
yline(-cfg.voltage_limit, 'k--');
grid on;
xlabel('Time (ms)');
ylabel('Vq command (V)');
title('Voltage command');
legend([{cases.name}, {'Limit'}], 'Interpreter', 'none', 'Location', 'best');
xlim([2 cfg.stop_time * 1e3]);

exportgraphics(gcf, plot_file, 'Resolution', 160);
close(gcf);
end
