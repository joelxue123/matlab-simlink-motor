%% PI tuning sweep for green-joint 1 ms current square-wave test
%
% Fast numerical twin for the MBD current-loop PI. It keeps the same
% physical plant and PI structure used by the Simulink v0 twin, then sweeps
% bandwidth plus small Kp/Ki scaling factors before touching generated code.

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
cfg.feedback_alpha = 0.95;

bandwidth_hz = [800 1000 1200 1500 1800 2000 2200 2500 2800 3000];
kp_scales = [0.75 0.90 1.00 1.10 1.25];
ki_scales = [0.70 0.85 1.00 1.15 1.30];

baseline = simulate_case(cfg, 800, 1.0, 1.0);

rows = [];
for bw = bandwidth_hz
    for kp_scale = kp_scales
        for ki_scale = ki_scales
            result = simulate_case(cfg, bw, kp_scale, ki_scale);
            rows = [rows; result.summary]; %#ok<AGROW>
        end
    end
end

results = array2table(rows, 'VariableNames', { ...
    'bandwidth_hz', 'kp_scale', 'ki_scale', 'kp', 'ki', ...
    'gain_1khz', 'phase_lag_deg', 'lag_us', 'rmse_a', ...
    'max_abs_error_a', 'iq_peak_pos_a', 'iq_peak_neg_a', ...
    'iq_pp_a', 'vq_abs_max_v', 'voltage_norm_max', 'score'});

% Avoid voltage-saturated candidates and prefer small RMSE/lag with near-unity
% fundamental gain. This is a tuning aid, not an automatic hardware decision.
valid = results.voltage_norm_max < 0.85 & results.gain_1khz < 1.35;
valid_results = results(valid, :);
valid_results = sortrows(valid_results, 'score', 'ascend');
top_results = valid_results(1:min(12, height(valid_results)), :);

hardware_safe = results(results.voltage_norm_max < 0.85 ...
    & results.iq_peak_pos_a <= 0.35 ...
    & results.iq_peak_neg_a >= -0.35 ...
    & results.gain_1khz <= 1.12, :);
hardware_safe = sortrows(hardware_safe, 'score', 'ascend');
top_safe_results = hardware_safe(1:min(12, height(hardware_safe)), :);

balanced = find_candidate(results, 2500, 1.25, 0.70);
safe = find_candidate(results, 2200, 1.25, 0.70);
aggressive = top_safe_results(1, :);

safe_wave = simulate_case(cfg, safe.bandwidth_hz, safe.kp_scale, safe.ki_scale);
balanced_wave = simulate_case(cfg, balanced.bandwidth_hz, balanced.kp_scale, ...
    balanced.ki_scale);
aggressive_wave = simulate_case(cfg, aggressive.bandwidth_hz, ...
    aggressive.kp_scale, aggressive.ki_scale);

fprintf('\nGreen-joint current-loop PI tuning sweep: 1 ms square wave\n');
fprintf('  plant Rs/Lq             = %.6g ohm / %.6g H\n', cfg.rs, cfg.lq);
fprintf('  iq_ref                  = +/-%.6g A, %.6g kHz square\n', ...
    cfg.square_amplitude, 1 / cfg.square_period / 1e3);
fprintf('  controller/plant Ts      = %.6g us / %.6g us\n', ...
    cfg.ts_ctrl * 1e6, cfg.ts_plant * 1e6);
fprintf('  voltage limit           = %.6g V\n', cfg.voltage_limit);

fprintf('\nBaseline 800 Hz physical-pole-cancel gains:\n');
print_case(baseline);

fprintf('\nTop candidates:\n');
disp(top_results(:, {'bandwidth_hz', 'kp_scale', 'ki_scale', 'kp', 'ki', ...
    'gain_1khz', 'phase_lag_deg', 'lag_us', 'rmse_a', ...
    'voltage_norm_max', 'score'}));

fprintf('\nHardware-safe candidates, constrained to about <= +/-0.35 A peak:\n');
disp(top_safe_results(:, {'bandwidth_hz', 'kp_scale', 'ki_scale', 'kp', 'ki', ...
    'gain_1khz', 'phase_lag_deg', 'lag_us', 'rmse_a', ...
    'iq_peak_pos_a', 'iq_peak_neg_a', 'voltage_norm_max', 'score'}));

fprintf('\nRecommended staged candidates:\n');
fprintf('Safe first try:\n');
print_case(safe_wave);
fprintf('Balanced next try:\n');
print_case(balanced_wave);
fprintf('Aggressive simulation limit:\n');
print_case(aggressive_wave);

plot_file = fullfile(script_dir, 'current_loop_pi_tuning_sweep_best.png');
plot_comparison(cfg, baseline, safe_wave, balanced_wave, aggressive_wave, ...
    plot_file);

csv_file = fullfile(script_dir, 'current_loop_pi_tuning_sweep_results.csv');
writetable(results, csv_file);

fprintf('\nArtifacts:\n');
fprintf('  plot = %s\n', plot_file);
fprintf('  csv  = %s\n', csv_file);

function result = simulate_case(cfg, bandwidth_hz, kp_scale, ki_scale)
wc = 2 * pi * bandwidth_hz;
kp = cfg.lq * wc * kp_scale;
ki = cfg.rs * wc * ki_scale;

t = (0:cfg.ts_plant:cfg.stop_time)';
ref = square_ref(t, cfg.square_period, cfg.square_amplitude);

iq = zeros(size(t));
iq_fbk = zeros(size(t));
vq = zeros(size(t));
integrator = 0.0;
last_vq = 0.0;

ctrl_steps = round(cfg.ts_ctrl / cfg.ts_plant);
for k = 2:numel(t)
    if mod(k - 2, ctrl_steps) == 0
        err = ref(k - 1) - iq_fbk(k - 1);
        pre_sat = kp * err + integrator;
        cmd = min(max(pre_sat, -cfg.voltage_limit), cfg.voltage_limit);
        integrator = integrator + ...
            (ki * err + cfg.pi_correction_gain * (cmd - pre_sat)) * cfg.ts_ctrl;
        last_vq = cmd;
    end

    vq(k) = last_vq;
    iq(k) = iq(k - 1) + cfg.ts_plant * (vq(k) - cfg.rs * iq(k - 1)) / cfg.lq;
    iq_fbk(k) = iq_fbk(k - 1) + cfg.feedback_alpha * (iq(k) - iq_fbk(k - 1));
end

measure = t >= cfg.measure_start;
fundamental = fundamental_metrics(t(measure), ref(measure), iq_fbk(measure), ...
    1 / cfg.square_period);
err = ref(measure) - iq_fbk(measure);
iq_peak_pos = max(iq_fbk(measure));
iq_peak_neg = min(iq_fbk(measure));
iq_pp = iq_peak_pos - iq_peak_neg;
vq_abs_max = max(abs(vq));
voltage_norm_max = vq_abs_max / cfg.voltage_limit;
rmse = sqrt(mean(err.^2));
max_abs_error = max(abs(err));

score = rmse ...
    + 0.0015 * abs(fundamental.phase_lag_deg) ...
    + 0.10 * abs(fundamental.gain - 1.0) ...
    + 0.20 * max(0, voltage_norm_max - 0.75);

result.t = t;
result.ref = ref;
result.iq = iq_fbk;
result.vq = vq;
result.summary = [bandwidth_hz kp_scale ki_scale kp ki ...
    fundamental.gain fundamental.phase_lag_deg fundamental.lag_s * 1e6 ...
    rmse max_abs_error iq_peak_pos iq_peak_neg iq_pp vq_abs_max ...
    voltage_norm_max score];
end

function candidate = find_candidate(results, bandwidth_hz, kp_scale, ki_scale)
matches = results(results.bandwidth_hz == bandwidth_hz ...
    & abs(results.kp_scale - kp_scale) < 1e-9 ...
    & abs(results.ki_scale - ki_scale) < 1e-9, :);
if isempty(matches)
    error('Candidate %.6g Hz, Kp %.6g, Ki %.6g was not found.', ...
        bandwidth_hz, kp_scale, ki_scale);
end
candidate = matches(1, :);
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
phase_lag_deg = wrap_to_180(phase_lag_deg);
lag_s = phase_lag_deg / 360 / freq_hz;

metrics.gain = gain;
metrics.phase_lag_deg = phase_lag_deg;
metrics.lag_s = lag_s;
end

function value = wrap_to_180(value)
value = mod(value + 180, 360) - 180;
end

function print_case(result)
s = result.summary;
fprintf('  bandwidth = %.6g Hz, kp_scale = %.3g, ki_scale = %.3g\n', ...
    s(1), s(2), s(3));
fprintf('  Kp = %.6g V/A, Ki = %.6g V/(A*s)\n', s(4), s(5));
fprintf('  gain@1kHz = %.6g, phase lag = %.6g deg (%.6g us)\n', ...
    s(6), s(7), s(8));
fprintf('  RMSE = %.6g A, max error = %.6g A\n', s(9), s(10));
fprintf('  iq peak = %.6g / %.6g A, |vq|max = %.6g V, vnorm max = %.6g\n', ...
    s(11), s(12), s(14), s(15));
end

function plot_comparison(cfg, baseline, safe_wave, balanced_wave, ...
    aggressive_wave, plot_file)
figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1200 760]);

subplot(2, 1, 1);
plot(baseline.t * 1e3, baseline.ref * 1e3, 'k-', 'LineWidth', 1.0);
hold on;
plot(baseline.t * 1e3, baseline.iq * 1e3, 'Color', [0.85 0.20 0.15], ...
    'LineWidth', 1.2);
plot(safe_wave.t * 1e3, safe_wave.iq * 1e3, 'Color', [0.10 0.55 0.25], ...
    'LineWidth', 1.2);
plot(balanced_wave.t * 1e3, balanced_wave.iq * 1e3, 'Color', [0.05 0.45 0.80], ...
    'LineWidth', 1.2);
plot(aggressive_wave.t * 1e3, aggressive_wave.iq * 1e3, 'Color', [0.80 0.35 0.00], ...
    'LineWidth', 1.2);
grid on;
xlabel('Time (ms)');
ylabel('Current (mA)');
title('1 ms square-wave current response: baseline vs tuned');
legend('Iq Ref', '800 Hz baseline', 'Safe', 'Balanced', 'Aggressive', ...
    'Location', 'best');
xlim([2 cfg.stop_time * 1e3]);

subplot(2, 1, 2);
plot(baseline.t * 1e3, baseline.vq, 'Color', [0.85 0.20 0.15], ...
    'LineWidth', 1.2);
hold on;
plot(safe_wave.t * 1e3, safe_wave.vq, 'Color', [0.10 0.55 0.25], ...
    'LineWidth', 1.2);
plot(balanced_wave.t * 1e3, balanced_wave.vq, 'Color', [0.05 0.45 0.80], ...
    'LineWidth', 1.2);
plot(aggressive_wave.t * 1e3, aggressive_wave.vq, 'Color', [0.80 0.35 0.00], ...
    'LineWidth', 1.2);
yline(cfg.voltage_limit, 'k--');
yline(-cfg.voltage_limit, 'k--');
grid on;
xlabel('Time (ms)');
ylabel('Vq command (V)');
title('Voltage command headroom');
legend('800 Hz baseline', 'Safe', 'Balanced', 'Aggressive', 'Limit', ...
    'Location', 'best');
xlim([2 cfg.stop_time * 1e3]);

exportgraphics(gcf, plot_file, 'Resolution', 160);
close(gcf);
end
