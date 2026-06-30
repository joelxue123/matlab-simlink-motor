%% Sweep PiCorrectionGain for green-joint current-loop anti-windup
%
% This script validates whether gj_mbd_pi_correction_gain = 400 is a sensible
% back-calculation anti-windup value for the current Kp/Ki = 1 / 20000 tuning.
% It mirrors the generated MBD current-loop PI equation on the q axis:
%
%   integrator += Ts * (Ki * error + Kaw * (vq_cmd - vq_pre))
%
% The plant is the same fast R/L q-axis approximation used by the V0 sanity
% tests. V1 is still the waveform authority, but this sweep isolates the
% anti-windup term better than a full motor plant.

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

run(fullfile(script_dir, 'setup_green_joint_current_loop_twin.m'));

cfg.ts_ctrl = GJDT_Ts;
cfg.ts_plant = GJDT_TsPlant;
cfg.stop_time = 0.030;
cfg.release_time = 0.008;
cfg.high_ref_a = 4.0;
cfg.low_ref_a = 1.5;
cfg.settling_band_a = 0.15;
cfg.sustained_time_s = 0.0005;
cfg.exit_saturation_threshold = 0.98;
cfg.enter_saturation_threshold = 0.995;
cfg.vbus = double(GJDT_Vbus_V);
cfg.rs = GJDT_Rs_Ohm;
cfg.lq = GJDT_Lq_H;
cfg.voltage_limit = cfg.vbus * 0.577 * 0.9;
cfg.kp = 1.0;
cfg.ki = 20000.0;

kaw_values = [0 100 200 400 800 1200 2000 5000 10000 20000];

results = cell(numel(kaw_values), 1);
rows = zeros(numel(kaw_values), 14);
for i = 1:numel(kaw_values)
    results{i} = simulate_case(cfg, kaw_values(i));
    rows(i, :) = results{i}.summary;
end

summary = array2table(rows, 'VariableNames', { ...
    'kaw', 'kaw_ts', 'anti_windup_tau_ms', ...
    'pre_release_iq_a', 'integrator_at_release_v', ...
    'exit_saturation_ms', 'settling_ms', ...
    'iq_peak_after_release_a', 'iq_min_after_release_a', ...
    'iq_final_a', 'vq_abs_max_v', 'voltage_norm_max', ...
    'post_release_sat_time_ms', 'score'});

results_dir = fullfile(script_dir, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

csv_file = fullfile(results_dir, 'pi_correction_gain_sweep_kp1_ki20000.csv');
writetable(summary, csv_file);

plot_file = fullfile(results_dir, 'pi_correction_gain_sweep_kp1_ki20000.png');
plot_sweep(cfg, results, plot_file);

fprintf('\nGreen-joint PiCorrectionGain sweep, Kp=%.6g Ki=%.6g\n', ...
    cfg.kp, cfg.ki);
fprintf('  Ts_ctrl / Ts_plant      = %.6g us / %.6g us\n', ...
    cfg.ts_ctrl * 1e6, cfg.ts_plant * 1e6);
fprintf('  voltage limit           = %.6g V\n', cfg.voltage_limit);
fprintf('  saturation scenario     = %.6g A -> %.6g A at %.6g ms\n', ...
    cfg.high_ref_a, cfg.low_ref_a, cfg.release_time * 1e3);
fprintf('  Kaw=400 scale           = Kaw*Ts = %.6g, tau = %.6g ms\n', ...
    400 * cfg.ts_ctrl, 1e3 / 400);
fprintf('\nSweep summary:\n');
disp(summary(:, {'kaw', 'kaw_ts', 'anti_windup_tau_ms', ...
    'integrator_at_release_v', 'exit_saturation_ms', 'settling_ms', ...
    'iq_peak_after_release_a', 'iq_min_after_release_a', ...
    'post_release_sat_time_ms', 'score'}));

fprintf('\nArtifacts:\n');
fprintf('  csv  = %s\n', csv_file);
fprintf('  plot = %s\n', plot_file);

function result = simulate_case(cfg, kaw)
t = (0:cfg.ts_plant:cfg.stop_time)';
ref = cfg.high_ref_a * ones(size(t));
ref(t >= cfg.release_time) = cfg.low_ref_a;

ctrl_steps = round(cfg.ts_ctrl / cfg.ts_plant);
iq = zeros(size(t));
vq = zeros(size(t));
integrator = 0.0;
last_vq = 0.0;
integrator_trace = zeros(size(t));

for k = 2:numel(t)
    if mod(k - 2, ctrl_steps) == 0
        err = ref(k - 1) - iq(k - 1);
        pre_sat = cfg.kp * err + integrator;
        cmd = min(max(pre_sat, -cfg.voltage_limit), cfg.voltage_limit);
        integrator = integrator + ...
            (cfg.ki * err + kaw * (cmd - pre_sat)) * cfg.ts_ctrl;
        last_vq = cmd;
    end

    vq(k) = last_vq;
    iq(k) = iq(k - 1) + cfg.ts_plant * ...
        (vq(k) - cfg.rs * iq(k - 1)) / cfg.lq;
    integrator_trace(k) = integrator;
end

vnorm = abs(vq) / cfg.voltage_limit;
pre_release = t >= (cfg.release_time - 0.001) & t < cfg.release_time;
post_release = t >= cfg.release_time;

exit_time = first_sustained_time(t, vnorm <= cfg.exit_saturation_threshold, ...
    cfg.release_time, cfg.sustained_time_s);
settling_time = first_sustained_time(t, ...
    abs(iq - cfg.low_ref_a) <= cfg.settling_band_a, ...
    cfg.release_time, cfg.sustained_time_s);

post_sat_time = sum(post_release & vnorm > cfg.exit_saturation_threshold) ...
    * cfg.ts_plant;

if isnan(exit_time)
    exit_ms = NaN;
else
    exit_ms = (exit_time - cfg.release_time) * 1e3;
end

if isnan(settling_time)
    settling_ms = NaN;
else
    settling_ms = (settling_time - cfg.release_time) * 1e3;
end

tau_ms = Inf;
if kaw > 0
    tau_ms = 1e3 / kaw;
end

score = nan_to_penalty(exit_ms, 100) ...
    + nan_to_penalty(settling_ms, 100) ...
    + 2.0 * post_sat_time * 1e3 ...
    + 5.0 * max(0, max(iq(post_release)) - 2.3) ...
    + 5.0 * max(0, 1.2 - min(iq(post_release)));

release_idx = find(t >= cfg.release_time, 1, 'first');
result.t = t;
result.ref = ref;
result.iq = iq;
result.vq = vq;
result.vnorm = vnorm;
result.integrator = integrator_trace;
result.kaw = kaw;
result.summary = [ ...
    kaw, kaw * cfg.ts_ctrl, tau_ms, ...
    interp1(t, iq, cfg.release_time, 'previous', 'extrap'), ...
    integrator_trace(release_idx), ...
    exit_ms, settling_ms, ...
    max(iq(post_release)), min(iq(post_release)), iq(end), ...
    max(abs(vq)), max(vnorm), post_sat_time * 1e3, score];

if max(vnorm(pre_release)) < cfg.enter_saturation_threshold
    warning('Kaw %.6g did not enter expected saturation before release.', kaw);
end
end

function value = nan_to_penalty(value, penalty)
if isnan(value)
    value = penalty;
end
end

function first_time = first_sustained_time(time, condition, start_time, hold_time)
first_time = NaN;
start_index = find(time >= start_time, 1, 'first');
if isempty(start_index)
    return;
end

for i = start_index:numel(time)
    if ~condition(i)
        continue;
    end

    hold_indices = time >= time(i) & time <= (time(i) + hold_time);
    if any(hold_indices) && all(condition(hold_indices))
        first_time = time(i);
        return;
    end
end
end

function plot_sweep(cfg, results, plot_file)
figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1200 820]);

selected = [1 4 5 7 10];
selected = selected(selected <= numel(results));

subplot(3, 1, 1);
plot(results{selected(1)}.t * 1e3, results{selected(1)}.ref, ...
    'k-', 'LineWidth', 1.0);
hold on;
for idx = selected
    plot(results{idx}.t * 1e3, results{idx}.iq, 'LineWidth', 1.1);
end
grid on;
xlabel('Time (ms)');
ylabel('Iq (A)');
title('Saturation release current response');
legend_entries = [{'Iq Ref'}, arrayfun(@(i) sprintf('Kaw=%g', ...
    results{i}.kaw), selected, 'UniformOutput', false)];
legend(legend_entries, 'Location', 'best');
xlim([cfg.release_time * 1e3 - 1, cfg.stop_time * 1e3]);

subplot(3, 1, 2);
for idx = selected
    plot(results{idx}.t * 1e3, results{idx}.vq, 'LineWidth', 1.1);
    hold on;
end
yline(cfg.voltage_limit, 'k--');
yline(-cfg.voltage_limit, 'k--');
grid on;
xlabel('Time (ms)');
ylabel('Vq (V)');
title('Voltage command and limit');
legend([arrayfun(@(i) sprintf('Kaw=%g', results{i}.kaw), selected, ...
    'UniformOutput', false), {'Limit'}], 'Location', 'best');
xlim([cfg.release_time * 1e3 - 1, cfg.stop_time * 1e3]);

subplot(3, 1, 3);
for idx = selected
    plot(results{idx}.t * 1e3, results{idx}.integrator, 'LineWidth', 1.1);
    hold on;
end
grid on;
xlabel('Time (ms)');
ylabel('Integrator (V)');
title('Integrator unwind');
legend(arrayfun(@(i) sprintf('Kaw=%g', results{i}.kaw), selected, ...
    'UniformOutput', false), 'Location', 'best');
xlim([cfg.release_time * 1e3 - 1, cfg.stop_time * 1e3]);

exportgraphics(gcf, plot_file, 'Resolution', 160);
close(gcf);
end
