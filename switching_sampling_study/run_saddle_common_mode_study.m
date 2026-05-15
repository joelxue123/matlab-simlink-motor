function result = run_saddle_common_mode_study(varargin)
%RUN_SADDLE_COMMON_MODE_STUDY Visualize saddle-wave modulation and common-mode voltage.
% This script sweeps one electrical cycle, reallocates the zero-vector time
% between V0 and V7, and plots the resulting three-phase saddle wave,
% zero-sequence offset, and equivalent average common-mode voltage.

switching_sampling_study_config;

cfg.modulation_ratio = ss_cfg.modulation_ratio;
cfg.num_points = 721;
cfg.split_cases = ss_cfg.v0v7_splits;
cfg.case_names = ss_cfg.case_names;

cfg = local_parse_inputs(cfg, varargin{:});

theta = linspace(0, 2 * pi, cfg.num_points);
theta_deg = theta * 180 / pi;
v_mag = cfg.modulation_ratio * (ss_cfg.Vdc / sqrt(3));

result.cfg = cfg;
result.theta_deg = theta_deg;
result.case_data = repmat(struct( ...
    'name', '', ...
    'split', 0.0, ...
    'duty', zeros(3, cfg.num_points), ...
    'mod_wave', zeros(3, cfg.num_points), ...
    'raw_mod_wave', zeros(3, cfg.num_points), ...
    'zero_sequence_norm', zeros(1, cfg.num_points), ...
    'v_cm_avg', zeros(1, cfg.num_points), ...
    'v_cm_mean', 0.0, ...
    'v_cm_rms', 0.0, ...
    'v_cm_min', 0.0, ...
    'v_cm_max', 0.0), numel(cfg.split_cases), 1);

fprintf('\nSaddle-wave and common-mode study\n');
fprintf('modulation ratio = %.4f, Vdc = %.1f V\n', cfg.modulation_ratio, ss_cfg.Vdc);

for case_idx = 1:numel(cfg.split_cases)
    split = cfg.split_cases(case_idx);
    duty = zeros(3, cfg.num_points);
    raw_mod = zeros(3, cfg.num_points);
    mod_wave = zeros(3, cfg.num_points);
    zero_seq = zeros(1, cfg.num_points);
    v_cm_avg = zeros(1, cfg.num_points);

    for k = 1:cfg.num_points
        [va, vb, vc] = local_voltage_vector(v_mag, theta(k));
        raw_mod(:, k) = [va; vb; vc] / ss_cfg.Vdc;
        [duty(:, k), shift] = local_allocate_zero_vector([va; vb; vc], ss_cfg.Vdc, split);
        mod_wave(:, k) = duty(:, k) - 0.5;
        zero_seq(k) = shift;
        v_phase = (duty(:, k) - 0.5) * ss_cfg.Vdc;
        v_cm_avg(k) = mean(v_phase);
    end

    result.case_data(case_idx).name = cfg.case_names{case_idx};
    result.case_data(case_idx).split = split;
    result.case_data(case_idx).duty = duty;
    result.case_data(case_idx).mod_wave = mod_wave;
    result.case_data(case_idx).raw_mod_wave = raw_mod;
    result.case_data(case_idx).zero_sequence_norm = zero_seq;
    result.case_data(case_idx).v_cm_avg = v_cm_avg;
    result.case_data(case_idx).v_cm_mean = mean(v_cm_avg);
    result.case_data(case_idx).v_cm_rms = sqrt(mean(v_cm_avg .^ 2));
    result.case_data(case_idx).v_cm_min = min(v_cm_avg);
    result.case_data(case_idx).v_cm_max = max(v_cm_avg);

    fprintf('[%s] split = %.2f, v_cm mean = %.3f V, rms = %.3f V, range = [%.3f, %.3f] V\n', ...
        result.case_data(case_idx).name, split, result.case_data(case_idx).v_cm_mean, ...
        result.case_data(case_idx).v_cm_rms, result.case_data(case_idx).v_cm_min, ...
        result.case_data(case_idx).v_cm_max);
end

local_plot_results(result, ss_cfg);

end

function cfg = local_parse_inputs(cfg, varargin)
if mod(numel(varargin), 2) ~= 0
    error('Arguments must be name/value pairs.');
end

for idx = 1:2:numel(varargin)
    name = varargin{idx};
    value = varargin{idx + 1};
    switch lower(name)
        case 'modulationratio'
            cfg.modulation_ratio = value;
        case 'numpoints'
            cfg.num_points = value;
        case 'splitcases'
            cfg.split_cases = value;
        case 'casenames'
            cfg.case_names = value;
        otherwise
            error('Unknown parameter: %s', name);
    end
end

if numel(cfg.case_names) ~= numel(cfg.split_cases)
    error('caseNames must match splitCases in length.');
end
end

function [va, vb, vc] = local_voltage_vector(v_mag, theta_e)
v_alpha = -v_mag * sin(theta_e);
v_beta = v_mag * cos(theta_e);

va = v_alpha;
vb = -0.5 * v_alpha + sqrt(3) / 2 * v_beta;
vc = -0.5 * v_alpha - sqrt(3) / 2 * v_beta;
end

function [duty, shift] = local_allocate_zero_vector(vabc, Vdc, split)
duty_raw = vabc / Vdc + 0.5;
shift_min = -min(duty_raw);
shift_max = 1.0 - max(duty_raw);
shift = shift_min + split * (shift_max - shift_min);
duty = min(max(duty_raw + shift, 0.0), 1.0);
duty = duty(:);
end

function local_plot_results(result, ss_cfg)
colors = lines(numel(result.case_data));
phase_colors = [0.00 0.45 0.74; 0.85 0.33 0.10; 0.47 0.67 0.19];

figure('Name', 'Saddle Wave and Common-Mode Voltage', 'Color', 'w');
tiledlayout(2 + numel(result.case_data), 1, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
hold on;
for idx = 1:numel(result.case_data)
    plot(result.theta_deg, result.case_data(idx).v_cm_avg, 'Color', colors(idx, :), 'LineWidth', 1.2);
end
grid on;
xlabel('Electrical angle (deg)');
ylabel('v_cm (V)');
title('Average common-mode voltage over one electrical cycle');
legend({result.case_data.name}, 'Location', 'best');

nexttile;
hold on;
for idx = 1:numel(result.case_data)
    plot(result.theta_deg, result.case_data(idx).zero_sequence_norm * ss_cfg.Vdc, ...
        'Color', colors(idx, :), 'LineWidth', 1.2);
end
grid on;
xlabel('Electrical angle (deg)');
ylabel('Zero-sequence shift (V)');
title('Injected zero-sequence offset from V0/V7 reallocation');
legend({result.case_data.name}, 'Location', 'best');

for idx = 1:numel(result.case_data)
    nexttile;
    hold on;
    plot(result.theta_deg, result.case_data(idx).raw_mod_wave(1, :) * ss_cfg.Vdc, '--', 'Color', phase_colors(1, :), 'LineWidth', 1.0);
    plot(result.theta_deg, result.case_data(idx).raw_mod_wave(2, :) * ss_cfg.Vdc, '--', 'Color', phase_colors(2, :), 'LineWidth', 1.0);
    plot(result.theta_deg, result.case_data(idx).raw_mod_wave(3, :) * ss_cfg.Vdc, '--', 'Color', phase_colors(3, :), 'LineWidth', 1.0);
    plot(result.theta_deg, result.case_data(idx).mod_wave(1, :) * ss_cfg.Vdc, 'Color', phase_colors(1, :), 'LineWidth', 1.2);
    plot(result.theta_deg, result.case_data(idx).mod_wave(2, :) * ss_cfg.Vdc, 'Color', phase_colors(2, :), 'LineWidth', 1.2);
    plot(result.theta_deg, result.case_data(idx).mod_wave(3, :) * ss_cfg.Vdc, 'Color', phase_colors(3, :), 'LineWidth', 1.2);
    grid on;
    xlabel('Electrical angle (deg)');
    ylabel('Phase ref (V)');
    title(sprintf('%s saddle wave: dashed = sinusoid, solid = zero-sequence injected', result.case_data(idx).name));
    legend({'A raw', 'B raw', 'C raw', 'A saddle', 'B saddle', 'C saddle'}, 'Location', 'bestoutside');
end

figure('Name', 'Common-Mode Statistics', 'Color', 'w');
stats = zeros(numel(result.case_data), 3);
for idx = 1:numel(result.case_data)
    stats(idx, 1) = result.case_data(idx).v_cm_mean;
    stats(idx, 2) = result.case_data(idx).v_cm_rms;
    stats(idx, 3) = result.case_data(idx).v_cm_max - result.case_data(idx).v_cm_min;
end
bar(stats);
grid on;
set(gca, 'XTickLabel', {result.case_data.name});
ylabel('Voltage (V)');
legend({'Mean', 'RMS', 'Peak-to-peak'}, 'Location', 'best');
title(sprintf('Common-mode metrics, modulation ratio = %.4f', result.cfg.modulation_ratio));
end