function result = run_triangle_carrier_study(varargin)
%RUN_TRIANGLE_CARRIER_STUDY Study center-aligned triangular-carrier PWM.
% This script compares different zero-vector allocations by shifting the
% common-mode component inside the feasible duty range, then builds the
% corresponding center-aligned carrier comparison waveforms.

switching_sampling_study_config;

cfg.theta_e_deg = 35;
cfg.samples_per_pwm = 4000;
cfg.modulation_ratio = ss_cfg.modulation_ratio;
cfg.split_cases = ss_cfg.v0v7_splits;
cfg.case_names = ss_cfg.case_names;

cfg = local_parse_inputs(cfg, varargin{:});

theta_e = deg2rad(cfg.theta_e_deg);
v_mag = cfg.modulation_ratio * (ss_cfg.Vdc / sqrt(3));
[va, vb, vc] = local_voltage_vector(v_mag, theta_e);

t = linspace(0, ss_cfg.T_pwm, cfg.samples_per_pwm).';
carrier = local_center_aligned_carrier(t, ss_cfg.T_pwm);

result.cfg = cfg;
result.time = t;
result.carrier = carrier;
result.case_data = repmat(struct( ...
    'name', '', ...
    'split', 0, ...
    'duty', zeros(3, 1), ...
    'upper_gate', zeros(numel(t), 3), ...
    'lower_gate', zeros(numel(t), 3), ...
    'low_side_windows_s', zeros(3, 1), ...
    'sample_window_start_s', zeros(3, 1), ...
    'sample_window_end_s', zeros(3, 1), ...
    'sample_point_s', zeros(3, 1), ...
    'min_low_window_s', 0, ...
    'second_low_window_s', 0), numel(cfg.split_cases), 1);

fprintf('\nTriangle carrier PWM study\n');
fprintf('theta_e = %.1f deg, modulation ratio = %.3f, f_pwm = %.1f kHz\n', ...
    cfg.theta_e_deg, cfg.modulation_ratio, 1e-3 / ss_cfg.T_pwm);

for case_idx = 1:numel(cfg.split_cases)
    split = cfg.split_cases(case_idx);
    duty = local_allocate_zero_vector([va; vb; vc], ss_cfg.Vdc, split);
    upper_gate = local_pwm_compare(carrier, duty.');
    lower_gate = 1 - upper_gate;

    low_windows = zeros(3, 1);
    low_window_start = zeros(3, 1);
    low_window_end = zeros(3, 1);
    sample_point = zeros(3, 1);
    for phase_idx = 1:3
        [low_windows(phase_idx), low_window_start(phase_idx), low_window_end(phase_idx)] = ...
            local_longest_true_window(t, lower_gate(:, phase_idx));
        sample_point(phase_idx) = 0.5 * (low_window_start(phase_idx) + low_window_end(phase_idx));
    end
    low_sorted = sort(low_windows, 'descend');

    result.case_data(case_idx).name = cfg.case_names{case_idx};
    result.case_data(case_idx).split = split;
    result.case_data(case_idx).duty = duty;
    result.case_data(case_idx).upper_gate = upper_gate;
    result.case_data(case_idx).lower_gate = lower_gate;
    result.case_data(case_idx).low_side_windows_s = low_windows;
    result.case_data(case_idx).sample_window_start_s = low_window_start;
    result.case_data(case_idx).sample_window_end_s = low_window_end;
    result.case_data(case_idx).sample_point_s = sample_point;
    result.case_data(case_idx).min_low_window_s = min(low_windows);
    result.case_data(case_idx).second_low_window_s = low_sorted(2);

    fprintf('[%s] duty = [%.3f %.3f %.3f], min low window = %.2f us, 2nd low window = %.2f us\n', ...
        cfg.case_names{case_idx}, duty(1), duty(2), duty(3), ...
        1e6 * result.case_data(case_idx).min_low_window_s, ...
        1e6 * result.case_data(case_idx).second_low_window_s);
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
        case 'thetaedeg'
            cfg.theta_e_deg = value;
        case 'samplesperpwm'
            cfg.samples_per_pwm = value;
        case 'modulationratio'
            cfg.modulation_ratio = value;
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

function duty = local_allocate_zero_vector(vabc, Vdc, split)
duty_raw = vabc / Vdc + 0.5;
shift_min = -min(duty_raw);
shift_max = 1.0 - max(duty_raw);
shift = shift_min + split * (shift_max - shift_min);
shifted = min(max(duty_raw + shift, 0.0), 1.0);
duty = shifted(:);
end

function upper_gate = local_pwm_compare(carrier, duty_row)
upper_gate = zeros(numel(carrier), numel(duty_row));
for phase_idx = 1:numel(duty_row)
    duty = duty_row(phase_idx);
    if duty <= 0.0
        upper_gate(:, phase_idx) = 0;
    elseif duty >= 1.0
        upper_gate(:, phase_idx) = 1;
    else
        upper_gate(:, phase_idx) = double(carrier < duty);
    end
end
end

function carrier = local_center_aligned_carrier(t, T_pwm)
phase = mod(t / T_pwm, 1.0);
carrier = zeros(size(phase));
up_mask = phase < 0.5;
carrier(up_mask) = 2.0 * phase(up_mask);
carrier(~up_mask) = 2.0 * (1.0 - phase(~up_mask));
end

function [window_s, window_start_s, window_end_s] = local_longest_true_window(t, logic_vec)
logic_vec = logic_vec(:) ~= 0;
dt = mean(diff(t));

if ~any(logic_vec)
    window_s = 0.0;
    window_start_s = NaN;
    window_end_s = NaN;
    return;
end

edge_idx = diff([false; logic_vec; false]);
start_idx = find(edge_idx == 1);
end_idx = find(edge_idx == -1) - 1;
window_lengths = (end_idx - start_idx + 1) * dt;
[window_s, best_idx] = max(window_lengths);
window_start_s = t(start_idx(best_idx));
window_end_s = t(end_idx(best_idx));
end

function local_plot_results(result, ss_cfg)
figure('Name', 'Triangle Carrier PWM Study', 'Color', 'w');
tiledlayout(4, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(result.time * 1e6, result.carrier, 'k', 'LineWidth', 1.2); hold on;
colors = lines(numel(result.case_data));
for idx = 1:numel(result.case_data)
    yline(result.case_data(idx).duty(1), '--', 'Color', colors(idx, :), ...
        'LineWidth', 1.1, 'Label', [result.case_data(idx).name ' duty A']);
end
grid on;
xlabel('Time within one PWM period (us)');
ylabel('Normalized carrier / duty');
title('Center-aligned triangular carrier and phase-A duty levels');

for phase_idx = 1:3
    nexttile;
    hold on;
    for idx = 1:numel(result.case_data)
        stairs(result.time * 1e6, result.case_data(idx).lower_gate(:, phase_idx), ...
            'Color', colors(idx, :), 'LineWidth', 1.1);
    end
    ylim([-0.1 1.1]);
    grid on;
    xlabel('Time within one PWM period (us)');
    ylabel(sprintf('S%d low', phase_idx));
    title(sprintf('Phase-%c low-side gate window', 'A' + phase_idx - 1));
end

figure('Name', 'Low-Side Window Comparison', 'Color', 'w');
bar_data = zeros(numel(result.case_data), 2);
for idx = 1:numel(result.case_data)
    bar_data(idx, 1) = 1e6 * result.case_data(idx).min_low_window_s;
    bar_data(idx, 2) = 1e6 * result.case_data(idx).second_low_window_s;
end
bar(bar_data);
grid on;
set(gca, 'XTickLabel', {result.case_data.name});
ylabel('Window length (us)');
legend({'Min low-side window', '2nd low-side window'}, 'Location', 'best');
title(sprintf('Window metrics at theta_e = %.1f deg', result.cfg.theta_e_deg));

annotation_text = sprintf([ ...
    'PWM period = %.2f us\n' ...
    'Dead time placeholder = %.2f us\n' ...
    'Min valid window target = %.2f us'], ...
    ss_cfg.T_pwm * 1e6, ss_cfg.dead_time_s * 1e6, ss_cfg.min_valid_window_s * 1e6);
annotation('textbox', [0.62 0.76 0.25 0.12], 'String', annotation_text, ...
    'FitBoxToText', 'on', 'BackgroundColor', [0.95 0.98 1.0]);

phase_names = {'A', 'B', 'C'};
for idx = 1:numel(result.case_data)
    figure('Name', ['Carrier and PWM - ' result.case_data(idx).name], 'Color', 'w');
    tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
    for phase_idx = 1:3
        nexttile;
        hold on;
        plot(result.time * 1e6, result.carrier, 'k', 'LineWidth', 1.1);
        yline(result.case_data(idx).duty(phase_idx), '--', 'Color', [0 0.45 0.74], 'LineWidth', 1.0);
        stairs(result.time * 1e6, result.case_data(idx).upper_gate(:, phase_idx), ...
            'Color', [0.85 0.33 0.10], 'LineWidth', 1.1);
        stairs(result.time * 1e6, result.case_data(idx).lower_gate(:, phase_idx), ...
            'Color', [0.47 0.67 0.19], 'LineWidth', 1.1);
        if ~isnan(result.case_data(idx).sample_point_s(phase_idx))
            xline(result.case_data(idx).sample_point_s(phase_idx) * 1e6, '--', ...
                'Color', [0.49 0.18 0.56], 'LineWidth', 1.0);
            scatter(result.case_data(idx).sample_point_s(phase_idx) * 1e6, 0.5, 28, ...
                [0.49 0.18 0.56], 'filled');
        end
        grid on;
        ylim([-0.1 1.1]);
        xlabel('Time within one PWM period (us)');
        ylabel(sprintf('Phase %s', phase_names{phase_idx}));
        title(sprintf('%s: carrier, PWM gates, and candidate sample point', result.case_data(idx).name));
        legend({'carrier', 'duty', 'upper gate', 'lower gate', 'sample point'}, 'Location', 'best');
    end
end
end