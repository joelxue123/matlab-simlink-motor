function result = run_mcu_sampling_window_study(varargin)
%RUN_MCU_SAMPLING_WINDOW_STUDY Reproduce the MCU zero-sequence duty shift logic.
% The study starts from symmetric SVPWM duties, applies the same sector-
% dependent duty clamp used on the MCU, and visualizes the carrier, PWM
% gates, and candidate sampling windows before and after the extra shift.

switching_sampling_study_config;

cfg.theta_e_deg = ss_cfg.theta_e_deg;
cfg.samples_per_pwm = 4000;
cfg.modulation_ratio = ss_cfg.modulation_ratio;
cfg.duty_sample_limit = ss_cfg.duty_sample_limit;

cfg = local_parse_inputs(cfg, varargin{:});

theta_e = deg2rad(cfg.theta_e_deg);
v_mag = cfg.modulation_ratio * (ss_cfg.Vdc / sqrt(3));
[va, vb, vc] = local_voltage_vector(v_mag, theta_e);

t = linspace(0, ss_cfg.T_pwm, cfg.samples_per_pwm).';
carrier = local_center_aligned_carrier(t, ss_cfg.T_pwm);

base_duty = local_allocate_zero_vector([va; vb; vc], ss_cfg.Vdc, 0.5);
[shifted_duty, shift_info] = local_apply_mcu_sampling_shift(base_duty, [va; vb; vc], cfg.duty_sample_limit);

result.cfg = cfg;
result.time = t;
result.carrier = carrier;
result.shift_info = shift_info;
result.base = local_build_case('Symmetric SVPWM', base_duty, carrier, t, shift_info.sampled_phase_idx);
result.shifted = local_build_case('MCU sampling shift', shifted_duty, carrier, t, shift_info.sampled_phase_idx);
result.v_cm_base = mean((base_duty - 0.5) * ss_cfg.Vdc);
result.v_cm_shifted = mean((shifted_duty - 0.5) * ss_cfg.Vdc);

fprintf('\nMCU sampling-window study\n');
fprintf('theta_e = %.1f deg, modulation ratio = %.5f\n', cfg.theta_e_deg, cfg.modulation_ratio);
fprintf('sector = %d, sampled phases = %s, duty limit = %.3f\n', ...
    shift_info.sector, shift_info.sampled_phase_label, cfg.duty_sample_limit);
fprintf('base duty    = [%.4f %.4f %.4f]\n', base_duty(1), base_duty(2), base_duty(3));
fprintf('shifted duty = [%.4f %.4f %.4f]\n', shifted_duty(1), shifted_duty(2), shifted_duty(3));
fprintf('applied common shift = %.4f\n', shift_info.applied_shift);
fprintf('min sampled low window: base = %.2f us, shifted = %.2f us\n', ...
    1e6 * result.base.min_sampled_low_window_s, 1e6 * result.shifted.min_sampled_low_window_s);
fprintf('common-mode average:   base = %.3f V, shifted = %.3f V\n', ...
    result.v_cm_base, result.v_cm_shifted);

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
        case 'dutysamplelimit'
            cfg.duty_sample_limit = value;
        otherwise
            error('Unknown parameter: %s', name);
    end
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
duty = min(max(duty_raw + shift, 0.0), 1.0);
duty = duty(:);
end

function carrier = local_center_aligned_carrier(t, T_pwm)
phase = mod(t / T_pwm, 1.0);
carrier = zeros(size(phase));
up_mask = phase < 0.5;
carrier(up_mask) = 2.0 * phase(up_mask);
carrier(~up_mask) = 2.0 * (1.0 - phase(~up_mask));
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

function [shifted_duty, info] = local_apply_mcu_sampling_shift(base_duty, vabc, duty_limit)
sector = local_determine_sector(vabc);
sampled_phase_idx = local_sampled_phases_for_sector(sector);
sampled_duties = base_duty(sampled_phase_idx);
max_sampled = max(sampled_duties);

applied_shift = 0.0;
requested_shift = 0.0;
clamped_by_min = false;
shifted_duty = base_duty;

if max_sampled > duty_limit
    requested_shift = -(max_sampled - duty_limit);
    min_duty = min(base_duty);
    applied_shift = requested_shift;
    if (min_duty + applied_shift) < 0.0
        applied_shift = -min_duty;
        clamped_by_min = true;
    end
    shifted_duty = base_duty + applied_shift;
end

info.sector = sector;
info.sampled_phase_idx = sampled_phase_idx;
info.sampled_phase_label = local_phase_label(sampled_phase_idx);
info.max_sampled = max_sampled;
info.requested_shift = requested_shift;
info.applied_shift = applied_shift;
info.clamped_by_min = clamped_by_min;
end

function sector = local_determine_sector(vabc)
va = vabc(1);
vb = vabc(2);
vc = vabc(3);

if (va >= vb) && (vb >= vc)
    sector = 1;
elseif (vb >= va) && (va >= vc)
    sector = 2;
elseif (vb >= vc) && (vc >= va)
    sector = 3;
elseif (vc >= vb) && (vb >= va)
    sector = 4;
elseif (vc >= va) && (va >= vb)
    sector = 5;
else
    sector = 6;
end
end

function sampled_phase_idx = local_sampled_phases_for_sector(sector)
switch sector
    case {1, 6}
        sampled_phase_idx = [2 3];
    case {2, 3}
        sampled_phase_idx = [1 3];
    case {4, 5}
        sampled_phase_idx = [1 2];
    otherwise
        sampled_phase_idx = [1 2];
end
end

function label = local_phase_label(phase_idx)
phase_names = 'ABC';
label = sprintf('%c/%c', phase_names(phase_idx(1)), phase_names(phase_idx(2)));
end

function case_data = local_build_case(name, duty, carrier, t, sampled_phase_idx)
upper_gate = local_pwm_compare(carrier, duty.');
lower_gate = 1 - upper_gate;

low_windows = zeros(3, 1);
window_start = zeros(3, 1);
window_end = zeros(3, 1);
sample_point = zeros(3, 1);
for phase_idx = 1:3
    [low_windows(phase_idx), window_start(phase_idx), window_end(phase_idx)] = ...
        local_longest_true_window(t, lower_gate(:, phase_idx));
    sample_point(phase_idx) = 0.5 * (window_start(phase_idx) + window_end(phase_idx));
end

case_data.name = name;
case_data.duty = duty;
case_data.upper_gate = upper_gate;
case_data.lower_gate = lower_gate;
case_data.low_side_windows_s = low_windows;
case_data.sample_window_start_s = window_start;
case_data.sample_window_end_s = window_end;
case_data.sample_point_s = sample_point;
case_data.sampled_phase_idx = sampled_phase_idx;
case_data.min_sampled_low_window_s = min(low_windows(sampled_phase_idx));
case_data.min_low_window_s = min(low_windows);
case_data.second_low_window_s = sort(low_windows, 'descend');
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
phase_names = {'A', 'B', 'C'};
cases = {result.base, result.shifted};

figure('Name', 'MCU Sampling Window Summary', 'Color', 'w');
tiledlayout(2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
bar([[result.base.duty.'; result.shifted.duty.']]);
grid on;
set(gca, 'XTickLabel', {'Base', 'MCU shift'});
ylabel('Duty');
legend({'A', 'B', 'C'}, 'Location', 'best');
title(sprintf('Duty comparison, sector %d, sampled phases %s', ...
    result.shift_info.sector, result.shift_info.sampled_phase_label));

nexttile;
bar([[1e6 * result.base.low_side_windows_s.'; 1e6 * result.shifted.low_side_windows_s.']]);
grid on;
set(gca, 'XTickLabel', {'Base', 'MCU shift'});
ylabel('Low-side window (us)');
legend({'A', 'B', 'C'}, 'Location', 'best');
title(sprintf('Low-side window comparison, duty limit = %.3f', result.cfg.duty_sample_limit));

for case_idx = 1:numel(cases)
    case_data = cases{case_idx};
    figure('Name', ['Carrier and PWM - ' case_data.name], 'Color', 'w');
    tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
    for phase_idx = 1:3
        nexttile;
        hold on;
        plot(result.time * 1e6, result.carrier, 'k', 'LineWidth', 1.1);
        yline(case_data.duty(phase_idx), '--', 'Color', [0 0.45 0.74], 'LineWidth', 1.0);
        stairs(result.time * 1e6, case_data.upper_gate(:, phase_idx), 'Color', [0.85 0.33 0.10], 'LineWidth', 1.1);
        stairs(result.time * 1e6, case_data.lower_gate(:, phase_idx), 'Color', [0.47 0.67 0.19], 'LineWidth', 1.1);
        if ~isnan(case_data.sample_point_s(phase_idx))
            xline(case_data.sample_point_s(phase_idx) * 1e6, '--', 'Color', [0.49 0.18 0.56], 'LineWidth', 1.0);
            scatter(case_data.sample_point_s(phase_idx) * 1e6, 0.5, 28, [0.49 0.18 0.56], 'filled');
        end
        grid on;
        ylim([-0.1 1.1]);
        xlabel('Time within one PWM period (us)');
        ylabel(sprintf('Phase %s', phase_names{phase_idx}));
        title(sprintf('%s: carrier, PWM gates, and sample point', case_data.name));
        legend({'carrier', 'duty', 'upper gate', 'lower gate', 'sample point'}, 'Location', 'best');
    end
end

annotation_text = sprintf([ ...
    'Requested shift = %.4f\n' ...
    'Applied shift = %.4f\n' ...
    'Dead time = %.2f us\n' ...
    'ADC settle = %.2f us'], ...
    result.shift_info.requested_shift, result.shift_info.applied_shift, ...
    ss_cfg.dead_time_s * 1e6, ss_cfg.adc_settle_time_s * 1e6);
annotation('textbox', [0.68 0.78 0.22 0.12], 'String', annotation_text, ...
    'FitBoxToText', 'on', 'BackgroundColor', [0.95 0.98 1.0]);
end