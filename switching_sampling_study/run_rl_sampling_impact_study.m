function result = run_rl_sampling_impact_study(varargin)
%RUN_RL_SAMPLING_IMPACT_STUDY Compare RL phase current and sampled current.
% This study reproduces the MCU sector-based duty clamp, drives a simple
% floating-neutral three-phase RL load with the resulting PWM states, and
% overlays the sampled current points on the true current waveforms.

switching_sampling_study_config;

cfg.theta_e_deg = ss_cfg.theta_e_deg;
cfg.modulation_ratio = ss_cfg.modulation_ratio;
cfg.duty_sample_limit = ss_cfg.duty_sample_limit;
cfg.samples_per_pwm = 4000;
cfg.periods = ss_cfg.rl_study_periods;
cfg.R_ohm = ss_cfg.rl_load_R_ohm;
cfg.L_h = ss_cfg.rl_load_L_h;

cfg = local_parse_inputs(cfg, varargin{:});

theta_e = deg2rad(cfg.theta_e_deg);
v_mag = cfg.modulation_ratio * (ss_cfg.Vdc / sqrt(3));
[va, vb, vc] = local_voltage_vector(v_mag, theta_e);
base_duty = local_allocate_zero_vector([va; vb; vc], ss_cfg.Vdc, 0.5);
[shifted_duty, shift_info] = local_apply_mcu_sampling_shift(base_duty, [va; vb; vc], cfg.duty_sample_limit);

result.cfg = cfg;
result.shift_info = shift_info;
result.base = local_simulate_case('Symmetric SVPWM', base_duty, cfg, ss_cfg, shift_info.sampled_phase_idx);
result.shifted = local_simulate_case('MCU sampling shift', shifted_duty, cfg, ss_cfg, shift_info.sampled_phase_idx);

fprintf('\nRL sampling-impact study\n');
fprintf('theta_e = %.1f deg, modulation ratio = %.5f\n', cfg.theta_e_deg, cfg.modulation_ratio);
fprintf('R = %.3f ohm, L = %.0f uH, periods = %d\n', cfg.R_ohm, cfg.L_h * 1e6, cfg.periods);
fprintf('sector = %d, sampled phases = %s, duty limit = %.3f\n', ...
    shift_info.sector, shift_info.sampled_phase_label, cfg.duty_sample_limit);
fprintf('applied common shift = %.4f\n', shift_info.applied_shift);
fprintf('sampled-phase ripple pk-pk: base = %.4f A, shifted = %.4f A\n', ...
    result.base.sampled_phase_ripple_pkpk_A, result.shifted.sampled_phase_ripple_pkpk_A);
fprintf('sampled-current error vs period average: base = %.4f A, shifted = %.4f A\n', ...
    result.base.sample_error_rms_A, result.shifted.sample_error_rms_A);

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
        case 'modulationratio'
            cfg.modulation_ratio = value;
        case 'dutysamplelimit'
            cfg.duty_sample_limit = value;
        case 'samplesperpwm'
            cfg.samples_per_pwm = value;
        case 'periods'
            cfg.periods = value;
        case 'rohm'
            cfg.R_ohm = value;
        case 'lh'
            cfg.L_h = value;
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

function case_data = local_simulate_case(name, duty, cfg, ss_cfg, sampled_phase_idx)
num_samples = cfg.samples_per_pwm * cfg.periods;
Ts = ss_cfg.T_pwm / cfg.samples_per_pwm;
t = (0:(num_samples - 1)).' * Ts;
carrier = local_center_aligned_carrier(t, ss_cfg.T_pwm);
upper_gate = local_pwm_compare(carrier, duty.');
lower_gate = 1 - upper_gate;

[sample_point_one_period, sample_index_one_period] = local_select_sample_points(duty, cfg.samples_per_pwm, ss_cfg.T_pwm, sampled_phase_idx);
period_offsets = ((0:(cfg.periods - 1)).') * cfg.samples_per_pwm;
period_sample_idx = period_offsets + reshape(sample_index_one_period, 1, []);

[i_abc, v_phase] = local_simulate_rl_current(upper_gate, cfg.R_ohm, cfg.L_h, ss_cfg.Vdc, Ts);

valid_periods = max(cfg.periods - 10, 1):cfg.periods;
valid_global_idx = period_sample_idx(valid_periods, :);
sampled_current_A = zeros(numel(valid_periods), numel(sampled_phase_idx));
period_avg_current_A = zeros(numel(valid_periods), numel(sampled_phase_idx));
for phase_local_idx = 1:numel(sampled_phase_idx)
    phase_idx = sampled_phase_idx(phase_local_idx);
    sampled_current_A(:, phase_local_idx) = i_abc(valid_global_idx(:, phase_local_idx), phase_idx);
    for period_pos = 1:numel(valid_periods)
        period_idx = valid_periods(period_pos);
        sample_range = ((period_idx - 1) * cfg.samples_per_pwm + 1):(period_idx * cfg.samples_per_pwm);
        period_avg_current_A(period_pos, phase_local_idx) = mean(i_abc(sample_range, phase_idx));
    end
end

sample_error_A = sampled_current_A - period_avg_current_A;

case_data.name = name;
case_data.time = t;
case_data.carrier = carrier;
case_data.duty = duty;
case_data.upper_gate = upper_gate;
case_data.lower_gate = lower_gate;
case_data.v_phase = v_phase;
case_data.i_abc = i_abc;
case_data.sampled_phase_idx = sampled_phase_idx;
case_data.sample_point_one_period_s = sample_point_one_period;
case_data.sample_index_one_period = sample_index_one_period;
case_data.period_sample_idx = period_sample_idx;
case_data.sampled_current_A = sampled_current_A;
case_data.period_avg_current_A = period_avg_current_A;
case_data.sample_error_A = sample_error_A;
case_data.sample_error_rms_A = sqrt(mean(sample_error_A .^ 2, 'all'));
case_data.sampled_phase_ripple_pkpk_A = local_ripple_pkpk(i_abc, sampled_phase_idx, cfg.samples_per_pwm);
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

function [sample_point_s, sample_idx] = local_select_sample_points(duty, samples_per_pwm, T_pwm, sampled_phase_idx)
t_one = (0:(samples_per_pwm - 1)).' * (T_pwm / samples_per_pwm);
carrier_one = local_center_aligned_carrier(t_one, T_pwm);
upper_gate_one = local_pwm_compare(carrier_one, duty.');
lower_gate_one = 1 - upper_gate_one;

sample_point_s = zeros(1, numel(sampled_phase_idx));
sample_idx = zeros(1, numel(sampled_phase_idx));
for phase_local_idx = 1:numel(sampled_phase_idx)
    phase_idx = sampled_phase_idx(phase_local_idx);
    [~, window_start_s, window_end_s] = local_longest_true_window(t_one, lower_gate_one(:, phase_idx));
    sample_point_s(phase_local_idx) = 0.5 * (window_start_s + window_end_s);
    sample_idx(phase_local_idx) = max(1, min(samples_per_pwm, round(sample_point_s(phase_local_idx) / (T_pwm / samples_per_pwm)) + 1));
end
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

function [i_abc, v_phase] = local_simulate_rl_current(upper_gate, R_ohm, L_h, Vdc, Ts)
num_samples = size(upper_gate, 1);
i_abc = zeros(num_samples, 3);
v_phase = zeros(num_samples, 3);

for k = 1:(num_samples - 1)
    pole_v = (2.0 * upper_gate(k, :) - 1.0) * (Vdc / 2.0);
    neutral_v = mean(pole_v);
    v_phase(k, :) = pole_v - neutral_v;
    di = (v_phase(k, :) - R_ohm * i_abc(k, :)) / L_h;
    i_abc(k + 1, :) = i_abc(k, :) + Ts * di;
end

pole_v = (2.0 * upper_gate(num_samples, :) - 1.0) * (Vdc / 2.0);
neutral_v = mean(pole_v);
v_phase(num_samples, :) = pole_v - neutral_v;
end

function ripple_pkpk_A = local_ripple_pkpk(i_abc, sampled_phase_idx, samples_per_pwm)
tail_samples = min(size(i_abc, 1), 5 * samples_per_pwm);
tail_idx = (size(i_abc, 1) - tail_samples + 1):size(i_abc, 1);
ripple_per_phase = zeros(1, numel(sampled_phase_idx));
for idx = 1:numel(sampled_phase_idx)
    phase_idx = sampled_phase_idx(idx);
    current_tail = i_abc(tail_idx, phase_idx);
    ripple_per_phase(idx) = max(current_tail) - min(current_tail);
end
ripple_pkpk_A = max(ripple_per_phase);
end

function local_plot_results(result, ss_cfg)
cases = {result.base, result.shifted};
phase_names = {'A', 'B', 'C'};

figure('Name', 'RL Sampling Impact Summary', 'Color', 'w');
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
bar([result.base.sampled_phase_ripple_pkpk_A, result.shifted.sampled_phase_ripple_pkpk_A; ...
    result.base.sample_error_rms_A, result.shifted.sample_error_rms_A]);
grid on;
set(gca, 'XTickLabel', {'Ripple pk-pk', 'Sample error RMS'});
ylabel('Current (A)');
legend({'Base', 'MCU shift'}, 'Location', 'best');
title('RL current metrics on sampled phases');

for case_idx = 1:numel(cases)
    case_data = cases{case_idx};
    tail_samples = 2 * result.cfg.samples_per_pwm;
    tail_idx = (numel(case_data.time) - tail_samples + 1):numel(case_data.time);
    t_us = case_data.time(tail_idx) * 1e6;

    figure('Name', ['RL Current and Sampling - ' case_data.name], 'Color', 'w');
    tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
    for phase_local_idx = 1:numel(case_data.sampled_phase_idx)
        phase_idx = case_data.sampled_phase_idx(phase_local_idx);
        nexttile;
        hold on;
        plot(t_us, case_data.i_abc(tail_idx, phase_idx), 'Color', [0 0.45 0.74], 'LineWidth', 1.2);
        sample_idx = case_data.period_sample_idx(:, phase_local_idx);
        sample_tail = sample_idx(sample_idx >= tail_idx(1));
        scatter(case_data.time(sample_tail) * 1e6, case_data.i_abc(sample_tail, phase_idx), 30, ...
            [0.85 0.33 0.10], 'filled');
        plot(t_us, case_data.v_phase(tail_idx, phase_idx) / max(ss_cfg.Vdc / 2.0, 1.0), '--', ...
            'Color', [0.47 0.67 0.19], 'LineWidth', 1.0);
        grid on;
        xlabel('Time (us)');
        ylabel(sprintf('i_%s (A)', phase_names{phase_idx}));
        title(sprintf('%s sampled phase %s: true current, sample points, normalized phase voltage', ...
            case_data.name, phase_names{phase_idx}));
        legend({'true current', 'sample points', 'v_phase / (Vdc/2)'}, 'Location', 'best');
    end
    if numel(case_data.sampled_phase_idx) < 3
        nexttile;
        axis off;
        text(0.0, 0.9, sprintf('R = %.3f ohm\nL = %.0f uH\nSamples per PWM = %d\nPeriods simulated = %d\nSample error RMS = %.4f A', ...
            result.cfg.R_ohm, result.cfg.L_h * 1e6, result.cfg.samples_per_pwm, result.cfg.periods, case_data.sample_error_rms_A), ...
            'VerticalAlignment', 'top');
    end
end

figure('Name', 'RL Sample vs Period Average', 'Color', 'w');
tiledlayout(numel(result.base.sampled_phase_idx), 1, 'Padding', 'compact', 'TileSpacing', 'compact');
for phase_local_idx = 1:numel(result.base.sampled_phase_idx)
    phase_idx = result.base.sampled_phase_idx(phase_local_idx);
    nexttile;
    hold on;
    period_axis = 1:size(result.base.sampled_current_A, 1);
    plot(period_axis, result.base.sampled_current_A(:, phase_local_idx), 'o-', 'Color', [0 0.45 0.74], 'LineWidth', 1.1);
    plot(period_axis, result.shifted.sampled_current_A(:, phase_local_idx), 's-', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.1);
    plot(period_axis, result.base.period_avg_current_A(:, phase_local_idx), '--', 'Color', [0.47 0.67 0.19], 'LineWidth', 1.0);
    grid on;
    xlabel('Tail period index');
    ylabel(sprintf('i_%s (A)', phase_names{phase_idx}));
    title(sprintf('Sampled current vs period average for phase %s', phase_names{phase_idx}));
    legend({'base sampled', 'shifted sampled', 'base period avg'}, 'Location', 'best');
end
end