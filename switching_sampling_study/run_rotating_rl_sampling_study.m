function result = run_rotating_rl_sampling_study(varargin)
%RUN_ROTATING_RL_SAMPLING_STUDY Rotate the electrical angle through whole cycles.
% The script updates the duty command once per PWM period over one or more
% electrical cycles, applies the MCU sampling-window clamp, drives a simple
% three-phase RL load, and compares sampled current with per-period average
% current across the full rotating trajectory.

switching_sampling_study_config;

cfg.modulation_ratio = ss_cfg.modulation_ratio;
cfg.duty_sample_limit = ss_cfg.duty_sample_limit;
cfg.samples_per_pwm = 1000;
cfg.R_ohm = ss_cfg.rl_load_R_ohm;
cfg.L_h = ss_cfg.rl_load_L_h;
cfg.electrical_cycles = ss_cfg.rl_rotating_cycles;
cfg.electrical_freq_hz = ss_cfg.electrical_freq_hz;

cfg = local_parse_inputs(cfg, varargin{:});

periods_per_cycle = max(1, round(ss_cfg.f_pwm / cfg.electrical_freq_hz));
total_periods = cfg.electrical_cycles * periods_per_cycle;
theta_period_deg = mod((0:(total_periods - 1)) * 360.0 / periods_per_cycle, 360.0);

result.cfg = cfg;
result.periods_per_cycle = periods_per_cycle;
result.total_periods = total_periods;
result.theta_period_deg = theta_period_deg(:);
result.base = local_simulate_rotating_case('Symmetric SVPWM', false, theta_period_deg, cfg, ss_cfg);
result.shifted = local_simulate_rotating_case('MCU sampling shift', true, theta_period_deg, cfg, ss_cfg);

fprintf('\nRotating RL sampling study\n');
fprintf('electrical frequency = %.1f Hz, PWM = %.1f kHz\n', cfg.electrical_freq_hz, ss_cfg.f_pwm * 1e-3);
fprintf('electrical cycles = %d, periods per cycle = %d\n', cfg.electrical_cycles, periods_per_cycle);
fprintf('R = %.3f ohm, L = %.0f uH, duty limit = %.3f\n', cfg.R_ohm, cfg.L_h * 1e6, cfg.duty_sample_limit);
fprintf('sample error RMS: base = %.4f A, shifted = %.4f A\n', ...
    result.base.sample_error_rms_A, result.shifted.sample_error_rms_A);
fprintf('sampled ripple pk-pk: base = %.4f A, shifted = %.4f A\n', ...
    result.base.sampled_phase_ripple_pkpk_A, result.shifted.sampled_phase_ripple_pkpk_A);

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
        case 'dutysamplelimit'
            cfg.duty_sample_limit = value;
        case 'samplesperpwm'
            cfg.samples_per_pwm = value;
        case 'rohm'
            cfg.R_ohm = value;
        case 'lh'
            cfg.L_h = value;
        case 'electricalcycles'
            cfg.electrical_cycles = value;
        case 'electricalfreqhz'
            cfg.electrical_freq_hz = value;
        otherwise
            error('Unknown parameter: %s', name);
    end
end
end

function case_data = local_simulate_rotating_case(name, applyShift, theta_period_deg, cfg, ss_cfg)
total_periods = numel(theta_period_deg);
total_samples = total_periods * cfg.samples_per_pwm;
Ts = ss_cfg.T_pwm / cfg.samples_per_pwm;
t = (0:(total_samples - 1)).' * Ts;

upper_gate = zeros(total_samples, 3);
lower_gate = zeros(total_samples, 3);
v_phase = zeros(total_samples, 3);
i_cm = zeros(total_samples, 1);
i_abc = zeros(total_samples, 3);
duty_per_period = zeros(total_periods, 3);
v_cm_avg_per_period = zeros(total_periods, 1);
sample_value = NaN(total_periods, 3);
period_average = NaN(total_periods, 3);
sample_error = NaN(total_periods, 3);
sample_time_s = NaN(total_periods, 3);
sector_vec = zeros(total_periods, 1);
shift_vec = zeros(total_periods, 1);
sample_mask = false(total_periods, 3);

current_state = [0.0 0.0 0.0];

for period_idx = 1:total_periods
    theta_e = deg2rad(theta_period_deg(period_idx));
    v_mag = cfg.modulation_ratio * (ss_cfg.Vdc / sqrt(3));
    [va, vb, vc] = local_voltage_vector(v_mag, theta_e);
    duty = local_allocate_zero_vector([va; vb; vc], ss_cfg.Vdc, 0.5);
    if applyShift
        [duty, shift_info] = local_apply_mcu_sampling_shift(duty, [va; vb; vc], cfg.duty_sample_limit);
    else
        shift_info.sector = local_determine_sector([va; vb; vc]);
        shift_info.sampled_phase_idx = local_sampled_phases_for_sector(shift_info.sector);
        shift_info.applied_shift = 0.0;
    end

    duty_per_period(period_idx, :) = duty.';
    sector_vec(period_idx) = shift_info.sector;
    shift_vec(period_idx) = shift_info.applied_shift;
    sample_mask(period_idx, shift_info.sampled_phase_idx) = true;

    local_t = ((0:(cfg.samples_per_pwm - 1)).') * Ts;
    carrier = local_center_aligned_carrier(local_t, ss_cfg.T_pwm);
    local_upper = local_pwm_compare(carrier, duty.');
    local_lower = 1 - local_upper;
    [sample_point_s, sample_idx] = local_select_sample_points(duty, cfg.samples_per_pwm, ss_cfg.T_pwm, shift_info.sampled_phase_idx);

    global_range = ((period_idx - 1) * cfg.samples_per_pwm + 1):(period_idx * cfg.samples_per_pwm);
    upper_gate(global_range, :) = local_upper;
    lower_gate(global_range, :) = local_lower;

    local_current = zeros(cfg.samples_per_pwm, 3);
    local_current(1, :) = current_state;
    local_v_phase = zeros(cfg.samples_per_pwm, 3);
    local_v_cm = zeros(cfg.samples_per_pwm, 1);
    for k = 1:(cfg.samples_per_pwm - 1)
        pole_v = (2.0 * local_upper(k, :) - 1.0) * (ss_cfg.Vdc / 2.0);
        neutral_v = mean(pole_v);
        local_v_cm(k) = neutral_v;
        local_v_phase(k, :) = pole_v - neutral_v;
        di = (local_v_phase(k, :) - cfg.R_ohm * local_current(k, :)) / cfg.L_h;
        local_current(k + 1, :) = local_current(k, :) + Ts * di;
    end
    pole_v = (2.0 * local_upper(end, :) - 1.0) * (ss_cfg.Vdc / 2.0);
    neutral_v = mean(pole_v);
    local_v_cm(end) = neutral_v;
    local_v_phase(end, :) = pole_v - neutral_v;

    current_state = local_current(end, :);
    i_abc(global_range, :) = local_current;
    v_phase(global_range, :) = local_v_phase;
    i_cm(global_range) = local_v_cm;
    v_cm_avg_per_period(period_idx) = mean(local_v_cm);

    for phase_idx = 1:3
        period_average(period_idx, phase_idx) = mean(local_current(:, phase_idx));
    end
    for idx = 1:numel(shift_info.sampled_phase_idx)
        phase_idx = shift_info.sampled_phase_idx(idx);
        sample_value(period_idx, phase_idx) = local_current(sample_idx(idx), phase_idx);
        sample_error(period_idx, phase_idx) = sample_value(period_idx, phase_idx) - period_average(period_idx, phase_idx);
        sample_time_s(period_idx, phase_idx) = t(global_range(1)) + sample_point_s(idx);
    end
end

valid_errors = sample_error(~isnan(sample_error));
if isempty(valid_errors)
    sample_error_rms_A = 0.0;
else
    sample_error_rms_A = sqrt(mean(valid_errors .^ 2));
end

tail_periods = max(1, total_periods - periods_window(total_periods) + 1):total_periods;
tail_sample_range = ((tail_periods(1) - 1) * cfg.samples_per_pwm + 1):(tail_periods(end) * cfg.samples_per_pwm);
sampled_phase_ripple_pkpk_A = 0.0;
for phase_idx = 1:3
    if any(sample_mask(:, phase_idx))
        current_tail = i_abc(tail_sample_range, phase_idx);
        sampled_phase_ripple_pkpk_A = max(sampled_phase_ripple_pkpk_A, max(current_tail) - min(current_tail));
    end
end

case_data.name = name;
case_data.time = t;
case_data.i_abc = i_abc;
case_data.v_phase = v_phase;
case_data.v_cm = i_cm;
case_data.upper_gate = upper_gate;
case_data.lower_gate = lower_gate;
case_data.theta_period_deg = theta_period_deg(:);
case_data.duty_per_period = duty_per_period;
case_data.v_cm_avg_per_period = v_cm_avg_per_period;
case_data.sample_value_A = sample_value;
case_data.period_average_A = period_average;
case_data.sample_error_A = sample_error;
case_data.sample_time_s = sample_time_s;
case_data.sample_mask = sample_mask;
case_data.sector_vec = sector_vec;
case_data.shift_vec = shift_vec;
case_data.sample_error_rms_A = sample_error_rms_A;
case_data.sampled_phase_ripple_pkpk_A = sampled_phase_ripple_pkpk_A;
end

function count = periods_window(total_periods)
count = min(total_periods, 20);
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

function local_plot_results(result, ss_cfg)
phase_names = {'A', 'B', 'C'};

figure('Name', 'Rotating RL Sampling Summary', 'Color', 'w');
tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(result.theta_period_deg, result.base.shift_vec, '--', 'Color', [0 0.45 0.74], 'LineWidth', 1.0); hold on;
plot(result.theta_period_deg, result.shifted.shift_vec, '--', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.0);
plot(result.theta_period_deg, result.base.v_cm_avg_per_period, 'Color', [0 0.45 0.74], 'LineWidth', 1.2);
plot(result.theta_period_deg, result.shifted.v_cm_avg_per_period, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.2);
grid on;
xlabel('Electrical angle (deg)');
ylabel('Shift / v_{cm}');
title('Per-period common shift and common-mode voltage over one rotating electrical cycle');
legend({'Base shift', 'MCU shift', 'Base v_cm', 'MCU v_cm'}, 'Location', 'best');

nexttile;
plot(result.theta_period_deg, result.base.duty_per_period(:, 1), '--', 'Color', [0 0.45 0.74], 'LineWidth', 1.0); hold on;
plot(result.theta_period_deg, result.shifted.duty_per_period(:, 1), 'Color', [0 0.45 0.74], 'LineWidth', 1.2);
plot(result.theta_period_deg, result.base.duty_per_period(:, 2), '--', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.0);
plot(result.theta_period_deg, result.shifted.duty_per_period(:, 2), 'Color', [0.85 0.33 0.10], 'LineWidth', 1.2);
plot(result.theta_period_deg, result.base.duty_per_period(:, 3), '--', 'Color', [0.47 0.67 0.19], 'LineWidth', 1.0);
plot(result.theta_period_deg, result.shifted.duty_per_period(:, 3), 'Color', [0.47 0.67 0.19], 'LineWidth', 1.2);
grid on;
xlabel('Electrical angle (deg)');
ylabel('Duty');
title('Duty trajectory across the electrical cycle');
legend({'A base', 'A shift', 'B base', 'B shift', 'C base', 'C shift'}, 'Location', 'best');

nexttile;
bar([result.base.sample_error_rms_A, result.shifted.sample_error_rms_A; ...
    result.base.sampled_phase_ripple_pkpk_A, result.shifted.sampled_phase_ripple_pkpk_A]);
grid on;
set(gca, 'XTickLabel', {'Sample error RMS', 'Ripple pk-pk'});
ylabel('Current (A)');
legend({'Base', 'MCU shift'}, 'Location', 'best');
title('Rotating RL current metrics');

figure('Name', 'Rotating RL Sample Error by Phase', 'Color', 'w');
tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
for phase_idx = 1:3
    nexttile;
    hold on;
    base_mask = result.base.sample_mask(:, phase_idx);
    shifted_mask = result.shifted.sample_mask(:, phase_idx);
    plot(result.theta_period_deg(base_mask), result.base.sample_error_A(base_mask, phase_idx), 'o', ...
        'Color', [0 0.45 0.74], 'MarkerSize', 4);
    plot(result.theta_period_deg(shifted_mask), result.shifted.sample_error_A(shifted_mask, phase_idx), 's', ...
        'Color', [0.85 0.33 0.10], 'MarkerSize', 4);
    grid on;
    xlabel('Electrical angle (deg)');
    ylabel(sprintf('e_{%s} (A)', phase_names{phase_idx}));
    title(sprintf('Sample error on phase %s when it is actively sampled', phase_names{phase_idx}));
    legend({'Base', 'MCU shift'}, 'Location', 'best');
end

figure('Name', 'Rotating RL Current vs Sampled Current', 'Color', 'w');
tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
for phase_idx = 1:3
    nexttile;
    hold on;
    plot(result.theta_period_deg, result.base.period_average_A(:, phase_idx), '--', 'Color', [0 0.45 0.74], 'LineWidth', 1.0);
    plot(result.theta_period_deg, result.shifted.period_average_A(:, phase_idx), '--', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.0);
    base_mask = result.base.sample_mask(:, phase_idx);
    shifted_mask = result.shifted.sample_mask(:, phase_idx);
    plot(result.theta_period_deg(base_mask), result.base.sample_value_A(base_mask, phase_idx), 'o', 'Color', [0 0.45 0.74], 'MarkerSize', 4);
    plot(result.theta_period_deg(shifted_mask), result.shifted.sample_value_A(shifted_mask, phase_idx), 's', 'Color', [0.85 0.33 0.10], 'MarkerSize', 4);
    grid on;
    xlabel('Electrical angle (deg)');
    ylabel(sprintf('i_%s (A)', phase_names{phase_idx}));
    title(sprintf('Phase %s sampled current across the rotating electrical cycle', phase_names{phase_idx}));
    legend({'Base avg', 'Shift avg', 'Base sampled', 'Shift sampled'}, 'Location', 'best');
end

tail_samples = min(numel(result.base.time), 2 * result.cfg.samples_per_pwm);
tail_idx = (numel(result.base.time) - tail_samples + 1):numel(result.base.time);
t_us = result.base.time(tail_idx) * 1e6;
figure('Name', 'Rotating RL Tail Waveform', 'Color', 'w');
tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
for phase_idx = 1:3
    nexttile;
    hold on;
    plot(t_us, result.shifted.i_abc(tail_idx, phase_idx), 'Color', [0 0.45 0.74], 'LineWidth', 1.1);
    plot(t_us, result.shifted.v_phase(tail_idx, phase_idx) / max(ss_cfg.Vdc / 2.0, 1.0), '--', 'Color', [0.47 0.67 0.19], 'LineWidth', 1.0);
    plot(t_us, result.shifted.v_cm(tail_idx) / max(ss_cfg.Vdc / 2.0, 1.0), ':', 'Color', [0.49 0.18 0.56], 'LineWidth', 1.0);
    grid on;
    xlabel('Time (us)');
    ylabel(sprintf('i_%s (A)', phase_names{phase_idx}));
    title(sprintf('Tail waveform on phase %s for rotating MCU-shift case', phase_names{phase_idx}));
    legend({'current', 'v_phase / (Vdc/2)', 'v_cm / (Vdc/2)'}, 'Location', 'best');
end
end