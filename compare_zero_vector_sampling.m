%% Compare zero-vector allocation on PWM sampling windows
% This script compares how different V0/V7 allocations change duty cycles
% and the available phase current sampling windows. It does not use sector
% switching logic. Instead, it allocates the total zero-vector interval by
% shifting the common-mode component inside the feasible range.

motor_control_params;

Ts_pwm = simcfg.Ts_ctrl;
Vdc = inverter.Vdc;

cfg.modulation_ratio = 0.90;   % relative to linear SVPWM limit
cfg.num_points = 721;
cfg.splits = [0.0, 0.5, 1.0]; % 0: all V0, 0.5: symmetric, 1: all V7
cfg.labels = {'All V0', 'Symmetric V0/V7', 'All V7'};

theta = linspace(0, 2*pi, cfg.num_points);
v_mag = cfg.modulation_ratio * inverter.modulation_limit;

duty = zeros(3, cfg.num_points, numel(cfg.splits));
t_low = zeros(3, cfg.num_points, numel(cfg.splits));
t_high = zeros(3, cfg.num_points, numel(cfg.splits));
worst_low = zeros(cfg.num_points, numel(cfg.splits));
second_low = zeros(cfg.num_points, numel(cfg.splits));
worst_high = zeros(cfg.num_points, numel(cfg.splits));

for split_idx = 1:numel(cfg.splits)
    split = cfg.splits(split_idx);
    for k = 1:cfg.num_points
        [va, vb, vc] = local_voltage_vector(v_mag, theta(k));
        [da, db, dc] = local_allocate_zero_vector(va, vb, vc, Vdc, split);

        d = [da; db; dc];
        duty(:, k, split_idx) = d;
        t_low(:, k, split_idx) = (1.0 - d) * Ts_pwm;
        t_high(:, k, split_idx) = d * Ts_pwm;

        low_sorted = sort(t_low(:, k, split_idx), 'descend');
        worst_low(k, split_idx) = min(t_low(:, k, split_idx));
        second_low(k, split_idx) = low_sorted(2);
        worst_high(k, split_idx) = min(t_high(:, k, split_idx));
    end
end

fprintf('\nZero-vector allocation comparison\n');
fprintf('Vdc = %.2f V, Ts = %.1f us, modulation ratio = %.2f\n', ...
    Vdc, Ts_pwm * 1e6, cfg.modulation_ratio);
for split_idx = 1:numel(cfg.splits)
    fprintf('\n[%s]\n', cfg.labels{split_idx});
    fprintf('  Min low-side window      : %.2f us\n', 1e6 * min(worst_low(:, split_idx)));
    fprintf('  Min 2nd low-side window  : %.2f us\n', 1e6 * min(second_low(:, split_idx)));
    fprintf('  Min high-side window     : %.2f us\n', 1e6 * min(worst_high(:, split_idx)));
    fprintf('  Duty range               : [%.3f, %.3f]\n', ...
        min(duty(:, :, split_idx), [], 'all'), max(duty(:, :, split_idx), [], 'all'));
end

theta_deg = theta * 180 / pi;

figure('Name', 'Zero Vector Allocation vs Sampling Window', 'Color', 'w');
tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(theta_deg, squeeze(duty(1, :, 1)), '--', 'LineWidth', 1.0); hold on;
plot(theta_deg, squeeze(duty(1, :, 2)), '-', 'LineWidth', 1.2);
plot(theta_deg, squeeze(duty(1, :, 3)), ':', 'LineWidth', 1.4);
grid on;
xlabel('Electrical angle (deg)');
ylabel('Duty A');
title('Phase-A duty under different V0/V7 allocations');
legend(cfg.labels, 'Location', 'best');

nexttile;
plot(theta_deg, 1e6 * worst_low(:, 1), '--', 'LineWidth', 1.0); hold on;
plot(theta_deg, 1e6 * worst_low(:, 2), '-', 'LineWidth', 1.2);
plot(theta_deg, 1e6 * worst_low(:, 3), ':', 'LineWidth', 1.4);
grid on;
xlabel('Electrical angle (deg)');
ylabel('Worst low-side window (us)');
title('Conservative current-sampling window metric');
legend(cfg.labels, 'Location', 'best');

nexttile;
plot(theta_deg, 1e6 * second_low(:, 1), '--', 'LineWidth', 1.0); hold on;
plot(theta_deg, 1e6 * second_low(:, 2), '-', 'LineWidth', 1.2);
plot(theta_deg, 1e6 * second_low(:, 3), ':', 'LineWidth', 1.4);
grid on;
xlabel('Electrical angle (deg)');
ylabel('2nd best low-side window (us)');
title('Two-phase sampling friendly window metric');
legend(cfg.labels, 'Location', 'best');

figure('Name', 'All Three Duties', 'Color', 'w');
phase_names = {'Duty A', 'Duty B', 'Duty C'};
for phase_idx = 1:3
    subplot(3, 1, phase_idx);
    plot(theta_deg, squeeze(duty(phase_idx, :, 1)), '--', 'LineWidth', 1.0); hold on;
    plot(theta_deg, squeeze(duty(phase_idx, :, 2)), '-', 'LineWidth', 1.2);
    plot(theta_deg, squeeze(duty(phase_idx, :, 3)), ':', 'LineWidth', 1.4);
    grid on;
    ylabel(phase_names{phase_idx});
    if phase_idx == 1
        title('Duty comparison for all phases');
    end
    if phase_idx == 3
        xlabel('Electrical angle (deg)');
    end
end
legend(cfg.labels, 'Location', 'best');

function [va, vb, vc] = local_voltage_vector(v_mag, theta_e)
    v_alpha = -v_mag * sin(theta_e);
    v_beta = v_mag * cos(theta_e);

    va = v_alpha;
    vb = -0.5 * v_alpha + sqrt(3) / 2 * v_beta;
    vc = -0.5 * v_alpha - sqrt(3) / 2 * v_beta;
end

function [da, db, dc] = local_allocate_zero_vector(va, vb, vc, Vdc, split)
    duty_raw = [va; vb; vc] / Vdc + 0.5;

    shift_min = -min(duty_raw);
    shift_max = 1.0 - max(duty_raw);
    shift = shift_min + split * (shift_max - shift_min);

    duty = duty_raw + shift;
    duty = min(max(duty, 0.0), 1.0);

    da = duty(1);
    db = duty(2);
    dc = duty(3);
end