function result = plot_phase_a_deadtime_comparison(varargin)
%PLOT_PHASE_A_DEADTIME_COMPARISON Plot phase-A PWM with and without dead time.
% This script mirrors the current switching-study logic for phase A and
% overlays the ideal complementary gates with the dead-time-adjusted gates.

switching_sampling_study_config;

cfg.theta_e_deg = ss_cfg.theta_e_deg;
cfg.modulation_ratio = ss_cfg.modulation_ratio;
cfg.samples_per_pwm = 4000;
cfg.periods = 2;
cfg.dead_time_s = ss_cfg.dead_time_s;

cfg = local_parse_inputs(cfg, varargin{:});

Ts_model = ss_cfg.T_pwm / cfg.samples_per_pwm;
t = (0:(cfg.periods * cfg.samples_per_pwm - 1)).' * Ts_model;

duty_a = local_phase_a_duty(cfg.theta_e_deg, cfg.modulation_ratio, ss_cfg.Vdc);
carrier = local_center_aligned_carrier(t, ss_cfg.T_pwm);
pwm_cmd = local_pwm_compare(carrier, duty_a);

s_high_ideal = double(pwm_cmd);
s_low_ideal = double(~pwm_cmd);

[s_high_dead, s_low_dead] = local_apply_deadtime(pwm_cmd, cfg.dead_time_s, Ts_model);

result.time = t;
result.duty_a = duty_a;
result.carrier = carrier;
result.pwm_cmd = double(pwm_cmd);
result.s_high_ideal = s_high_ideal;
result.s_low_ideal = s_low_ideal;
result.s_high_dead = double(s_high_dead);
result.s_low_dead = double(s_low_dead);
result.Ts_model = Ts_model;
result.dead_time_s = cfg.dead_time_s;

fprintf('\nPhase-A dead-time comparison\n');
fprintf('theta_e = %.1f deg\n', cfg.theta_e_deg);
fprintf('modulation ratio = %.3f\n', cfg.modulation_ratio);
fprintf('duty_a = %.4f\n', duty_a);
fprintf('T_pwm = %.2f us, Ts_model = %.3f us\n', ss_cfg.T_pwm * 1e6, Ts_model * 1e6);
fprintf('dead_time = %.3f us\n', cfg.dead_time_s * 1e6);

local_plot(result);

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
        case 'samplesperpwm'
            cfg.samples_per_pwm = value;
        case 'periods'
            cfg.periods = value;
        case {'deadtimes', 'dead_time_s', 'deadtime'}
            cfg.dead_time_s = value;
        otherwise
            error('Unknown parameter: %s', name);
    end
end
end

function da = local_phase_a_duty(theta_e_deg, modulation_ratio, Vdc)
theta_e = theta_e_deg * pi / 180;
v_mag = modulation_ratio * (Vdc / sqrt(3));
v_alpha = -v_mag * sin(theta_e);
v_beta =  v_mag * cos(theta_e);

va = v_alpha;
vb = -0.5 * v_alpha + sqrt(3) / 2 * v_beta;
vc = -0.5 * v_alpha - sqrt(3) / 2 * v_beta;

duty_raw = [va; vb; vc] / Vdc + 0.5;
shift_min = -min(duty_raw);
shift_max = 1.0 - max(duty_raw);
shift = 0.5 * (shift_min + shift_max);
duty = min(max(duty_raw + shift, 0.0), 1.0);
da = duty(1);
end

function carrier = local_center_aligned_carrier(t, T_pwm)
phase = mod(t / T_pwm, 1.0);
carrier = zeros(size(phase));
up_mask = phase < 0.5;
carrier(up_mask) = 2.0 * phase(up_mask);
carrier(~up_mask) = 2.0 * (1.0 - phase(~up_mask));
end

function pwm_cmd = local_pwm_compare(carrier, duty)
if duty <= 0.0
    pwm_cmd = false(size(carrier));
elseif duty >= 1.0
    pwm_cmd = true(size(carrier));
else
    pwm_cmd = carrier < duty;
end
end

function [s_high, s_low] = local_apply_deadtime(pwm_cmd, dead_time, Ts_model)
num_samples = numel(pwm_cmd);
s_high = false(num_samples, 1);
s_low = false(num_samples, 1);

if dead_time <= 0
    s_high = pwm_cmd >= 0.5;
    s_low = ~s_high;
    return;
end

samples_dt = max(1, ceil(dead_time / Ts_model));
active_state = pwm_cmd(1) >= 0.5;
pending_state = active_state;
dead_count = 0;

for k = 1:num_samples
    requested_state = pwm_cmd(k) >= 0.5;

    if dead_count > 0
        pending_state = requested_state;
        dead_count = dead_count - 1;
        if dead_count == 0
            active_state = pending_state;
        else
            active_state = -1;
        end
    elseif requested_state ~= active_state
        pending_state = requested_state;
        active_state = -1;
        dead_count = samples_dt;
    end

    if active_state == 1
        s_high(k) = true;
        s_low(k) = false;
    elseif active_state == 0
        s_high(k) = false;
        s_low(k) = true;
    else
        s_high(k) = false;
        s_low(k) = false;
    end
end
end

function local_plot(result)
t_us = result.time * 1e6;

figure('Name', 'Phase-A Dead-Time Comparison', 'Color', 'w');
tiledlayout(4, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(t_us, result.carrier, 'k', 'LineWidth', 1.1); hold on;
yline(result.duty_a, '--', 'Color', [0 0.45 0.74], 'LineWidth', 1.2);
grid on;
xlabel('Time (us)');
ylabel('Carrier / duty');
title('Phase-A duty against center-aligned triangular carrier');
legend({'carrier', 'duty A'}, 'Location', 'best');

nexttile;
stairs(t_us, result.pwm_cmd, 'Color', [0.2 0.2 0.2], 'LineWidth', 1.1); hold on;
stairs(t_us, result.s_high_dead, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.1);
grid on;
ylim([-0.1 1.1]);
xlabel('Time (us)');
ylabel('High gate');
title('A-phase high-side: ideal PWM command vs dead-time output');
legend({'ideal pwm cmd', 'with dead time'}, 'Location', 'best');

nexttile;
stairs(t_us, result.s_low_ideal, 'Color', [0.2 0.2 0.2], 'LineWidth', 1.1); hold on;
stairs(t_us, result.s_low_dead, 'Color', [0 0.45 0.74], 'LineWidth', 1.1);
grid on;
ylim([-0.1 1.1]);
xlabel('Time (us)');
ylabel('Low gate');
title('A-phase low-side: ideal complementary gate vs dead-time output');
legend({'ideal low gate', 'with dead time'}, 'Location', 'best');

nexttile;
dead_zone = double(~result.s_high_dead & ~result.s_low_dead);
stairs(t_us, dead_zone, 'Color', [0.49 0.18 0.56], 'LineWidth', 1.1);
grid on;
ylim([-0.1 1.1]);
xlabel('Time (us)');
ylabel('Dead zone');
title('Inserted dead-time intervals (both gates off)');
end