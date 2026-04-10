function [iq_ref_cmd, iq_ff, learn_active] = vibration_compensator(theta_meas, iq_ref_base, w_ref, w_meas, t_now, params)
% Mechanical-angle synchronous vibration compensation.
%
% Inputs:
%   theta_meas  - unwrapped mechanical angle (rad)
%   iq_ref_base - speed PI output before vibration compensation (A)
%   w_ref       - speed reference (rad/s)
%   w_meas      - measured speed (rad/s)
%   t_now       - simulation time (s)
%   params      - [points, learning_rate, phase_advance_deg, output_limit,
%                  mean_alpha, min_speed_abs, speed_err_threshold,
%                  learn_start_time, enable_learning, enable_ff,
%                  ff_enable_time]
%
% Outputs:
%   iq_ref_cmd  - compensated q-axis current command
%   iq_ff       - learned feedforward contribution
%   learn_active - 1 when learning is active, else 0

MAX_POINTS = 360;
persistent table iq_mean prev_points
if isempty(table)
    table = zeros(MAX_POINTS, 1);
    iq_mean = 0;
    prev_points = 0;
end

points = min(MAX_POINTS, max(8, round(params(1))));
learning_rate = min(1, max(0, params(2)));
phase_advance_rad = params(3) * pi / 180;
output_limit = max(0, params(4));
mean_alpha = min(1, max(0, params(5)));
min_speed_abs = max(0, params(6));
speed_err_threshold = max(0, params(7));
learn_start_time = max(0, params(8));
enable_learning = params(9) > 0.5;
enable_ff = params(10) > 0.5;
ff_enable_time = max(0, params(11));

if prev_points ~= points
    table(:) = 0;
    iq_mean = 0;
    prev_points = points;
end

theta_wrap = mod(theta_meas, 2 * pi);
if theta_wrap < 0
    theta_wrap = theta_wrap + 2 * pi;
end

theta_adv = mod(theta_wrap + phase_advance_rad, 2 * pi);

idx = 1 + floor(theta_wrap / (2 * pi) * points);
idx_adv = 1 + floor(theta_adv / (2 * pi) * points);
if idx > points
    idx = points;
end
if idx_adv > points
    idx_adv = points;
end

iq_mean = (1 - mean_alpha) * iq_mean + mean_alpha * iq_ref_base;
learn_signal = iq_ref_base - iq_mean;

learn_condition = enable_learning && ...
    t_now >= learn_start_time && ...
    abs(w_ref) >= min_speed_abs && ...
    abs(w_meas - w_ref) <= speed_err_threshold;
learn_active = 0.0;

if learn_condition
    table(idx) = (1 - learning_rate) * table(idx) + learning_rate * learn_signal;
    learn_active = 1.0;
end

iq_ff = table(idx_adv);
if iq_ff > output_limit
    iq_ff = output_limit;
elseif iq_ff < -output_limit
    iq_ff = -output_limit;
end

if ~enable_ff || t_now < ff_enable_time
    iq_ff = 0;
end

iq_ref_cmd = iq_ref_base + iq_ff;
end
