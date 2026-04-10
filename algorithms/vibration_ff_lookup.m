function [iq_ref_cmd, iq_ff, learn_active] = vibration_ff_lookup(theta_meas, iq_ref_base, t_now, ff_table, params)
% Fixed offline vibration feedforward lookup.
%
% ff_table is expected to be a fixed-length vector (MAX_POINTS x 1), while
% points selects the active prefix used for lookup.

MAX_POINTS = 360;
points = min(MAX_POINTS, max(8, round(params(1))));
phase_advance_rad = params(2) * pi / 180;
output_limit = max(0, params(3));
enable_ff = params(4) > 0.5;
ff_enable_time = max(0, params(5));

if size(ff_table, 1) < MAX_POINTS
    table = zeros(MAX_POINTS, 1);
    table(1:size(ff_table, 1)) = ff_table(:);
else
    table = ff_table(1:MAX_POINTS);
end

theta_wrap = mod(theta_meas, 2 * pi);
if theta_wrap < 0
    theta_wrap = theta_wrap + 2 * pi;
end

theta_adv = mod(theta_wrap + phase_advance_rad, 2 * pi);
idx_adv = 1 + floor(theta_adv / (2 * pi) * points);
if idx_adv > points
    idx_adv = points;
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
learn_active = 0.0;
end
