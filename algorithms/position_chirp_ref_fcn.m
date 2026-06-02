function pos_ref = position_chirp_ref_fcn(t_now, params)
% Generate a position-reference chirp signal.
% params = [amp_rad, f0_hz, f1_hz, t_start, duration, offset_rad]

amp_rad = params(1);
f0_hz = params(2);
f1_hz = params(3);
t_start = params(4);
duration = params(5);
offset_rad = params(6);

if t_now < t_start
    pos_ref = offset_rad;
    return;
end

tau = t_now - t_start;
if tau > duration
    pos_ref = offset_rad;
    return;
end

chirp_rate_hz_s = (f1_hz - f0_hz) / max(duration, eps);
phase_rad = 2 * pi * (f0_hz * tau + 0.5 * chirp_rate_hz_s * tau^2);
pos_ref = offset_rad + amp_rad * sin(phase_rad);
end