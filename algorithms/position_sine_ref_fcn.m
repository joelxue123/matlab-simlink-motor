function pos_ref = position_sine_ref_fcn(t_now, params)
% Generate a sinusoidal position reference.
% params = [amp_rad, freq_hz, start_time, offset_rad]

amp_rad = params(1);
freq_hz = params(2);
start_time = params(3);
offset_rad = params(4);

if t_now < start_time
    pos_ref = offset_rad;
    return;
end

tau = t_now - start_time;
pos_ref = offset_rad + amp_rad * sin(2 * pi * freq_hz * tau);
end