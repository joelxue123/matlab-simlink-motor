function report = estimate_positionloop_bandwidth_chirp(varargin)
% Estimate the closed-loop position bandwidth using a chirp reference.
%
% Usage:
%   report = estimate_positionloop_bandwidth_chirp();
%   report = estimate_positionloop_bandwidth_chirp('amplitude_rad', deg2rad(1));

options = local_parse_inputs(varargin{:});

motor_control_params;
chirp_stop_time = options.start_time + options.duration + 0.5;
overrides = struct( ...
    'plot_results', false, ...
    'pos_ref_mode', 'chirp', ...
    'pos_use_planner', false, ...
    'pos_chirp_amplitude_rad', options.amplitude_rad, ...
    'pos_chirp_f0_hz', options.f0_hz, ...
    'pos_chirp_f1_hz', options.f1_hz, ...
    'pos_chirp_start_time', options.start_time, ...
    'pos_chirp_duration', options.duration, ...
    'pos_chirp_offset_rad', options.offset_rad, ...
    'stop_time', chirp_stop_time);

result = run_position_pidreg3_test(overrides);

t_raw = result.time(:);
u_raw = result.pos_ref(:);
y_raw = result.pos(:);
mask = t_raw >= options.start_time & t_raw <= (options.start_time + options.duration);
t_raw = t_raw(mask);
u_raw = u_raw(mask);
y_raw = y_raw(mask);

motor_control_params;
t = (t_raw(1):simcfg.Ts_pos:t_raw(end)).';
u = interp1(t_raw, u_raw, t, 'linear', 'extrap');
y = interp1(t_raw, y_raw, t, 'linear', 'extrap');

if numel(t) < 128
    error('Chirp window too short for bandwidth estimation.');
end

fs = 1 / simcfg.Ts_pos;
u = u - mean(u);
y = y - mean(y);

[freq_hz, mag_db, phase_deg] = local_estimate_frf(u, y, fs);
valid = freq_hz >= options.f0_hz & freq_hz <= options.f1_hz;
freq_hz = freq_hz(valid);
mag_db = mag_db(valid);
phase_deg = phase_deg(valid);

low_freq_mask = freq_hz <= min(options.f0_hz * 2.5, max(freq_hz));
if ~any(low_freq_mask)
    low_freq_mask = 1:min(5, numel(freq_hz));
end
ref_gain_db = mean(mag_db(low_freq_mask));
idx_bw = find(mag_db <= (ref_gain_db - 3), 1, 'first');
if isempty(idx_bw)
    bw_hz = NaN;
    bw_rad_s = NaN;
else
    bw_hz = freq_hz(idx_bw);
    bw_rad_s = 2 * pi * bw_hz;
end

report = struct();
report.design_pos_bw_hz = control.pos_bandwidth_hz;
report.design_pos_bw_rad_s = control.pos_bandwidth_rad_s;
report.estimated_bw_hz = bw_hz;
report.estimated_bw_rad_s = bw_rad_s;
report.freq_hz = freq_hz;
report.mag_db = mag_db;
report.phase_deg = phase_deg;
report.ref_gain_db = ref_gain_db;
report.chirp = options;
report.result = result;

fprintf('\n=== Position-loop chirp bandwidth estimate ===\n');
fprintf('Chirp amplitude : %.6f rad\n', options.amplitude_rad);
fprintf('Chirp range     : %.3f Hz -> %.3f Hz\n', options.f0_hz, options.f1_hz);
fprintf('Design BW       : %.3f Hz (%.3f rad/s)\n', ...
    report.design_pos_bw_hz, report.design_pos_bw_rad_s);
if isnan(report.estimated_bw_hz)
    fprintf('Estimated BW    : not found within chirp range\n');
else
    fprintf('Estimated BW    : %.3f Hz (%.3f rad/s)\n', ...
        report.estimated_bw_hz, report.estimated_bw_rad_s);
end

local_plot_report(report);
assignin('base', 'positionloop_chirp_bw_report', report);
end

function options = local_parse_inputs(varargin)
options = struct();
options.amplitude_rad = deg2rad(2.0);
options.f0_hz = 0.2;
options.f1_hz = 25.0;
options.start_time = 0.05;
options.duration = 8.0;
options.offset_rad = 0.0;

if rem(numel(varargin), 2) ~= 0
    error('Name-value inputs must come in pairs.');
end
for idx = 1:2:numel(varargin)
    name = lower(varargin{idx});
    value = varargin{idx + 1};
    switch name
        case 'amplitude_rad'
            options.amplitude_rad = value;
        case 'f0_hz'
            options.f0_hz = value;
        case 'f1_hz'
            options.f1_hz = value;
        case 'start_time'
            options.start_time = value;
        case 'duration'
            options.duration = value;
        case 'offset_rad'
            options.offset_rad = value;
        otherwise
            error('Unknown option: %s', varargin{idx});
    end
end
end

function [freq_hz, mag_db, phase_deg] = local_estimate_frf(u, y, fs)
nfft = 2 ^ floor(log2(numel(u)));
nfft = max(nfft, 256);
nfft = min(nfft, 2 ^ 16);
nfft = min(nfft, numel(u));
window = hann(nfft);
noverlap = floor(0.75 * nfft);

if exist('tfestimate', 'file') == 2
    [H, f] = tfestimate(u, y, window, noverlap, nfft, fs);
else
    [Pyu, f] = cpsd(y, u, window, noverlap, nfft, fs);
    [Puu, ~] = cpsd(u, u, window, noverlap, nfft, fs);
    H = Pyu ./ Puu;
end

freq_hz = f(:);
mag_db = 20 * log10(abs(H(:)) + eps);
phase_deg = unwrap(angle(H(:))) * 180 / pi;
end

function local_plot_report(report)
figure('Name', 'Position-loop Chirp Bandwidth', 'Color', 'w');

subplot(2, 1, 1);
semilogx(report.freq_hz, report.mag_db, 'LineWidth', 1.2); hold on;
yline(report.ref_gain_db - 3, '--', 'LineWidth', 1.0);
if ~isnan(report.estimated_bw_hz)
    xline(report.estimated_bw_hz, '--', 'LineWidth', 1.0);
end
grid on;
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
title('Closed-loop position response');

subplot(2, 1, 2);
semilogx(report.freq_hz, report.phase_deg, 'LineWidth', 1.2);
grid on;
xlabel('Frequency (Hz)');
ylabel('Phase (deg)');
title('Position-loop phase response');
end