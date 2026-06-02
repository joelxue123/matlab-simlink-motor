function report = identify_positionloop_closedloop_tf_swept_sine(varargin)
% Identify a low-order closed-loop transfer function from swept-sine data.
%
% Usage:
%   report = identify_positionloop_closedloop_tf_swept_sine();
%   report = identify_positionloop_closedloop_tf_swept_sine('freq_hz', logspace(0, 1.7, 10));

options = local_parse_inputs(varargin{:});

freq_hz = options.freq_hz(:);
point_count = numel(freq_hz);
H = zeros(point_count, 1);
mag_db = zeros(point_count, 1);
phase_deg = zeros(point_count, 1);

for idx = 1:point_count
    f_hz = freq_hz(idx);
    cycles_total = options.settle_cycles + options.measure_cycles;
    stop_time = options.start_time + cycles_total / f_hz + options.tail_time;
    overrides = struct( ...
        'plot_results', false, ...
        'pos_ref_mode', 'sine', ...
        'pos_use_planner', false, ...
        'pos_sine_amplitude_rad', options.amplitude_rad, ...
        'pos_sine_freq_hz', f_hz, ...
        'pos_sine_start_time', options.start_time, ...
        'pos_sine_offset_rad', options.offset_rad, ...
        'stop_time', stop_time);

    fprintf('Running swept-sine point %d/%d at %.3f Hz\n', idx, point_count, f_hz);
    result = run_position_pidreg3_test(overrides);
    H(idx) = local_extract_frf_point(result.time, result.pos_ref, result.pos, f_hz, options);
    mag_db(idx) = 20 * log10(abs(H(idx)) + eps);
    phase_deg(idx) = angle(H(idx)) * 180 / pi;
end

w_rad_s = 2 * pi * freq_hz;
fits = local_fit_models(H, w_rad_s, options.max_order);
[~, best_idx] = min([fits.bic]);
best_fit = fits(best_idx);

report = struct();
report.freq_hz = freq_hz;
report.w_rad_s = w_rad_s;
report.H = H;
report.mag_db = mag_db;
report.phase_deg = phase_deg;
report.fits = fits;
report.best_fit = best_fit;

fprintf('\n=== Position-loop swept-sine identification ===\n');
fprintf('Best denominator order by BIC: %d\n', best_fit.order);
fprintf('Numerator coefficients: ');
disp(best_fit.num);
fprintf('Denominator coefficients: ');
disp(best_fit.den);
fprintf('Fit RMS error: %.6e\n', best_fit.rms_error);
fprintf('BIC: %.6f\n', best_fit.bic);

local_plot_identification(report);
assignin('base', 'positionloop_tf_ident_report', report);
end

function options = local_parse_inputs(varargin)
options = struct();
options.freq_hz = logspace(log10(1.0), log10(40.0), 10);
options.amplitude_rad = deg2rad(1.0);
options.start_time = 0.05;
options.offset_rad = 0.0;
options.settle_cycles = 6;
options.measure_cycles = 8;
options.tail_time = 0.1;
options.max_order = 4;

if rem(numel(varargin), 2) ~= 0
    error('Name-value inputs must come in pairs.');
end
for idx = 1:2:numel(varargin)
    name = lower(varargin{idx});
    value = varargin{idx + 1};
    switch name
        case 'freq_hz'
            options.freq_hz = value;
        case 'amplitude_rad'
            options.amplitude_rad = value;
        case 'start_time'
            options.start_time = value;
        case 'offset_rad'
            options.offset_rad = value;
        case 'settle_cycles'
            options.settle_cycles = value;
        case 'measure_cycles'
            options.measure_cycles = value;
        case 'tail_time'
            options.tail_time = value;
        case 'max_order'
            options.max_order = value;
        otherwise
            error('Unknown option: %s', varargin{idx});
    end
end
end

function H = local_extract_frf_point(t, u, y, freq_hz, options)
omega = 2 * pi * freq_hz;
measure_start = options.start_time + options.settle_cycles / freq_hz;
mask = t >= measure_start;
t_fit = t(mask);
u_fit = u(mask);
y_fit = y(mask);

Xu = [sin(omega * (t_fit - options.start_time)), cos(omega * (t_fit - options.start_time)), ones(size(t_fit))];
coef_u = Xu \ u_fit;
coef_y = Xu \ y_fit;

amp_u = hypot(coef_u(1), coef_u(2));
amp_y = hypot(coef_y(1), coef_y(2));
phase_u = atan2(coef_u(2), coef_u(1));
phase_y = atan2(coef_y(2), coef_y(1));

H = (amp_y / max(amp_u, eps)) * exp(1i * (phase_y - phase_u));
end

function fits = local_fit_models(H, w_rad_s, max_order)
fits = repmat(struct('order', 0, 'num', [], 'den', [], 'sys', [], ...
    'H_fit', [], 'rms_error', NaN, 'aic', NaN, 'bic', NaN), 1, max_order);

for order = 1:max_order
    num_order = max(0, order - 1);
    [num, den] = invfreqs(H, w_rad_s, num_order, order);
    sys = tf(num, den);
    H_fit = squeeze(freqresp(sys, w_rad_s));
    err = H(:) - H_fit(:);
    mse = mean(abs(err).^2);
    param_count = numel(num) + numel(den) - 1;

    fits(order).order = order;
    fits(order).num = num;
    fits(order).den = den;
    fits(order).sys = sys;
    fits(order).H_fit = H_fit(:);
    fits(order).rms_error = sqrt(mse);
    fits(order).aic = numel(H) * log(max(mse, eps)) + 2 * param_count;
    fits(order).bic = numel(H) * log(max(mse, eps)) + param_count * log(numel(H));
end
end

function local_plot_identification(report)
figure('Name', 'Position-loop Closed-loop TF Identification', 'Color', 'w');

subplot(2, 1, 1);
semilogx(report.freq_hz, report.mag_db, 'ko', 'LineWidth', 1.1); hold on;
for idx = 1:numel(report.fits)
    semilogx(report.freq_hz, 20 * log10(abs(report.fits(idx).H_fit) + eps), 'LineWidth', 1.0);
end
grid on;
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
title('Measured and fitted closed-loop magnitude');

subplot(2, 1, 2);
semilogx(report.freq_hz, report.phase_deg, 'ko', 'LineWidth', 1.1); hold on;
for idx = 1:numel(report.fits)
    semilogx(report.freq_hz, unwrap(angle(report.fits(idx).H_fit)) * 180 / pi, 'LineWidth', 1.0);
end
grid on;
xlabel('Frequency (Hz)');
ylabel('Phase (deg)');
title('Measured and fitted closed-loop phase');
end