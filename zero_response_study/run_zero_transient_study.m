function result = run_zero_transient_study(varargin)
%RUN_ZERO_TRANSIENT_STUDY Study how transfer-function zeros change transients.
% The study keeps the dominant second-order poles fixed and varies only the
% zero location so the effect on rise time, overshoot, and inverse response
% can be compared directly under a unit-step input.

cfg = local_default_config();
cfg = local_parse_inputs(cfg, varargin{:});

cases = local_build_cases(cfg);
t = linspace(0.0, cfg.t_end_s, cfg.num_samples).';

result.cfg = cfg;
result.time_s = t;
result.cases = repmat(struct(), numel(cases), 1);

fprintf('\nZero transient-response study\n');
fprintf('Base poles: wn = %.3f rad/s, zeta = %.3f, final value normalized to 1\n', ...
    cfg.wn_rad_s, cfg.zeta);
fprintf('%-22s %-16s %-10s %-10s %-10s %-10s\n', ...
    'Case', 'Zero', 'Rise(s)', 'Settling', 'Overshoot', 'Min y');

figure('Name', 'Zero impact on transient response', 'Color', 'w');
tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
hold on;
grid on;
title('Unit-step response');
xlabel('Time (s)');
ylabel('y(t)');

nexttile;
hold on;
grid on;
title('Early transient zoom');
xlabel('Time (s)');
ylabel('y(t)');

nexttile;
hold on;
grid on;
title('Pole-zero map');
xlabel('Real axis');
ylabel('Imag axis');

nexttile;
hold on;
grid on;
title('Initial  slope  dy/dt');
xlabel('Time (s)');
ylabel('dy/dt');

legend_entries = cell(numel(cases), 1);
zoom_limit_s = min(cfg.t_end_s, cfg.zoom_window_s);
pole_handle = gobjects(0);
zero_handle = gobjects(0);

for idx = 1:numel(cases)
    sys = cases(idx).sys;
    y = step(sys, t);
    info = stepinfo(y, t, 1.0, 'SettlingTimeThreshold', cfg.settling_threshold);
    dydt = gradient(y, t);
    zero_values = zero(sys);
    pole_values = pole(sys);

    result.cases(idx).name = cases(idx).name;
    result.cases(idx).sys = sys;
    result.cases(idx).zero = zero_values;
    result.cases(idx).pole = pole_values;
    result.cases(idx).response = y;
    result.cases(idx).stepinfo = info;
    result.cases(idx).min_response = min(y);
    result.cases(idx).max_response = max(y);
    result.cases(idx).initial_slope = dydt(1);

    legend_entries{idx} = cases(idx).name;
    zero_label = local_zero_label(zero_values);
    fprintf('%-22s %-16s %-10.4f %-10.4f %-10.2f %-10.4f\n', ...
        cases(idx).name, zero_label, info.RiseTime, info.SettlingTime, info.Overshoot, min(y));

    nexttile(1);
    plot(t, y, 'LineWidth', 1.5);

    nexttile(2);
    zoom_mask = t <= zoom_limit_s;
    plot(t(zoom_mask), y(zoom_mask), 'LineWidth', 1.5);

    nexttile(3);
    pole_plot = plot(real(pole_values), imag(pole_values), 'x', 'MarkerSize', 10, 'LineWidth', 1.5);
    if isempty(pole_handle)
        pole_handle = pole_plot(1);
    end
    if ~isempty(zero_values)
        zero_plot = plot(real(zero_values), imag(zero_values), 'o', 'MarkerSize', 8, 'LineWidth', 1.5);
        if isempty(zero_handle)
            zero_handle = zero_plot(1);
        end
    end

    nexttile(4);
    plot(t(zoom_mask), dydt(zoom_mask), 'LineWidth', 1.5);
end

nexttile(1);
yline(1.0, '--k', 'Final value');
legend(legend_entries, 'Location', 'best');

nexttile(2);
yline(1.0, '--k', 'Final value');

nexttile(3);
xline(0.0, ':k');
yline(0.0, ':k');
if isempty(zero_handle)
    legend(pole_handle, {'Poles'}, 'Location', 'best');
else
    legend([pole_handle zero_handle], {'Poles', 'Zeros'}, 'Location', 'best');
end

nexttile(4);
xline(0.0, ':k');

fprintf('\nInterpretation\n');
fprintf('1. Left-half-plane zero closer to the imaginary axis usually speeds up the initial response.\n');
fprintf('2. That speed-up often comes with more overshoot and weaker damping in the visible transient.\n');
fprintf('3. Right-half-plane zero creates inverse response: output first moves in the wrong direction.\n');
fprintf('4. Keeping DC gain fixed isolates the zero effect instead of conflating it with static gain changes.\n');

end

function cfg = local_default_config()
cfg.wn_rad_s = 40.0;
cfg.zeta = 0.35;
cfg.t_end_s = 0.45;
cfg.num_samples = 3000;
cfg.zoom_window_s = 0.08;
cfg.settling_threshold = 0.02;
cfg.lhp_zero_rad_s = [400.0, 80.0, 25.0];
cfg.include_no_zero = true;
cfg.include_rhp_zero = true;
cfg.rhp_zero_rad_s = 25.0;
end

function cfg = local_parse_inputs(cfg, varargin)
if mod(numel(varargin), 2) ~= 0
    error('Arguments must be name/value pairs.');
end

for idx = 1:2:numel(varargin)
    name = varargin{idx};
    value = varargin{idx + 1};
    switch lower(name)
        case 'wnrads'
            cfg.wn_rad_s = value;
        case 'zeta'
            cfg.zeta = value;
        case 'tends'
            cfg.t_end_s = value;
        case 'numsamples'
            cfg.num_samples = value;
        case 'zoomwindows'
            cfg.zoom_window_s = value;
        case 'settlingthreshold'
            cfg.settling_threshold = value;
        case 'lhpzerorads'
            cfg.lhp_zero_rad_s = value;
        case 'includenozero'
            cfg.include_no_zero = logical(value);
        case 'includerhpzero'
            cfg.include_rhp_zero = logical(value);
        case 'rhpzerorads'
            cfg.rhp_zero_rad_s = value;
        otherwise
            error('Unknown parameter: %s', name);
    end
end
end

function cases = local_build_cases(cfg)
s = tf('s');
den = s^2 + 2 * cfg.zeta * cfg.wn_rad_s * s + cfg.wn_rad_s^2;

case_idx = 0;
cases = repmat(struct('name', '', 'sys', []), 0, 1);

if cfg.include_no_zero
    case_idx = case_idx + 1;
    cases(case_idx).name = 'No zero';
    cases(case_idx).sys = (cfg.wn_rad_s^2) / den;
end

for idx = 1:numel(cfg.lhp_zero_rad_s)
    wz = cfg.lhp_zero_rad_s(idx);
    case_idx = case_idx + 1;
    cases(case_idx).name = sprintf('LHP zero @ -%.1f', wz);
    cases(case_idx).sys = (cfg.wn_rad_s^2) * (1 + s / wz) / den;
end

if cfg.include_rhp_zero
    wz = cfg.rhp_zero_rad_s;
    case_idx = case_idx + 1;
    cases(case_idx).name = sprintf('RHP zero @ +%.1f', wz);
    cases(case_idx).sys = (cfg.wn_rad_s^2) * (1 - s / wz) / den;
end
end

function label = local_zero_label(zero_values)
if isempty(zero_values)
    label = 'none';
elseif numel(zero_values) == 1
    label = sprintf('%.3g', zero_values);
else
    label = sprintf('%d zeros', numel(zero_values));
end
end