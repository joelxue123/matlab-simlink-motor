function result = run_same_poles_zero_example(varargin)
%RUN_SAME_POLES_ZERO_EXAMPLE Show how equal poles can still yield different transients.
% This is a compact example for the common root-locus question: if the
% closed-loop poles are already placed well, can zeros still increase
% overshoot or create inverse-looking behavior? The answer is yes.

cfg = local_default_config();
cfg = local_parse_inputs(cfg, varargin{:});

s = tf('s');
den = s^2 + 2 * cfg.zeta * cfg.wn_rad_s * s + cfg.wn_rad_s^2;
t = linspace(0.0, cfg.t_end_s, cfg.num_samples).';

cases = [ ...
    struct('name', 'No zero', 'sys', (cfg.wn_rad_s^2) / den); ...
    struct('name', sprintf('LHP zero @ -%.1f', cfg.left_zero_far_rad_s), ...
        'sys', (cfg.wn_rad_s^2) * (1 + s / cfg.left_zero_far_rad_s) / den); ...
    struct('name', sprintf('LHP zero @ -%.1f', cfg.left_zero_near_rad_s), ...
        'sys', (cfg.wn_rad_s^2) * (1 + s / cfg.left_zero_near_rad_s) / den); ...
    struct('name', sprintf('RHP zero @ +%.1f', cfg.right_zero_rad_s), ...
        'sys', (cfg.wn_rad_s^2) * (1 - s / cfg.right_zero_rad_s) / den) ...
    ];

result.cfg = cfg;
result.time_s = t;
result.common_poles = pole(cases(1).sys);
result.cases = repmat(struct(), numel(cases), 1);

fprintf('\nSame-poles, different-zeros example\n');
fprintf('All cases share poles at: %s\n', local_complex_pair_string(result.common_poles));
fprintf('%-20s %-12s %-10s %-10s %-10s %-10s\n', ...
    'Case', 'Zero', 'Rise(s)', 'Settling', 'Overshoot', 'Min y');

figure('Name', 'Same poles, different zeros', 'Color', 'w');
tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
hold on;
grid on;
title('Step responses with identical poles');
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
axis off;

zoom_mask = t <= cfg.zoom_window_s;
summary_lines = strings(numel(cases) + 3, 1);
summary_lines(1) = "Equal closed-loop poles, different closed-loop zeros:";
summary_lines(2) = sprintf('p = %.3f %+.3fj, %.3f %+.3fj', ...
    real(result.common_poles(1)), imag(result.common_poles(1)), ...
    real(result.common_poles(2)), imag(result.common_poles(2)));
summary_lines(3) = "";

for idx = 1:numel(cases)
    sys = cases(idx).sys;
    y = step(sys, t);
    info = stepinfo(y, t, 1.0, 'SettlingTimeThreshold', cfg.settling_threshold);
    zero_values = zero(sys);

    result.cases(idx).name = cases(idx).name;
    result.cases(idx).sys = sys;
    result.cases(idx).pole = pole(sys);
    result.cases(idx).zero = zero_values;
    result.cases(idx).response = y;
    result.cases(idx).stepinfo = info;
    result.cases(idx).min_response = min(y);

    fprintf('%-20s %-12s %-10.4f %-10.4f %-10.2f %-10.4f\n', ...
        cases(idx).name, local_zero_label(zero_values), info.RiseTime, ...
        info.SettlingTime, info.Overshoot, min(y));

    nexttile(1);
    plot(t, y, 'LineWidth', 1.5, 'DisplayName', cases(idx).name);

    nexttile(2);
    plot(t(zoom_mask), y(zoom_mask), 'LineWidth', 1.5, 'DisplayName', cases(idx).name);

    nexttile(3);
    plot(real(result.common_poles), imag(result.common_poles), 'xk', 'MarkerSize', 10, 'LineWidth', 1.5);
    if ~isempty(zero_values)
        plot(real(zero_values), imag(zero_values), 'o', 'MarkerSize', 8, 'LineWidth', 1.5, ...
            'DisplayName', cases(idx).name);
    end

    summary_lines(idx + 3) = sprintf('%s: OS = %.1f%%%%, min y = %.3f', ...
        cases(idx).name, info.Overshoot, min(y));
end

nexttile(1);
yline(1.0, '--k', 'Final value');
legend('Location', 'best');

nexttile(2);
yline(1.0, '--k', 'Final value');

nexttile(3);
xline(0.0, ':k');
yline(0.0, ':k');
legend('Location', 'best');

nexttile(4);
text(0.0, 1.0, summary_lines, 'VerticalAlignment', 'top', 'FontName', 'Courier');

fprintf('\nInterpretation\n');
fprintf('1. The common pole pair fixes the dominant damping trend and decay rate.\n');
fprintf('2. Moving an LHP zero toward the imaginary axis raises initial slope and usually overshoot.\n');
fprintf('3. An RHP zero can produce inverse-looking motion even though the poles stay stable.\n');
fprintf('4. Root-locus pole placement must therefore be checked against zero-induced waveform distortion.\n');

end

function cfg = local_default_config()
cfg.wn_rad_s = 24.0;
cfg.zeta = 0.50;
cfg.t_end_s = 0.80;
cfg.zoom_window_s = 0.18;
cfg.num_samples = 3000;
cfg.settling_threshold = 0.02;
cfg.left_zero_far_rad_s = 120.0;
cfg.left_zero_near_rad_s = 18.0;
cfg.right_zero_rad_s = 18.0;
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
        case 'zoomwindows'
            cfg.zoom_window_s = value;
        case 'numsamples'
            cfg.num_samples = value;
        case 'settlingthreshold'
            cfg.settling_threshold = value;
        case 'leftzerofarrads'
            cfg.left_zero_far_rad_s = value;
        case 'leftzeronearrads'
            cfg.left_zero_near_rad_s = value;
        case 'rightzerorads'
            cfg.right_zero_rad_s = value;
        otherwise
            error('Unknown parameter: %s', name);
    end
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

function label = local_complex_pair_string(values)
parts = strings(numel(values), 1);
for idx = 1:numel(values)
    parts(idx) = sprintf('%.3f %+.3fj', real(values(idx)), imag(values(idx)));
end
label = strjoin(cellstr(parts), ', ');
end