function result = plot_switching_deadtime_comp_before_after(varargin)
%PLOT_SWITCHING_DEADTIME_COMP_BEFORE_AFTER Compare phase currents before/after compensation.
%
% The physical plant is the switching MOSFET/diode bridge plus SPS PMSM.
% DeadtimeCompensationStep is still reused from motor_control_lib; this script
% only runs two harness cases and exports a PNG comparison.

script_dir = fileparts(mfilename('fullpath'));
results_dir = fullfile(script_dir, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

opts.cyclesToShow = 3;
opts.electricalFreqHz = 80;
opts.simStopTime = [];
opts.outputPng = fullfile(results_dir, 'switching_deadtime_comp_before_after.png');
opts.modulationRatio = 0.9;
opts.deadtime = 500e-9;
opts.vdc = 12;
opts.rOhm = 4.0;
opts.lH = 100e-6;
opts = local_parse_inputs(opts, varargin{:});
opts.electricalPeriod_s = 1 / opts.electricalFreqHz;
opts.computedStopTime_s = opts.cyclesToShow * opts.electricalPeriod_s;
if isempty(opts.simStopTime)
    opts.simStopTime = opts.computedStopTime_s;
end

fprintf('Electrical frequency = %.6g Hz\n', opts.electricalFreqHz);
fprintf('Electrical period = %.6g ms\n', opts.electricalPeriod_s * 1e3);
fprintf('Cycles requested = %.6g\n', opts.cyclesToShow);
fprintf('Simulation stop time = %.6g ms\n', opts.simStopTime * 1e3);

fprintf('Running switching case without deadtime compensation...\n');
before = run_switching_deadtime_motor_smoke_test( ...
    'deadtimeCompEnable', 0, ...
    'simStopTime', opts.simStopTime, ...
    'modulationRatio', opts.modulationRatio, ...
    'electricalFreqHz', opts.electricalFreqHz, ...
    'deadtime', opts.deadtime, ...
    'vdc', opts.vdc, ...
    'rOhm', opts.rOhm, ...
    'lH', opts.lH);

fprintf('Running switching case with deadtime compensation...\n');
after = run_switching_deadtime_motor_smoke_test( ...
    'deadtimeCompEnable', 1, ...
    'simStopTime', opts.simStopTime, ...
    'modulationRatio', opts.modulationRatio, ...
    'electricalFreqHz', opts.electricalFreqHz, ...
    'deadtime', opts.deadtime, ...
    'vdc', opts.vdc, ...
    'rOhm', opts.rOhm, ...
    'lH', opts.lH);

delta = local_compare_cases(before, after);

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [80 80 1600 1100]);
tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

all_currents = [before.ia(:); before.ib(:); before.ic(:); ...
    after.ia(:); after.ib(:); after.ic(:)];
y_pad = 0.08 * max(max(all_currents) - min(all_currents), eps);
y_lim = [min(all_currents) - y_pad, max(all_currents) + y_pad];

local_plot_case(nexttile, before, 'Before compensation');
ylim(y_lim);
local_plot_case(nexttile, after, 'After compensation');
ylim(y_lim);
local_plot_delta(nexttile, delta);

exportgraphics(fig, opts.outputPng, 'Resolution', 180);
close(fig);

report_file = fullfile(results_dir, 'switching_deadtime_comp_before_after_report.txt');
fid = fopen(report_file, 'w');
cleanup_file = onCleanup(@() fclose(fid));
fprintf(fid, 'electrical_freq_hz = %.9g\n', opts.electricalFreqHz);
fprintf(fid, 'electrical_period_s = %.9g\n', opts.electricalPeriod_s);
fprintf(fid, 'cycles_to_show = %.9g\n', opts.cyclesToShow);
fprintf(fid, 'computed_stop_time_s = %.9g\n', opts.computedStopTime_s);
fprintf(fid, 'actual_stop_time_s = %.9g\n', opts.simStopTime);
local_write_case(fid, 'before', before);
local_write_case(fid, 'after', after);
fprintf(fid, 'delta_ia_rms_A = %.9g\n', delta.ia_rms_A);
fprintf(fid, 'delta_ib_rms_A = %.9g\n', delta.ib_rms_A);
fprintf(fid, 'delta_ic_rms_A = %.9g\n', delta.ic_rms_A);
fprintf(fid, 'delta_max_abs_A = %.9g\n', delta.max_abs_A);
fprintf(fid, 'output_png = %s\n', opts.outputPng);

result.before = before;
result.after = after;
result.outputPng = opts.outputPng;
result.reportFile = report_file;

fprintf('Saved comparison PNG:\n  %s\n', opts.outputPng);
fprintf('Saved comparison report:\n  %s\n', report_file);
end

function local_plot_case(ax, data, title_text)
plot(ax, data.time * 1e3, data.ia, 'LineWidth', 0.8);
hold(ax, 'on');
plot(ax, data.time * 1e3, data.ib, 'LineWidth', 0.8);
plot(ax, data.time * 1e3, data.ic, 'LineWidth', 0.8);
grid(ax, 'on');
xlabel(ax, 'time (ms)');
ylabel(ax, 'phase current (A)');
title(ax, sprintf('%s: pk-pk = [%.4f %.4f %.4f] A, max |i| = %.4f A', ...
    title_text, data.ia_pkpk_A, data.ib_pkpk_A, data.ic_pkpk_A, ...
    data.max_abs_current_A));
legend(ax, {'ia', 'ib', 'ic'}, 'Location', 'eastoutside');
end

function local_plot_delta(ax, delta)
plot(ax, delta.time * 1e3, delta.ia, 'LineWidth', 0.8);
hold(ax, 'on');
plot(ax, delta.time * 1e3, delta.ib, 'LineWidth', 0.8);
plot(ax, delta.time * 1e3, delta.ic, 'LineWidth', 0.8);
grid(ax, 'on');
xlabel(ax, 'time (ms)');
ylabel(ax, 'after - before (A)');
title(ax, sprintf('Compensation effect: RMS = [%.5f %.5f %.5f] A, max |delta| = %.5f A', ...
    delta.ia_rms_A, delta.ib_rms_A, delta.ic_rms_A, delta.max_abs_A));
legend(ax, {'delta ia', 'delta ib', 'delta ic'}, 'Location', 'eastoutside');
end

function delta = local_compare_cases(before, after)
delta.time = before.time(:);
delta.ia = interp1(after.time(:), after.ia(:), delta.time, 'linear', 'extrap') - before.ia(:);
delta.ib = interp1(after.time(:), after.ib(:), delta.time, 'linear', 'extrap') - before.ib(:);
delta.ic = interp1(after.time(:), after.ic(:), delta.time, 'linear', 'extrap') - before.ic(:);
delta.ia_rms_A = rms(delta.ia);
delta.ib_rms_A = rms(delta.ib);
delta.ic_rms_A = rms(delta.ic);
delta.max_abs_A = max(abs([delta.ia(:); delta.ib(:); delta.ic(:)]));
end

function local_write_case(fid, name, data)
fprintf(fid, '%s_ia_pkpk_A = %.9g\n', name, data.ia_pkpk_A);
fprintf(fid, '%s_ib_pkpk_A = %.9g\n', name, data.ib_pkpk_A);
fprintf(fid, '%s_ic_pkpk_A = %.9g\n', name, data.ic_pkpk_A);
fprintf(fid, '%s_max_abs_current_A = %.9g\n', name, data.max_abs_current_A);
fprintf(fid, '%s_sum_current_rms_A = %.9g\n', name, data.sum_current_rms_A);
end

function opts = local_parse_inputs(opts, varargin)
if mod(numel(varargin), 2) ~= 0
    error('Arguments must be name/value pairs.');
end

for idx = 1:2:numel(varargin)
    name = lower(varargin{idx});
    value = varargin{idx + 1};
    switch name
        case 'simstoptime'
            opts.simStopTime = value;
        case 'cyclestoshow'
            opts.cyclesToShow = value;
        case 'electricalfreqhz'
            opts.electricalFreqHz = value;
        case 'outputpng'
            opts.outputPng = value;
        case 'modulationratio'
            opts.modulationRatio = value;
        case 'deadtime'
            opts.deadtime = value;
        case 'vdc'
            opts.vdc = value;
        case 'rohm'
            opts.rOhm = value;
        case 'lh'
            opts.lH = value;
        otherwise
            error('Unknown parameter: %s', varargin{idx});
    end
end

if opts.electricalFreqHz <= 0
    error('electricalFreqHz must be positive.');
end

if opts.cyclesToShow <= 0
    error('cyclesToShow must be positive.');
end
end
