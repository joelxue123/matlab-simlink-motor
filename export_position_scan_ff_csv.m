function result = export_position_scan_ff_csv(varargin)
% Export feedforward current CSV from position-scan IQ data without validation.
%
% Usage:
%   result = export_position_scan_ff_csv;
%   result = export_position_scan_ff_csv('position_scan_iq_result.mat');
%   result = export_position_scan_ff_csv(scan_result);
%   result = export_position_scan_ff_csv(forward_result, backward_result);
%   result = export_position_scan_ff_csv(..., cfg);
%
% Optional cfg fields:
%   csv_file, mat_file, smooth_window, use_mean_removal

[forward_input, backward_input, cfg] = local_parse_inputs(varargin{:});
forward = local_load_scan_result(forward_input, 'position_scan_iq_result');
backward = local_load_optional_scan_result(backward_input);
cfg = local_fill_defaults(cfg, forward);
local_print_disturbance_check(forward);

[scan, ff_table] = local_build_ff_table(forward, backward, cfg);
export_table = local_build_export_table(scan, ff_table, cfg);

writetable(export_table, cfg.csv_file);
save(cfg.mat_file, 'ff_table', 'scan', 'cfg');

result = struct();
result.config = cfg;
result.scan = scan;
result.ff_table = ff_table;
result.csv_file = cfg.csv_file;
result.mat_file = cfg.mat_file;
result.rows = height(export_table);

assignin('base', 'position_scan_ff_export', result);
assignin('base', 'position_scan_ff_table', ff_table);

fprintf('\n=== Position-scan FF CSV export ===\n');
fprintf('Mode        : %s\n', scan.mode);
fprintf('Points      : %d\n', scan.points);
fprintf('CSV file    : %s\n', cfg.csv_file);
fprintf('MAT file    : %s\n', cfg.mat_file);
fprintf('IQ FF range : [%.6f, %.6f] A\n', min(scan.iq_ff), max(scan.iq_ff));
end

function local_print_disturbance_check(scan_result)
if ~isfield(scan_result, 'disturbance')
    warning('Scan result has no disturbance metadata. Make sure it was generated from the intended load torque.');
    return;
end

d = scan_result.disturbance;
fprintf('\nScan disturbance metadata:\n');
fprintf('  load_harmonic1 = %g, load_amp1 = %g\n', d.load_harmonic1, d.load_amp1);
fprintf('  load_harmonic2 = %g, load_amp2 = %g\n', d.load_harmonic2, d.load_amp2);

expected = cogging_load_config;
if ~(d.load_harmonic1 == expected.harmonic1 && d.load_harmonic2 == expected.harmonic2 && ...
    abs(d.load_amp1 - expected.amp1) < eps && abs(d.load_amp2 - expected.amp2) < eps && ...
    abs(d.load_base_torque - expected.load_base_torque) < eps)
    warning(['This scan result was not generated with single-period load torque only. ', ...
        'Rerun the position scan before exporting the FF CSV.']);
end
end

function [forward_input, backward_input, cfg] = local_parse_inputs(varargin)
forward_input = [];
backward_input = [];
cfg = struct();

if nargin == 0
    return;
end

forward_input = varargin{1};
if nargin >= 2
    second = varargin{2};
    if isstruct(second) && ~local_is_scan_result(second) && ~ischar(second) && ~isstring(second)
        cfg = second;
        return;
    end
    backward_input = second;
end
if nargin >= 3
    cfg = varargin{3};
end
end

function cfg = local_fill_defaults(cfg, forward)
defaults = struct();
defaults.csv_file = 'position_scan_ff_table.csv';
defaults.mat_file = 'position_scan_ff_table.mat';
defaults.smooth_window = 5;
defaults.use_mean_removal = true;
defaults.table_points = numel(forward.theta_command);

names = fieldnames(defaults);
for idx = 1:numel(names)
    name = names{idx};
    if ~isfield(cfg, name) || isempty(cfg.(name))
        cfg.(name) = defaults.(name);
    end
end

cfg.table_points = min(360, max(1, round(cfg.table_points)));
cfg.smooth_window = max(1, round(cfg.smooth_window));
end

function tf = local_is_scan_result(value)
tf = isstruct(value) && isfield(value, 'theta_command') && isfield(value, 'iq_selected');
end

function result = local_load_scan_result(input_value, base_name)
if nargin < 2
    base_name = 'position_scan_iq_result';
end

if isempty(input_value)
    if evalin('base', sprintf("exist('%s','var')", base_name))
        result = evalin('base', base_name);
        return;
    end
    if exist([base_name '.mat'], 'file')
        result = local_load_scan_result([base_name '.mat'], base_name);
        return;
    end
    error('No scan result found. Run save_position_scan_iq_table first, or pass a scan result MAT file.');
end

if local_is_scan_result(input_value)
    result = input_value;
    return;
end

if ischar(input_value) || isstring(input_value)
    loaded = load(char(input_value));
    fields = fieldnames(loaded);
    for idx = 1:numel(fields)
        candidate = loaded.(fields{idx});
        if local_is_scan_result(candidate)
            result = candidate;
            return;
        end
    end
    error('File %s does not contain a valid position scan result struct.', char(input_value));
end

error('Unsupported scan result input.');
end

function result = local_load_optional_scan_result(input_value)
if isempty(input_value)
    result = [];
else
    result = local_load_scan_result(input_value, 'position_scan_iq_result_backward');
end
end

function [scan, ff_table] = local_build_ff_table(forward, backward, cfg)
theta = mod(forward.theta_command(:), 2 * pi);
iq_forward = forward.iq_selected(:);

if isempty(backward)
    iq_cog = iq_forward;
    iq_backward_aligned = [];
    iq_direction = zeros(size(iq_forward));
    mode = 'single-direction';
else
    theta_backward = mod(backward.theta_command(:), 2 * pi);
    iq_backward = backward.iq_selected(:);
    iq_backward_aligned = zeros(size(iq_forward));
    for idx = 1:numel(theta)
        [~, back_idx] = min(abs(local_wrap_to_pi(theta_backward - theta(idx))));
        iq_backward_aligned(idx) = iq_backward(back_idx);
    end
    iq_cog = 0.5 * (iq_forward + iq_backward_aligned);
    iq_direction = 0.5 * (iq_forward - iq_backward_aligned);
    mode = 'forward-backward average';
end

iq_ff_raw = iq_cog;
if cfg.use_mean_removal
    iq_ff_raw = iq_ff_raw - mean(iq_ff_raw);
end
iq_ff = local_periodic_smooth(iq_ff_raw, cfg.smooth_window);

ff_table = zeros(360, 1);
count = min([cfg.table_points, numel(iq_ff), 360]);
ff_table(1:count) = iq_ff(1:count);

scan = struct();
scan.mode = mode;
scan.points = count;
scan.theta_rad = theta(1:count);
scan.theta_deg = rad2deg(theta(1:count));
scan.iq_forward = iq_forward(1:count);
scan.iq_backward = iq_backward_aligned;
if ~isempty(iq_backward_aligned)
    scan.iq_backward = iq_backward_aligned(1:count);
end
scan.iq_cog = iq_cog(1:count);
scan.iq_direction = iq_direction(1:count);
scan.iq_ff_raw = iq_ff_raw(1:count);
scan.iq_ff = iq_ff(1:count);
end

function export_table = local_build_export_table(scan, ff_table, cfg)
active_index = (1:scan.points).';
controller_index_1based = active_index;
is_active_point = true(scan.points, 1);

if isempty(scan.iq_backward)
    iq_backward_A = nan(scan.points, 1);
else
    iq_backward_A = scan.iq_backward(:);
end

export_table = table( ...
    active_index, ...
    controller_index_1based, ...
    scan.theta_deg(:), ...
    scan.theta_rad(:), ...
    scan.iq_forward(:), ...
    iq_backward_A, ...
    scan.iq_cog(:), ...
    scan.iq_direction(:), ...
    scan.iq_ff_raw(:), ...
    scan.iq_ff(:), ...
    ff_table(controller_index_1based), ...
    is_active_point, ...
    'VariableNames', {'index_1based', 'controller_index_1based', 'theta_deg', 'theta_rad', ...
    'iq_forward_A', 'iq_backward_A', 'iq_cog_A', 'iq_direction_A', 'iq_ff_raw_A', ...
    'iq_ff_A', 'ff_table_value_A', 'is_active_point'});
end

function y = local_periodic_smooth(x, window_len)
x = x(:);
window_len = max(1, round(window_len));
if window_len == 1
    y = x;
    return;
end
kernel = ones(window_len, 1) / window_len;
pad = floor(window_len / 2);
x_ext = [x(end-pad+1:end); x; x(1:pad)];
y_ext = conv(x_ext, kernel, 'same');
y = y_ext(pad+1:pad+numel(x));
end

function wrapped = local_wrap_to_pi(angle)
wrapped = mod(angle + pi, 2 * pi) - pi;
end