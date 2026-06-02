
function result = validate_position_scan_ff_table(varargin)
% Build a provisional FF table from position-scan IQ data and validate it.
%
% Usage:
%   result = validate_position_scan_ff_table;
%   result = validate_position_scan_ff_table(forward_result);
%   result = validate_position_scan_ff_table(forward_result, backward_result);
%   result = validate_position_scan_ff_table('forward.mat');
%   result = validate_position_scan_ff_table('forward.mat', 'backward.mat');
%   result = validate_position_scan_ff_table(..., cfg);
%
% If only one scan result is provided, the table is built from that single
% direction after mean removal and circular smoothing. If both forward and
% backward are provided, the table is built from their average.

[forward_input, backward_input, cfg] = local_parse_inputs(varargin{:});
forward = local_load_scan_result(forward_input, 'position_scan_iq_result');
backward = local_load_optional_scan_result(backward_input);

motor_control_params;
cfg = local_fill_defaults(cfg, control, forward);

[scan_result, ff_table] = local_build_ff_table(forward, backward, cfg);
save(cfg.ff_table_file, 'ff_table', 'cfg', 'scan_result');

validation = local_validate_table(ff_table, cfg, motor, inverter, control, simcfg);
export_info = local_export_ff_table(ff_table, scan_result, validation, cfg);
local_plot_results(scan_result, validation, cfg);

result = struct();
result.config = cfg;
result.scan = scan_result;
result.ff_table = ff_table;
result.ff_table_file = cfg.ff_table_file;
result.validation = validation;
result.export = export_info;

assignin('base', 'validated_scan_ff_table', ff_table);
assignin('base', 'validated_scan_result', result);

fprintf('\n=== Position-scan FF validation ===\n');
fprintf('Mode                 : %s\n', scan_result.mode);
fprintf('Table points         : %d\n', cfg.table_points);
fprintf('FF table file        : %s\n', cfg.ff_table_file);
fprintf('FF csv file          : %s\n', export_info.csv_file);
fprintf('FF text file         : %s\n', export_info.text_file);
fprintf('Validation std red.  : %.2f %%\n', validation.metrics.std_reduction_pct);
fprintf('Validation p-p red.  : %.2f %%\n', validation.metrics.pp_reduction_pct);
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
    error(['No scan result provided. Pass the output of save_position_scan_iq_table, ' ...
        'or ensure %s exists in the base workspace.'], base_name);
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

function cfg = local_fill_defaults(cfg, control, forward)
defaults = struct();
defaults.table_points = min(360, numel(forward.theta_command));
defaults.smooth_window = 5;
defaults.ff_output_limit = 0.25 * control.iq_ref_limit;
defaults.ff_enable_time = 0.55;
defaults.learn_start_time = 0.15;
defaults.test_stop_time = 1.0;
defaults.ff_table_file = 'validated_scan_ff_table.mat';
defaults.ff_csv_file = 'validated_scan_ff_table.csv';
defaults.ff_text_file = 'validated_scan_ff_table.txt';
load_defaults = cogging_load_config;
defaults.harmonic1 = local_get_forward_field(forward, 'disturbance.load_harmonic1', load_defaults.harmonic1);
defaults.harmonic2 = local_get_forward_field(forward, 'disturbance.load_harmonic2', load_defaults.harmonic2);
defaults.amp1 = local_get_forward_field(forward, 'disturbance.load_amp1', load_defaults.amp1);
defaults.amp2 = local_get_forward_field(forward, 'disturbance.load_amp2', load_defaults.amp2);
defaults.phase1_deg = local_get_forward_field(forward, 'disturbance.load_phase1_deg', load_defaults.phase1_deg);
defaults.phase2_deg = local_get_forward_field(forward, 'disturbance.load_phase2_deg', load_defaults.phase2_deg);
defaults.load_base_torque = local_get_forward_field(forward, 'disturbance.load_base_torque', load_defaults.load_base_torque);
defaults.plot_results = true;

names = fieldnames(defaults);
for idx = 1:numel(names)
    name = names{idx};
    if ~isfield(cfg, name) || isempty(cfg.(name))
        cfg.(name) = defaults.(name);
    end
end

cfg.table_points = min(360, max(8, round(cfg.table_points)));
cfg.smooth_window = max(1, round(cfg.smooth_window));
end

function [scan_result, ff_table] = local_build_ff_table(forward, backward, cfg)
theta = mod(forward.theta_command(:), 2 * pi);
iq_forward = forward.iq_selected(:);

if isempty(backward)
    iq_cog = iq_forward;
    iq_fric = zeros(size(iq_forward));
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
    iq_fric = 0.5 * (iq_forward - iq_backward_aligned);
    mode = 'forward-backward average';
end

iq_ff = iq_cog - mean(iq_cog);
iq_ff = local_periodic_smooth(iq_ff, cfg.smooth_window);

ff_table = zeros(360, 1);
count = min(cfg.table_points, numel(iq_ff));
ff_table(1:count) = iq_ff(1:count);

scan_result = struct();
scan_result.mode = mode;
scan_result.theta = theta;
scan_result.iq_forward = iq_forward;
scan_result.iq_cog = iq_cog;
scan_result.iq_fric = iq_fric;
scan_result.iq_ff = iq_ff;
if isempty(backward)
    scan_result.iq_backward = [];
else
    scan_result.iq_backward = iq_backward_aligned;
end
end

function validation = local_validate_table(ff_table, cfg, motor, inverter, control, simcfg)
control.vib.mode = 'none';
control.vib.enable_learning = 0;
control.vib.enable_ff = 0;
control.vib.ff_table = zeros(360, 1);
control.vib.ff_table_file = cfg.ff_table_file;
control = apply_cogging_load_config(control, cfg);
control.vib.table_points = cfg.table_points;
control.vib.output_limit = cfg.ff_output_limit;
control.vib.ff_enable_time = cfg.ff_enable_time;
control.vib.learn_start_time = cfg.learn_start_time;
control.vib.test_stop_time = cfg.test_stop_time;

assignin('base', 'motor', motor);
assignin('base', 'inverter', inverter);
assignin('base', 'simcfg', simcfg);

assignin('base', 'control', control);
build_vibration_comp_test;
set_param('vibration_comp_test', 'InitFcn', '');
base_out = sim('vibration_comp_test', 'ReturnWorkspaceOutputs', 'on');
if bdIsLoaded('vibration_comp_test')
    close_system('vibration_comp_test', 0);
end

control.vib.mode = 'offline';
control.vib.enable_ff = 1;
control.vib.ff_table = ff_table;
assignin('base', 'control', control);
build_vibration_comp_test;
set_param('vibration_comp_test', 'InitFcn', '');
comp_out = sim('vibration_comp_test', 'ReturnWorkspaceOutputs', 'on');
if bdIsLoaded('vibration_comp_test')
    close_system('vibration_comp_test', 0);
end

metrics = local_compare_validation(base_out, comp_out, control, cfg.ff_table_file);
validation = struct();
validation.base_out = base_out;
validation.comp_out = comp_out;
validation.metrics = metrics;
end

function metrics = local_compare_validation(base_out, comp_out, control, ff_table_file)
wm_base = base_out.get('log_vib_wm');
wm_comp = comp_out.get('log_vib_wm');
iqff_comp = comp_out.get('log_vib_iqff');

window_start = max([control.vib.learn_start_time + 0.25, control.vib.ff_enable_time + 0.10, 0.40]);
window_end = control.vib.test_stop_time;
base_y = local_window_centered(wm_base, window_start, window_end);
comp_y = local_window_centered(wm_comp, window_start, window_end);

metrics = struct();
metrics.window_start = window_start;
metrics.window_end = window_end;
metrics.base_ripple_std = std(base_y);
metrics.comp_ripple_std = std(comp_y);
metrics.base_ripple_pp = max(base_y) - min(base_y);
metrics.comp_ripple_pp = max(comp_y) - min(comp_y);
metrics.std_reduction_pct = 100 * (metrics.base_ripple_std - metrics.comp_ripple_std) / max(metrics.base_ripple_std, eps);
metrics.pp_reduction_pct = 100 * (metrics.base_ripple_pp - metrics.comp_ripple_pp) / max(metrics.base_ripple_pp, eps);
metrics.iqff_rms = local_rms_window(iqff_comp, window_start, window_end);
metrics.ff_table_file = ff_table_file;
end

function export_info = local_export_ff_table(ff_table, scan_result, validation, cfg)
point_index = (1:numel(ff_table)).';
theta_deg = point_index - 1;
theta_rad = deg2rad(theta_deg);
is_active_point = point_index <= cfg.table_points;

ff_export = table(point_index, theta_deg, theta_rad, ff_table(:), is_active_point, ...
    'VariableNames', {'index_1based', 'theta_deg', 'theta_rad', 'iq_ff_A', 'is_active_point'});
    
writetable(ff_export, cfg.ff_csv_file);

fid = fopen(cfg.ff_text_file, 'w');
if fid < 0
    error('Failed to open %s for writing.', cfg.ff_text_file);
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, 'Position-scan feedforward table export\n');
fprintf(fid, 'mode=%s\n', scan_result.mode);
fprintf(fid, 'table_points=%d\n', cfg.table_points);
fprintf(fid, 'ff_table_file=%s\n', cfg.ff_table_file);
fprintf(fid, 'ff_csv_file=%s\n', cfg.ff_csv_file);
fprintf(fid, 'harmonic1=%g\n', cfg.harmonic1);
fprintf(fid, 'harmonic2=%g\n', cfg.harmonic2);
fprintf(fid, 'amp1=%g\n', cfg.amp1);
fprintf(fid, 'amp2=%g\n', cfg.amp2);
fprintf(fid, 'phase1_deg=%g\n', cfg.phase1_deg);
fprintf(fid, 'phase2_deg=%g\n', cfg.phase2_deg);
fprintf(fid, 'load_base_torque=%g\n', cfg.load_base_torque);
fprintf(fid, 'std_reduction_pct=%.6f\n', validation.metrics.std_reduction_pct);
fprintf(fid, 'pp_reduction_pct=%.6f\n', validation.metrics.pp_reduction_pct);
fprintf(fid, '\n');
fprintf(fid, 'index_1based,theta_deg,theta_rad,iq_ff_A,is_active_point\n');
for idx = 1:height(ff_export)
    fprintf(fid, '%d,%.6f,%.9f,%.9f,%d\n', ff_export.index_1based(idx), ...
        ff_export.theta_deg(idx), ff_export.theta_rad(idx), ff_export.iq_ff_A(idx), ...
        ff_export.is_active_point(idx));
end

export_info = struct();
export_info.csv_file = cfg.ff_csv_file;
export_info.text_file = cfg.ff_text_file;
export_info.rows = height(ff_export);
end

function local_plot_results(scan_result, validation, cfg)
if ~cfg.plot_results
    return;
end

wm_base = validation.base_out.get('log_vib_wm');
wm_comp = validation.comp_out.get('log_vib_wm');

figure('Name', 'Position Scan FF Validation', 'Color', 'w');

subplot(2,2,1);
plot(rad2deg(scan_result.theta), scan_result.iq_forward, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.0); hold on;
if ~isempty(scan_result.iq_backward)
    plot(rad2deg(scan_result.theta), scan_result.iq_backward, 'Color', [0 0.45 0.74], 'LineWidth', 1.0);
    legend('Forward', 'Backward', 'Location', 'best');
else
    legend('Forward', 'Location', 'best');
end
grid on;
xlabel('Mechanical angle (deg)');
ylabel('Iq (A)');
title('Scan Data');

subplot(2,2,2);
plot(rad2deg(scan_result.theta), scan_result.iq_cog, 'Color', [0.47 0.67 0.19], 'LineWidth', 1.2); hold on;
plot(rad2deg(scan_result.theta), scan_result.iq_fric, 'Color', [0.49 0.18 0.56], 'LineWidth', 1.0);
grid on;
xlabel('Mechanical angle (deg)');
ylabel('Iq (A)');
title(sprintf('Estimated Components (%s)', scan_result.mode));
legend('Cogging estimate', 'Direction term', 'Location', 'best');

subplot(2,2,3);
plot(rad2deg(scan_result.theta), scan_result.iq_ff, 'k', 'LineWidth', 1.2);
grid on;
xlabel('Mechanical angle (deg)');
ylabel('Iq_{ff} (A)');
title('Provisional FF Table');

subplot(2,2,4);
base_y = local_window_centered(wm_base, validation.metrics.window_start, validation.metrics.window_end);
comp_y = local_window_centered(wm_comp, validation.metrics.window_start, validation.metrics.window_end);
plot(linspace(validation.metrics.window_start, validation.metrics.window_end, numel(base_y)), base_y, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.0); hold on;
plot(linspace(validation.metrics.window_start, validation.metrics.window_end, numel(comp_y)), comp_y, 'Color', [0 0.45 0.74], 'LineWidth', 1.0);
grid on;
xlabel('Time (s)');
ylabel('Centered speed (rad/s)');
title(sprintf('Validation Ripple, std reduction %.1f%%', validation.metrics.std_reduction_pct));
legend('Baseline', 'Offline FF', 'Location', 'best');
end

function y = local_window_centered(ts, t0, t1)
mask = ts.Time >= t0 & ts.Time <= t1;
y = ts.Data(mask);
y = y(:) - mean(y(:));
end

function y_rms = local_rms_window(ts, t0, t1)
mask = ts.Time >= t0 & ts.Time <= t1;
y = ts.Data(mask);
if isempty(y)
    y_rms = NaN;
else
    y_rms = rms(y(:));
end
end

function y = local_periodic_smooth(x, window_len)
x = x(:);
window_len = max(1, round(window_len));
if window_len == 1
    y = x;
    return;
end
pad = floor(window_len / 2);
kernel = ones(window_len, 1) / window_len;
x_ext = [x(end-pad+1:end); x; x(1:pad)];
y_ext = conv(x_ext, kernel, 'same');
y = y_ext(pad+1:pad+numel(x));
end

function wrapped = local_wrap_to_pi(angle)
wrapped = mod(angle + pi, 2 * pi) - pi;
end

function value = local_get_forward_field(forward, fieldPath, fallback)
value = fallback;
parts = split(fieldPath, '.');
current = forward;
for idx = 1:numel(parts)
    part = char(parts{idx});
    if isstruct(current) && isfield(current, part)
        current = current.(part);
    else
        return;
    end
end
value = current;
end