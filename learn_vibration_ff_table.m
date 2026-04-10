function table_info = learn_vibration_ff_table()
% Learn a fixed vibration compensation lookup table offline from baseline logs.

motor_control_params;
control.vib.mode = 'none';
control.vib.enable_learning = 0;
control.vib.enable_ff = 0;
assignin('base', 'control', control);

assignin('base', 'motor', motor);
assignin('base', 'inverter', inverter);
assignin('base', 'simcfg', simcfg);

build_vibration_comp_test;
set_param('vibration_comp_test', 'InitFcn', '');
sim_out = sim('vibration_comp_test', 'ReturnWorkspaceOutputs', 'on');

theta_ts = sim_out.get('log_vib_theta');
iqbase_ts = sim_out.get('log_vib_iqbase');
wm_ts = sim_out.get('log_vib_wm');

window_start = max(control.vib.learn_start_time + 0.25, 0.40);
window_end = control.vib.test_stop_time;
t_common = theta_ts.Time(:);
mask = t_common >= window_start & t_common <= window_end;

theta = theta_ts.Data(mask);
iqbase = interp1(iqbase_ts.Time(:), iqbase_ts.Data(:), t_common(mask), 'linear', 'extrap');
wm = interp1(wm_ts.Time(:), wm_ts.Data(:), t_common(mask), 'linear', 'extrap');
if isempty(theta) || isempty(iqbase)
    error('No data in offline learning window.');
end

points = control.vib.table_points;
bin_edges = linspace(0, 2 * pi, points + 1);
theta_wrap = mod(theta(:), 2 * pi);
iqbase = iqbase(:);
learn_signal = iqbase - mean(iqbase);

active_table = zeros(points, 1);
counts = zeros(points, 1);
for idx = 1:points
    if idx < points
        in_bin = theta_wrap >= bin_edges(idx) & theta_wrap < bin_edges(idx + 1);
    else
        in_bin = theta_wrap >= bin_edges(idx) & theta_wrap <= bin_edges(idx + 1);
    end
    if any(in_bin)
        active_table(idx) = mean(learn_signal(in_bin));
        counts(idx) = sum(in_bin);
    end
end

% Fill empty bins by interpolation around the circle.
valid = counts > 0;
if nnz(valid) < 2
    error('Insufficient angular coverage to build offline FF table.');
end
bin_centers = (bin_edges(1:end-1) + bin_edges(2:end)) / 2;
valid_x = bin_centers(valid).';
valid_y = active_table(valid);
valid_x_ext = [valid_x(end) - 2 * pi; valid_x; valid_x(1) + 2 * pi];
valid_y_ext = [valid_y(end); valid_y; valid_y(1)];
active_table = interp1(valid_x_ext, valid_y_ext, bin_centers(:), 'linear');
active_table = active_table(:);

ff_table = zeros(360, 1);
ff_table(1:points) = active_table;
control.vib.ff_table = ff_table;
assignin('base', 'control', control);
save(control.vib.ff_table_file, 'ff_table', 'points', 'window_start', 'window_end');

table_info = struct();
table_info.points = points;
table_info.window_start = window_start;
table_info.window_end = window_end;
table_info.ff_table_file = control.vib.ff_table_file;
table_info.ff_table = ff_table;
table_info.learn_signal_rms = rms(learn_signal);
table_info.speed_mean = mean(wm);

fprintf('\n=== Offline vibration FF table learned ===\n');
fprintf('Points            : %d\n', points);
fprintf('Window            : [%.3f, %.3f] s\n', window_start, window_end);
fprintf('Learn signal RMS  : %.6f A\n', table_info.learn_signal_rms);
fprintf('Mean speed        : %.6f rad/s\n', table_info.speed_mean);
fprintf('Saved table file  : %s\n', control.vib.ff_table_file);

assignin('base', 'vib_table_info', table_info);
end
