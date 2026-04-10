function report = estimate_speedloop_bandwidth()
% Estimate closed-loop speed bandwidth from the existing step-response test.
%
% Usage:
%   report = estimate_speedloop_bandwidth();

motor_control_params;
assignin('base', 'control', control);
assignin('base', 'motor', motor);
assignin('base', 'inverter', inverter);
assignin('base', 'simcfg', simcfg);
build_speedloop_kf_test;
set_param('speedloop_kf_test', 'InitFcn', '');
sim_out = sim('speedloop_kf_test', 'ReturnWorkspaceOutputs', 'on');

wref_ts = sim_out.get('log_wref');
wm_ts = sim_out.get('log_wm');
wkf_ts = sim_out.get('log_wkf');

if isempty(wref_ts) || isempty(wm_ts) || isempty(wkf_ts)
    error('Missing speed logs from speedloop_kf_test.');
end

step_time = 0.02;
step_down_time = 0.30;
t = wm_ts.Time(:);
wm = wm_ts.Data(:);
wref = interp1(wref_ts.Time(:), wref_ts.Data(:), t, 'linear', 'extrap');
wkf = interp1(wkf_ts.Time(:), wkf_ts.Data(:), t, 'linear', 'extrap');

wm_metrics = local_step_metrics(t, wm, step_time, step_down_time);
wkf_metrics = local_step_metrics(t, wkf, step_time, step_down_time);

kf_delay_s = wkf_metrics.t50 - wm_metrics.t50;
phase_at_design_deg = 360 * control.speed_bandwidth_hz * kf_delay_s;
phase_at_meas_bw_deg = 360 * wm_metrics.bw_hz * kf_delay_s;

report = struct();
report.design_speed_bw_hz = control.speed_bandwidth_hz;
report.design_speed_bw_rad_s = control.speed_bandwidth_rad_s;
report.wm = wm_metrics;
report.wkf = wkf_metrics;
report.kf_delay_s = kf_delay_s;
report.kf_delay_us = kf_delay_s * 1e6;
report.phase_at_design_deg = phase_at_design_deg;
report.phase_at_meas_bw_deg = phase_at_meas_bw_deg;

fprintf('\n=== Speed-loop bandwidth estimate ===\n');
fprintf('Designed speed BW : %.3f Hz (%.3f rad/s)\n', ...
    report.design_speed_bw_hz, report.design_speed_bw_rad_s);
fprintf('\nFrom w_meas step response:\n');
fprintf('  t10           : %.6f s\n', wm_metrics.t10);
fprintf('  t50           : %.6f s\n', wm_metrics.t50);
fprintf('  t63           : %.6f s\n', wm_metrics.t63);
fprintf('  t90           : %.6f s\n', wm_metrics.t90);
fprintf('  rise time     : %.6f s\n', wm_metrics.rise_time_s);
fprintf('  tau_eq        : %.6f s\n', wm_metrics.tau_eq_s);
fprintf('  bw_eq         : %.3f Hz (%.3f rad/s)\n', wm_metrics.bw_hz, wm_metrics.bw_rad_s);
fprintf('\nFrom w_kf step response:\n');
fprintf('  t10           : %.6f s\n', wkf_metrics.t10);
fprintf('  t50           : %.6f s\n', wkf_metrics.t50);
fprintf('  t63           : %.6f s\n', wkf_metrics.t63);
fprintf('  t90           : %.6f s\n', wkf_metrics.t90);
fprintf('  rise time     : %.6f s\n', wkf_metrics.rise_time_s);
fprintf('  tau_eq        : %.6f s\n', wkf_metrics.tau_eq_s);
fprintf('  bw_eq         : %.3f Hz (%.3f rad/s)\n', wkf_metrics.bw_hz, wkf_metrics.bw_rad_s);
fprintf('\nKF relative delay (t50): %.3f us\n', report.kf_delay_us);
fprintf('Phase impact at designed BW: %.3f deg\n', report.phase_at_design_deg);
fprintf('Phase impact at measured BW: %.3f deg\n', report.phase_at_meas_bw_deg);

assignin('base', 'speedloop_bw_report', report);
end

function metrics = local_step_metrics(t, y, step_time, step_down_time)
pre_mask = t >= max(0, step_time - 0.01) & t < step_time - 0.002;
post_mask = t >= step_time + 0.08 & t < min(step_down_time - 0.02, step_time + 0.20);
if ~any(pre_mask) || ~any(post_mask)
    error('Insufficient data window for step metric calculation.');
end

initial_value = mean(y(pre_mask));
final_value = mean(y(post_mask));
step_amp = final_value - initial_value;
if abs(step_amp) < eps
    error('Step amplitude too small to estimate bandwidth.');
end

level10 = initial_value + 0.1 * step_amp;
level50 = initial_value + 0.5 * step_amp;
level63 = initial_value + 0.6321205588 * step_amp;
level90 = initial_value + 0.9 * step_amp;

search_mask = t >= step_time & t < step_down_time;
ts = t(search_mask);
ys = y(search_mask);

metrics = struct();
metrics.t10 = local_first_cross(ts, ys, level10);
metrics.t50 = local_first_cross(ts, ys, level50);
metrics.t63 = local_first_cross(ts, ys, level63);
metrics.t90 = local_first_cross(ts, ys, level90);
metrics.rise_time_s = metrics.t90 - metrics.t10;
metrics.tau_eq_s = metrics.t63 - step_time;
metrics.bw_hz = 1 / (2 * pi * metrics.tau_eq_s);
metrics.bw_rad_s = 1 / metrics.tau_eq_s;
end

function t_cross = local_first_cross(t, y, level)
idx = find(y >= level, 1, 'first');
if isempty(idx)
    t_cross = NaN;
    return;
end
if idx == 1
    t_cross = t(1);
    return;
end
x1 = t(idx - 1);
x2 = t(idx);
y1 = y(idx - 1);
y2 = y(idx);
if y2 == y1
    t_cross = x2;
else
    t_cross = x1 + (level - y1) * (x2 - x1) / (y2 - y1);
end
end
