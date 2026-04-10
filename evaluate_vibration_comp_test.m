function metrics = evaluate_vibration_comp_test()
% Compare steady-state speed ripple with and without offline vibration compensation.

motor_control_params;
assignin('base', 'control', control);
assignin('base', 'motor', motor);
assignin('base', 'inverter', inverter);
assignin('base', 'simcfg', simcfg);

% Baseline: no compensation.
control.vib.mode = 'none';
assignin('base', 'control', control);
build_vibration_comp_test;
set_param('vibration_comp_test', 'InitFcn', '');
base_out = sim('vibration_comp_test', 'ReturnWorkspaceOutputs', 'on');
wm_base = base_out.get('log_vib_wm');

% Offline learning pass.
table_info = learn_vibration_ff_table();
control = evalin('base', 'control');
control.vib.mode = 'offline';
control.vib.enable_ff = 1;
assignin('base', 'control', control);
build_vibration_comp_test;
set_param('vibration_comp_test', 'InitFcn', '');
comp_out = sim('vibration_comp_test', 'ReturnWorkspaceOutputs', 'on');
wm_comp = comp_out.get('log_vib_wm');
iqff_comp = comp_out.get('log_vib_iqff');
learn_comp = comp_out.get('log_vib_learn');

window_start = max([control.vib.learn_start_time + 0.25, control.vib.ff_enable_time + 0.10, 0.40]);
window_end = control.vib.test_stop_time;

[base_std, base_pp] = local_ripple_metrics(wm_base, window_start, window_end);
[comp_std, comp_pp] = local_ripple_metrics(wm_comp, window_start, window_end);

metrics = struct();
metrics.window_start = window_start;
metrics.window_end = window_end;
metrics.base_ripple_std = base_std;
metrics.base_ripple_pp = base_pp;
metrics.comp_ripple_std = comp_std;
metrics.comp_ripple_pp = comp_pp;
metrics.std_reduction_pct = 100 * (base_std - comp_std) / max(base_std, eps);
metrics.pp_reduction_pct = 100 * (base_pp - comp_pp) / max(base_pp, eps);
metrics.iqff_rms = local_rms_window(iqff_comp, window_start, window_end);
metrics.learn_active_mean = local_mean_window(learn_comp, window_start, window_end);
metrics.ff_table_file = table_info.ff_table_file;

fprintf('\n=== Vibration compensation evaluation ===\n');
fprintf('Window              : [%.3f, %.3f] s\n', window_start, window_end);
fprintf('Baseline ripple std : %.6f rad/s\n', metrics.base_ripple_std);
fprintf('Comp ripple std     : %.6f rad/s\n', metrics.comp_ripple_std);
fprintf('Std reduction       : %.2f %%\n', metrics.std_reduction_pct);
fprintf('Baseline ripple p-p : %.6f rad/s\n', metrics.base_ripple_pp);
fprintf('Comp ripple p-p     : %.6f rad/s\n', metrics.comp_ripple_pp);
fprintf('P-P reduction       : %.2f %%\n', metrics.pp_reduction_pct);
fprintf('Iq_ff RMS           : %.6f A\n', metrics.iqff_rms);
fprintf('Learning active avg : %.3f\n', metrics.learn_active_mean);

assignin('base', 'vib_metrics', metrics);
end

function [ripple_std, ripple_pp] = local_ripple_metrics(ts, t0, t1)
mask = ts.Time >= t0 & ts.Time <= t1;
y = ts.Data(mask);
if isempty(y)
    ripple_std = NaN;
    ripple_pp = NaN;
    return;
end
y = y(:) - mean(y(:));
ripple_std = std(y);
ripple_pp = max(y) - min(y);
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

function y_mean = local_mean_window(ts, t0, t1)
mask = ts.Time >= t0 & ts.Time <= t1;
y = ts.Data(mask);
if isempty(y)
    y_mean = NaN;
else
    y_mean = mean(y(:));
end
end
