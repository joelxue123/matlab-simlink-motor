function result = show_vibration_comp_effect()
% Run baseline and offline vibration compensation back-to-back and plot comparison.

motor_control_params;
assignin('base', 'control', control);
assignin('base', 'motor', motor);
assignin('base', 'inverter', inverter);
assignin('base', 'simcfg', simcfg);

% Baseline run: no compensation
control.vib.mode = 'none';
assignin('base', 'control', control);
build_vibration_comp_test;
set_param('vibration_comp_test', 'InitFcn', '');
out_base = sim('vibration_comp_test', 'ReturnWorkspaceOutputs', 'on');

% Offline learning + compensation run
info = learn_vibration_ff_table();
control = evalin('base', 'control');
control.vib.mode = 'offline';
control.vib.enable_ff = 1;
assignin('base', 'control', control);
build_vibration_comp_test;
set_param('vibration_comp_test', 'InitFcn', '');
out_comp = sim('vibration_comp_test', 'ReturnWorkspaceOutputs', 'on');

wref_base = out_base.get('log_vib_wref');
wm_base = out_base.get('log_vib_wm');
tload_base = out_base.get('log_vib_tload');

wref_comp = out_comp.get('log_vib_wref');
wm_comp = out_comp.get('log_vib_wm');
iqff_comp = out_comp.get('log_vib_iqff');
tload_comp = out_comp.get('log_vib_tload');

window_start = max([control.vib.learn_start_time + 0.25, control.vib.ff_enable_time + 0.10, 0.40]);
window_end = control.vib.test_stop_time;
ff_enable_time = control.vib.ff_enable_time;

figure('Name', 'Vibration Compensation Comparison', 'Color', 'w');
subplot(3,1,1);
plot(wref_base.Time, wref_base.Data, 'k--', 'LineWidth', 1.0); hold on;
plot(wm_base.Time, wm_base.Data, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.0);
plot(wm_comp.Time, wm_comp.Data, 'Color', [0 0.45 0.74], 'LineWidth', 1.0);
xline(window_start, ':', 'Window Start');
xline(window_end, ':', 'Window End');
xline(ff_enable_time, '--', 'FF On');
grid on;
legend('w_{ref}', 'w_{meas} baseline', 'w_{meas} offline FF', 'Location', 'best');
title('Speed Comparison');
ylabel('rad/s');

subplot(3,1,2);
plot(tload_base.Time, tload_base.Data, 'Color', [0.49 0.18 0.56], 'LineWidth', 1.0); hold on;
plot(iqff_comp.Time, iqff_comp.Data, 'Color', [0.47 0.67 0.19], 'LineWidth', 1.0);
xline(window_start, ':', 'Window Start');
xline(window_end, ':', 'Window End');
xline(ff_enable_time, '--', 'FF On');
grid on;
legend('T_{load}', 'Iq_{ff}', 'Location', 'best');
title('Load Ripple and Feedforward');
ylabel('N*m / A');

subplot(3,1,3);
wm_base_i = interp1(wm_base.Time(:), wm_base.Data(:), wm_comp.Time(:), 'linear', 'extrap');
plot(wm_comp.Time, wm_base_i - mean(wm_base_i), 'Color', [0.85 0.33 0.10], 'LineWidth', 1.0); hold on;
plot(wm_comp.Time, wm_comp.Data - mean(wm_comp.Data), 'Color', [0 0.45 0.74], 'LineWidth', 1.0);
xline(window_start, ':', 'Window Start');
xline(window_end, ':', 'Window End');
xline(ff_enable_time, '--', 'FF On');
grid on;
legend('Baseline ripple', 'Offline FF ripple', 'Location', 'best');
title('Centered Speed Ripple');
xlabel('Time (s)');
ylabel('rad/s');

metrics = evaluate_vibration_comp_test();
result = struct();
result.table_info = info;
result.metrics = metrics;
assignin('base', 'vib_compare_result', result);
end
