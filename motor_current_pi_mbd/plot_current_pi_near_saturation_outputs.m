%% Plot current PI output just above the saturation boundary
%
% This scenario uses the existing saturation-test harness but sets iq_ref only
% slightly above what the low Vdc limit can sustain.

clear; clc;

script_dir = fileparts(mfilename('fullpath'));
run(fullfile(script_dir, 'build_current_pi_saturation_test_model.m'));

cfg = evalin('base', 'current_pi_saturation_test_config');
sat_test = evalin('base', 'current_pi_sat_test');
model = cfg.modelName;
model_file = fullfile(script_dir, [model '.slx']);

sat_test.ref.iq_high = single(7.0);
sat_test.ref.release_time = 0.035;
sat_test.simcfg.stop_time = 0.040;
assignin('base', 'current_pi_sat_test', sat_test);
assignin('base', 'current_pi_simcfg', sat_test.simcfg);

default_kaw = get_dictionary_parameter(script_dir, 'Kaw_iq');
cleanup = onCleanup(@() restore_default_model(script_dir));

set_dictionary_parameter(script_dir, 'Kaw_iq', default_kaw);
with_aw = run_case(model_file, model);

set_dictionary_parameter(script_dir, 'Kaw_iq', single(0));
without_aw = run_case(model_file, model);

report_dir = fullfile(script_dir, 'reports');
if ~exist(report_dir, 'dir')
    mkdir(report_dir);
end

png_file = fullfile(report_dir, 'current_pi_near_saturation_outputs.png');
metrics = calculate_metrics(with_aw, without_aw, sat_test);
save_plot(png_file, with_aw, without_aw, sat_test, metrics);
print_metrics(metrics);

fprintf('\nSaved near-saturation PI output plot:\n  %s\n', png_file);

function result = run_case(model_file, model)
load_system(model_file);
set_param(model, 'SimulationCommand', 'update');

sim_out = sim(model, ...
    'ReturnWorkspaceOutputs', 'on', ...
    'SaveOutput', 'off');

iq_ref = sim_out.get('log_iq_ref_sat');
iq_meas = sim_out.get('log_iq_meas_sat');
vd_ref = sim_out.get('log_vd_ref_sat');
vq_ref = sim_out.get('log_vq_ref_sat');

result.t = iq_ref.Time;
result.iq_ref = double(iq_ref.Data(:));
result.iq_meas = double(iq_meas.Data(:));
result.vd_ref = double(vd_ref.Data(:));
result.vq_ref = double(vq_ref.Data(:));

close_system(model, 0);
end

function metrics = calculate_metrics(with_aw, without_aw, sat_test)
settled_window = with_aw.t >= 0.020 & with_aw.t < sat_test.ref.release_time;
v_limit = double(sat_test.vdc) * double(sat_test.v_limit_ratio);

metrics.v_limit = v_limit;
metrics.iq_ref = double(sat_test.ref.iq_high);
metrics.over_current = metrics.iq_ref - v_limit;
metrics.over_percent = 100 * metrics.over_current / v_limit;

metrics.with_aw_vq_min = min(with_aw.vq_ref(settled_window));
metrics.with_aw_vq_max = max(with_aw.vq_ref(settled_window));
metrics.with_aw_vq_pp = metrics.with_aw_vq_max - metrics.with_aw_vq_min;
metrics.with_aw_vq_std = std(with_aw.vq_ref(settled_window));
metrics.with_aw_iq_std = std(with_aw.iq_meas(settled_window));

metrics.without_aw_vq_min = min(without_aw.vq_ref(settled_window));
metrics.without_aw_vq_max = max(without_aw.vq_ref(settled_window));
metrics.without_aw_vq_pp = metrics.without_aw_vq_max - metrics.without_aw_vq_min;
metrics.without_aw_vq_std = std(without_aw.vq_ref(settled_window));
metrics.without_aw_iq_std = std(without_aw.iq_meas(settled_window));
end

function print_metrics(metrics)
fprintf('\nNear-saturation PI output metrics:\n');
fprintf('  v_limit                  = %.9g V\n', metrics.v_limit);
fprintf('  iq_ref                   = %.9g A\n', metrics.iq_ref);
fprintf('  iq_ref - limit equivalent = %.9g A (%.6g%%)\n', ...
    metrics.over_current, metrics.over_percent);
fprintf('  settled window           = [20ms, 35ms)\n');

fprintf('\nWith anti-windup:\n');
fprintf('  vq_ref range             = [%.9g, %.9g] V\n', ...
    metrics.with_aw_vq_min, metrics.with_aw_vq_max);
fprintf('  vq_ref p-p/std           = %.9g / %.9g V\n', ...
    metrics.with_aw_vq_pp, metrics.with_aw_vq_std);
fprintf('  iq_meas std              = %.9g A\n', metrics.with_aw_iq_std);

fprintf('\nWithout anti-windup:\n');
fprintf('  vq_ref range             = [%.9g, %.9g] V\n', ...
    metrics.without_aw_vq_min, metrics.without_aw_vq_max);
fprintf('  vq_ref p-p/std           = %.9g / %.9g V\n', ...
    metrics.without_aw_vq_pp, metrics.without_aw_vq_std);
fprintf('  iq_meas std              = %.9g A\n', metrics.without_aw_iq_std);
end

function save_plot(png_file, with_aw, without_aw, sat_test, metrics)
v_limit = metrics.v_limit;
release_time = sat_test.ref.release_time;

fig = figure('Visible', 'off', ...
    'Color', 'w', ...
    'Position', [100 100 1200 900]);
tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(with_aw.t * 1000, with_aw.vq_ref, 'LineWidth', 1.8);
hold on;
plot(without_aw.t * 1000, without_aw.vq_ref, '--', 'LineWidth', 1.8);
yline(v_limit, ':', 'LineWidth', 1.2);
xline(release_time * 1000, ':', 'LineWidth', 1.2);
grid on;
title('PI output vq\_ref just above saturation');
ylabel('vq\_ref (V)');
legend('Kaw\_iq = default', 'Kaw\_iq = 0', '+V limit', ...
    'release', 'Location', 'southeast');

nexttile;
plot(with_aw.t * 1000, with_aw.vq_ref, 'LineWidth', 1.8);
hold on;
plot(without_aw.t * 1000, without_aw.vq_ref, '--', 'LineWidth', 1.8);
yline(v_limit, ':', 'LineWidth', 1.2);
xline(release_time * 1000, ':', 'LineWidth', 1.2);
grid on;
title('Zoom: saturated hold window');
xlim([15 release_time * 1000]);
ylim([v_limit - 0.04, v_limit + 0.04]);
ylabel('vq\_ref (V)');
legend('Kaw\_iq = default', 'Kaw\_iq = 0', '+V limit', ...
    'release', 'Location', 'southeast');

nexttile;
plot(with_aw.t * 1000, with_aw.iq_ref, 'k:', 'LineWidth', 1.6);
hold on;
plot(with_aw.t * 1000, with_aw.iq_meas, 'LineWidth', 1.8);
plot(without_aw.t * 1000, without_aw.iq_meas, '--', 'LineWidth', 1.8);
xline(release_time * 1000, ':', 'LineWidth', 1.2);
grid on;
title('q-axis current response near saturation');
xlabel('Time (ms)');
ylabel('iq (A)');
legend('iq\_ref', 'iq\_meas, Kaw\_iq = default', ...
    'iq\_meas, Kaw\_iq = 0', 'release', 'Location', 'southeast');

exportgraphics(fig, png_file, 'Resolution', 150);
close(fig);
end

function value = get_dictionary_parameter(script_dir, name)
dd = Simulink.data.dictionary.open(fullfile(script_dir, 'current_pi_interface.sldd'));
cleanup = onCleanup(@() close(dd));
section = getSection(dd, 'Design Data');
entry = getEntry(section, name);
parameter = getValue(entry);
value = single(parameter.Value);
end

function set_dictionary_parameter(script_dir, name, value)
dd = Simulink.data.dictionary.open(fullfile(script_dir, 'current_pi_interface.sldd'));
cleanup = onCleanup(@() close(dd));
section = getSection(dd, 'Design Data');
entry = getEntry(section, name);
parameter = getValue(entry);
parameter.Value = double(value);
setValue(entry, parameter);
saveChanges(dd);
end

function restore_default_model(script_dir)
try
    run(fullfile(script_dir, 'build_current_pi_model.m'));
catch err
    warning('Could not restore default current PI model after plotting: %s', ...
        err.message);
end
end
