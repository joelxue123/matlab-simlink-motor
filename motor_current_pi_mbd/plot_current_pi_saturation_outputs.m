%% Plot current PI saturation output data
%
% Saves a PNG comparing the default anti-windup case against Kaw_iq = 0.

clear; clc;

script_dir = fileparts(mfilename('fullpath'));
run(fullfile(script_dir, 'build_current_pi_saturation_test_model.m'));

cfg = evalin('base', 'current_pi_saturation_test_config');
sat_test = evalin('base', 'current_pi_sat_test');
model = cfg.modelName;
model_file = fullfile(script_dir, [model '.slx']);

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

png_file = fullfile(report_dir, 'current_pi_saturation_outputs.png');
save_plot(png_file, with_aw, without_aw, sat_test);

fprintf('\nSaved PI output plot:\n  %s\n', png_file);

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

function save_plot(png_file, with_aw, without_aw, sat_test)
v_limit = double(sat_test.vdc) * double(sat_test.v_limit_ratio);
release_time = sat_test.ref.release_time;

fig = figure('Visible', 'off', ...
    'Color', 'w', ...
    'Position', [100 100 1200 850]);
tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(with_aw.t * 1000, with_aw.vq_ref, 'LineWidth', 1.8);
hold on;
plot(without_aw.t * 1000, without_aw.vq_ref, '--', 'LineWidth', 1.8);
yline(v_limit, ':', 'LineWidth', 1.2);
yline(-v_limit, ':', 'LineWidth', 1.2);
xline(release_time * 1000, ':', 'LineWidth', 1.2);
grid on;
title('PI output: vq\_ref saturation and release');
ylabel('vq\_ref (V)');
legend('Kaw\_iq = default', 'Kaw\_iq = 0', '+V limit', '-V limit', ...
    'release', 'Location', 'northeast');

nexttile;
plot(with_aw.t * 1000, with_aw.vd_ref, 'LineWidth', 1.8);
hold on;
plot(without_aw.t * 1000, without_aw.vd_ref, '--', 'LineWidth', 1.8);
xline(release_time * 1000, ':', 'LineWidth', 1.2);
grid on;
title('PI output: vd\_ref');
ylabel('vd\_ref (V)');
legend('Kaw\_iq = default', 'Kaw\_iq = 0', 'release', ...
    'Location', 'northeast');

nexttile;
plot(with_aw.t * 1000, with_aw.iq_ref, 'k:', 'LineWidth', 1.6);
hold on;
plot(with_aw.t * 1000, with_aw.iq_meas, 'LineWidth', 1.8);
plot(without_aw.t * 1000, without_aw.iq_meas, '--', 'LineWidth', 1.8);
xline(release_time * 1000, ':', 'LineWidth', 1.2);
grid on;
title('q-axis current response context');
xlabel('Time (ms)');
ylabel('iq (A)');
legend('iq\_ref', 'iq\_meas, Kaw\_iq = default', ...
    'iq\_meas, Kaw\_iq = 0', 'release', 'Location', 'northeast');

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
