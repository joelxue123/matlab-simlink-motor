function result = run_study(cfg)
% Build and run the model-following position-reference demo.
%
% Example:
%   cd studies/model_following_position_ref
%   result = run_study;
%
% Optional:
%   result = run_study(struct('ref_bandwidth_hz', 8, 'step_rad', pi));
%   result = run_study(struct('open_model', true));

if nargin < 1
    cfg = struct();
end

study_dir = fileparts(mfilename('fullpath'));
output_dir = fullfile(study_dir, 'outputs');
if exist(output_dir, 'dir') ~= 7
    mkdir(output_dir);
end

project_root = local_find_project_root(study_dir);
addpath(project_root);
project_root = init_project_paths(study_dir);
old_dir = pwd;
cleanup = onCleanup(@() cd(old_dir));
cd(study_dir);

mdl_path = build_model_following_position_ref_model(cfg);
[~, mdl_name, ~] = fileparts(mdl_path);

sim_out = sim(mdl_name, 'ReturnWorkspaceOutputs', 'on');
mf_cfg = evalin('base', 'mf_cfg');

signals = local_extract_signals(sim_out);
metrics = local_compute_metrics(signals, mf_cfg);

result = struct();
result.model_path = mdl_path;
result.output_dir = output_dir;
result.project_root = project_root;
result.config = mf_cfg;
result.signals = signals;
result.metrics = metrics;

save(fullfile(output_dir, 'model_following_position_ref_result.mat'), 'result');
local_export_csv(fullfile(output_dir, 'model_following_position_ref_signals.csv'), signals);
local_plot(result, fullfile(output_dir, 'model_following_position_ref.png'));

fprintf('\n=== Model-following position reference demo ===\n');
fprintf('Model                  : %s\n', mdl_name);
fprintf('Step                   : %.6g rad at %.6g s\n', mf_cfg.step_rad, mf_cfg.step_time);
fprintf('Reference bandwidth    : %.6g Hz, damping %.6g\n', ...
    mf_cfg.ref_bandwidth_hz, mf_cfg.ref_damping);
fprintf('Reference limits       : max vel %.6g rad/s, max acc %.6g rad/s^2\n', ...
    mf_cfg.max_vel_ref_rad_s, mf_cfg.max_acc_ref_rad_s2);
fprintf('Axis P gain / vFF gain : %.6g / %.6g\n', ...
    mf_cfg.axis_pos_kp, mf_cfg.axis_vel_ff_gain);
fprintf('Peak velocity ref      : %.6g rad/s\n', metrics.peak_vel_ref);
fprintf('Peak accel ref         : %.6g rad/s^2\n', metrics.peak_acc_ref);
fprintf('Peak unsat accel cmd   : %.6g rad/s^2\n', metrics.peak_acc_unsat);
fprintf('Accel saturation ratio : %.3f%% after step\n', metrics.acc_saturation_ratio_pct);
fprintf('Raw axis overshoot     : %.6g rad (%.3f%%)\n', ...
    metrics.axis_raw_overshoot_rad, metrics.axis_raw_overshoot_pct);
fprintf('MF pos-only overshoot  : %.6g rad (%.3f%%)\n', ...
    metrics.axis_mf_noff_overshoot_rad, metrics.axis_mf_noff_overshoot_pct);
fprintf('MF + vFF overshoot     : %.6g rad (%.3f%%)\n', ...
    metrics.axis_mf_vff_overshoot_rad, metrics.axis_mf_vff_overshoot_pct);
fprintf('MF pos-only max error  : %.6g rad\n', metrics.axis_mf_noff_max_ref_err);
fprintf('MF + vFF max error     : %.6g rad\n', metrics.axis_mf_vff_max_ref_err);
fprintf('Output dir             : %s\n', output_dir);

if isfield(mf_cfg, 'open_model') && mf_cfg.open_model
    open_system(mdl_name);
    open_system([mdl_name '/Reference Signals Scope']);
    open_system([mdl_name '/Axis Compare Scope']);
    open_system([mdl_name '/Speed Command Scope']);
end
end

function signals = local_extract_signals(sim_out)
signals = struct();
signals.cmd_raw = local_get(sim_out, 'cmd_raw_log');
signals.pos_ref = local_get(sim_out, 'pos_ref_log');
signals.vel_ref = local_get(sim_out, 'vel_ref_log');
signals.acc_ref = local_get(sim_out, 'acc_ref_log');
signals.acc_unsat = local_get(sim_out, 'acc_unsat_log');
signals.acc_sat_flag = local_get(sim_out, 'acc_sat_flag_log');
signals.axis_raw = local_get(sim_out, 'axis_raw_log');
signals.axis_mf_noff = local_get(sim_out, 'axis_mf_noff_log');
signals.axis_mf_vff = local_get(sim_out, 'axis_mf_vff_log');
signals.speed_cmd_raw = local_get(sim_out, 'speed_cmd_raw_log');
signals.speed_cmd_mf_noff = local_get(sim_out, 'speed_cmd_mf_noff_log');
signals.speed_cmd_mf_vff = local_get(sim_out, 'speed_cmd_mf_vff_log');
end

function sig = local_get(sim_out, name)
raw = sim_out.get(name);
sig = struct();
sig.time = raw.time(:);
sig.value = raw.signals.values(:);
end

function metrics = local_compute_metrics(signals, cfg)
final_cmd = cfg.step_rad;
step_mask = signals.cmd_raw.time >= cfg.step_time;

metrics = struct();
metrics.peak_vel_ref = max(abs(signals.vel_ref.value));
metrics.peak_acc_ref = max(abs(signals.acc_ref.value));
metrics.peak_acc_unsat = max(abs(signals.acc_unsat.value));
metrics.acc_saturation_ratio_pct = 100 * mean(signals.acc_sat_flag.value(step_mask) > 0.5);

[metrics.axis_raw_overshoot_rad, metrics.axis_raw_overshoot_pct] = ...
    local_overshoot(signals.axis_raw, final_cmd, cfg.step_time);
[metrics.axis_mf_noff_overshoot_rad, metrics.axis_mf_noff_overshoot_pct] = ...
    local_overshoot(signals.axis_mf_noff, final_cmd, cfg.step_time);
[metrics.axis_mf_vff_overshoot_rad, metrics.axis_mf_vff_overshoot_pct] = ...
    local_overshoot(signals.axis_mf_vff, final_cmd, cfg.step_time);

metrics.axis_raw_settle_s = local_settle_time(signals.axis_raw, final_cmd, cfg.step_time);
metrics.axis_mf_noff_settle_s = local_settle_time(signals.axis_mf_noff, final_cmd, cfg.step_time);
metrics.axis_mf_vff_settle_s = local_settle_time(signals.axis_mf_vff, final_cmd, cfg.step_time);

metrics.axis_mf_noff_max_ref_err = local_max_ref_error(signals.pos_ref, signals.axis_mf_noff, cfg.step_time);
metrics.axis_mf_vff_max_ref_err = local_max_ref_error(signals.pos_ref, signals.axis_mf_vff, cfg.step_time);
end

function [overshoot_rad, overshoot_pct] = local_overshoot(sig, final_value, step_time)
mask = sig.time >= step_time;
after = sig.value(mask);
overshoot_rad = max(after) - final_value;
overshoot_pct = 100 * overshoot_rad / max(abs(final_value), eps);
end

function value = local_max_ref_error(ref_sig, feedback_sig, step_time)
mask = feedback_sig.time >= step_time;
ref_interp = interp1(ref_sig.time(:), ref_sig.value(:), feedback_sig.time(:), 'linear', 'extrap');
value = max(abs(ref_interp(mask) - feedback_sig.value(mask)));
end

function settle_s = local_settle_time(sig, final_value, step_time)
band = 0.02 * max(abs(final_value), eps);
idx0 = find(sig.time >= step_time, 1, 'first');
settle_s = NaN;
if isempty(idx0)
    return;
end
err = abs(sig.value - final_value);
for idx = idx0:numel(sig.time)
    if all(err(idx:end) <= band)
        settle_s = sig.time(idx) - step_time;
        return;
    end
end
end

function local_export_csv(file_name, signals)
time = signals.cmd_raw.time(:);
cmd_raw = signals.cmd_raw.value(:);
pos_ref = local_interp(signals.pos_ref, time);
vel_ref = local_interp(signals.vel_ref, time);
acc_ref = local_interp(signals.acc_ref, time);
acc_unsat = local_interp(signals.acc_unsat, time);
acc_sat_flag = local_interp(signals.acc_sat_flag, time);
axis_raw = local_interp(signals.axis_raw, time);
axis_mf_noff = local_interp(signals.axis_mf_noff, time);
axis_mf_vff = local_interp(signals.axis_mf_vff, time);
speed_cmd_raw = local_interp(signals.speed_cmd_raw, time);
speed_cmd_mf_noff = local_interp(signals.speed_cmd_mf_noff, time);
speed_cmd_mf_vff = local_interp(signals.speed_cmd_mf_vff, time);

tbl = table(time, cmd_raw, pos_ref, vel_ref, acc_ref, acc_unsat, acc_sat_flag, ...
    axis_raw, axis_mf_noff, axis_mf_vff, ...
    speed_cmd_raw, speed_cmd_mf_noff, speed_cmd_mf_vff);
writetable(tbl, file_name);
end

function values = local_interp(sig, time)
values = interp1(sig.time(:), sig.value(:), time(:), 'linear', 'extrap');
end

function local_plot(result, file_name)
signals = result.signals;
metrics = result.metrics;

fig = figure('Name', 'Model Following Position Reference', 'Color', 'w');
tiledlayout(fig, 4, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(signals.cmd_raw.time, signals.cmd_raw.value, 'Color', [0.45 0.45 0.45], ...
    'LineWidth', 1.0); hold on;
plot(signals.pos_ref.time, signals.pos_ref.value, 'b', 'LineWidth', 1.4);
grid on;
ylabel('Position (rad)');
title('Discrete saturated reference model');
legend({'Raw step', 'pos\_ref'}, 'Location', 'best');

nexttile;
yyaxis left;
plot(signals.vel_ref.time, signals.vel_ref.value, 'Color', [0.85 0.33 0.10], ...
    'LineWidth', 1.3);
ylabel('vel\_ref (rad/s)');
yyaxis right;
plot(signals.acc_unsat.time, signals.acc_unsat.value, '--', ...
    'Color', [0.60 0.60 0.60], 'LineWidth', 1.0); hold on;
plot(signals.acc_ref.time, signals.acc_ref.value, 'Color', [0.47 0.67 0.19], ...
    'LineWidth', 1.2);
ylabel('acc (rad/s^2)');
grid on;
title(sprintf('Acceleration saturation: %.1f%% after step', ...
    metrics.acc_saturation_ratio_pct));
legend({'vel\_ref', 'acc command before sat', 'acc\_ref after sat'}, ...
    'Location', 'best');

nexttile;
plot(signals.cmd_raw.time, signals.cmd_raw.value, 'Color', [0.70 0.70 0.70], ...
    'LineWidth', 1.0); hold on;
plot(signals.axis_raw.time, signals.axis_raw.value, 'r--', 'LineWidth', 1.1);
plot(signals.axis_mf_noff.time, signals.axis_mf_noff.value, ...
    'Color', [0.49 0.18 0.56], 'LineWidth', 1.1);
plot(signals.axis_mf_vff.time, signals.axis_mf_vff.value, 'k', 'LineWidth', 1.4);
grid on;
ylabel('Axis pos (rad)');
title(sprintf('Max MF tracking error: pos-only %.3g rad, +vFF %.3g rad', ...
    metrics.axis_mf_noff_max_ref_err, metrics.axis_mf_vff_max_ref_err));
legend({'Command', 'Raw step P', 'MF pos-only', 'MF + velocity FF'}, ...
    'Location', 'best');

nexttile;
plot(signals.speed_cmd_raw.time, signals.speed_cmd_raw.value, 'r--', 'LineWidth', 1.1);
hold on;
plot(signals.speed_cmd_mf_noff.time, signals.speed_cmd_mf_noff.value, ...
    'Color', [0.49 0.18 0.56], 'LineWidth', 1.1);
plot(signals.speed_cmd_mf_vff.time, signals.speed_cmd_mf_vff.value, 'k', 'LineWidth', 1.3);
grid on;
xlabel('Time (s)');
ylabel('Speed cmd (rad/s)');
title('Position-loop speed command with velocity feedforward');
legend({'Raw step P', 'MF pos-only', 'MF + velocity FF'}, 'Location', 'best');

exportgraphics(fig, file_name, 'Resolution', 160);
end

function project_root = local_find_project_root(anchor_path)
current = anchor_path;
while true
    has_config = exist(fullfile(current, 'motor_control_params.m'), 'file') == 2;
    has_algorithms = exist(fullfile(current, 'algorithms'), 'dir') == 7;
    has_build_modules = exist(fullfile(current, 'build_modules'), 'dir') == 7;
    if has_config && has_algorithms && has_build_modules
        project_root = current;
        return;
    end

    parent = fileparts(current);
    if strcmp(parent, current)
        error('Could not locate average-inverter project root from %s.', anchor_path);
    end
    current = parent;
end
end
