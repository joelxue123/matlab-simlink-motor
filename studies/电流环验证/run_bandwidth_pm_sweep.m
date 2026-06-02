function report = run_bandwidth_pm_sweep(cfg)
% Sweep current-loop target bandwidth and phase margin on the validation study.

if nargin < 1
    cfg = struct();
end

study_dir = fileparts(mfilename('fullpath'));
output_dir = fullfile(study_dir, 'outputs');
if exist(output_dir, 'dir') ~= 7
    mkdir(output_dir);
end

project_root = fileparts(fileparts(study_dir));
if exist(fullfile(project_root, 'init_project_paths.m'), 'file') == 2
    addpath(project_root);
end
addpath(study_dir);

init_project_paths(study_dir);
cfg = local_apply_defaults(cfg);

old_dir = pwd;
cleanup_dir = onCleanup(@() cd(old_dir));
cd(output_dir);

bandwidth_list = cfg.bandwidth_hz_list(:).';
phase_margin_list = cfg.phase_margin_deg_list(:).';
case_count = numel(bandwidth_list) * numel(phase_margin_list);
cases = [];

overshoot_grid = nan(numel(phase_margin_list), numel(bandwidth_list));
settling_grid = nan(numel(phase_margin_list), numel(bandwidth_list));
rmse_grid = nan(numel(phase_margin_list), numel(bandwidth_list));
voltage_grid = nan(numel(phase_margin_list), numel(bandwidth_list));

case_idx = 0;
for pm_idx = 1:numel(phase_margin_list)
    for bw_idx = 1:numel(bandwidth_list)
        run_cfg = cfg.base_cfg;
        run_cfg.current_bandwidth_hz = bandwidth_list(bw_idx);
        run_cfg.phase_margin_deg = phase_margin_list(pm_idx);
        run_cfg.tuning_method = 'bandwidth_pm';
        run_cfg.plot_results = false;
        run_cfg.save_outputs = false;

        case_idx = case_idx + 1;
        result = run_study(run_cfg);
        case_result = result.cases(1);
        case_result.phase_margin_deg = phase_margin_list(pm_idx);
        case_result.bandwidth_hz = bandwidth_list(bw_idx);
        if case_idx == 1
            cases = repmat(case_result, case_count, 1);
        end
        cases(case_idx) = case_result;

        overshoot_grid(pm_idx, bw_idx) = case_result.overshoot_a;
        settling_grid(pm_idx, bw_idx) = case_result.settling_time_s;
        rmse_grid(pm_idx, bw_idx) = case_result.rmse_a;
        voltage_grid(pm_idx, bw_idx) = case_result.voltage_utilization;
    end
end

summary = local_build_summary_table(cases);
best_overshoot = local_pick_best(cases, 'overshoot_a');
best_settling = local_pick_best(cases, 'settling_time_s');
best_rmse = local_pick_best(cases, 'rmse_a');

report = struct();
report.config = cfg;
report.output_dir = output_dir;
report.cases = cases;
report.summary = summary;
report.bandwidth_hz_list = bandwidth_list;
report.phase_margin_deg_list = phase_margin_list;
report.overshoot_grid = overshoot_grid;
report.settling_time_grid = settling_grid;
report.rmse_grid = rmse_grid;
report.voltage_utilization_grid = voltage_grid;
report.best_overshoot = best_overshoot;
report.best_settling = best_settling;
report.best_rmse = best_rmse;

if cfg.save_outputs
    writetable(summary, fullfile(output_dir, 'bandwidth_pm_sweep_summary.csv'));
    save(fullfile(output_dir, 'bandwidth_pm_sweep_report.mat'), 'report');
    local_save_heatmap(bandwidth_list, phase_margin_list, overshoot_grid, ...
        'Overshoot (A)', 'bandwidth_pm_sweep_overshoot.png');
    local_save_heatmap(bandwidth_list, phase_margin_list, settling_grid, ...
        'Settling Time (s)', 'bandwidth_pm_sweep_settling.png');
    local_save_heatmap(bandwidth_list, phase_margin_list, rmse_grid, ...
        'RMSE (A)', 'bandwidth_pm_sweep_rmse.png');
end

disp(summary);
fprintf('\nBest overshoot case: bw = %.3f Hz, PM = %.3f deg, overshoot = %.6f A\n', ...
    best_overshoot.bandwidth_hz, best_overshoot.phase_margin_deg, best_overshoot.overshoot_a);
fprintf('Best settling case: bw = %.3f Hz, PM = %.3f deg, settling = %.6f s\n', ...
    best_settling.bandwidth_hz, best_settling.phase_margin_deg, best_settling.settling_time_s);
fprintf('Best RMSE case: bw = %.3f Hz, PM = %.3f deg, rmse = %.6f A\n', ...
    best_rmse.bandwidth_hz, best_rmse.phase_margin_deg, best_rmse.rmse_a);

clear cleanup_dir
end

function cfg = local_apply_defaults(cfg)
defaults = struct();
defaults.bandwidth_hz_list = [400 600 800 1000 1200];
defaults.phase_margin_deg_list = [45 50 55 60 65 70];
defaults.save_outputs = true;
defaults.base_cfg = struct();

fields = fieldnames(defaults);
for idx = 1:numel(fields)
    name = fields{idx};
    if ~isfield(cfg, name) || isempty(cfg.(name))
        cfg.(name) = defaults.(name);
    end
end

cfg.bandwidth_hz_list = cfg.bandwidth_hz_list(:).';
cfg.phase_margin_deg_list = cfg.phase_margin_deg_list(:).';
end

function summary = local_build_summary_table(cases)
case_count = numel(cases);
case_name = cell(case_count, 1);
ref_axis = cell(case_count, 1);
ref_waveform = cell(case_count, 1);
bandwidth_hz = zeros(case_count, 1);
phase_margin_deg = zeros(case_count, 1);
kp = zeros(case_count, 1);
ki = zeros(case_count, 1);
rise_time_s = zeros(case_count, 1);
settling_time_s = zeros(case_count, 1);
overshoot_a = zeros(case_count, 1);
rmse_a = zeros(case_count, 1);
steady_state_error_a = zeros(case_count, 1);
max_abs_cross_axis_a = zeros(case_count, 1);
voltage_utilization = zeros(case_count, 1);

for idx = 1:case_count
    item = cases(idx);
    case_name{idx} = item.case_name;
    ref_axis{idx} = item.ref_axis;
    ref_waveform{idx} = item.ref_waveform;
    bandwidth_hz(idx) = item.bandwidth_hz;
    phase_margin_deg(idx) = item.phase_margin_deg;
    kp(idx) = item.kp;
    ki(idx) = item.ki;
    rise_time_s(idx) = item.rise_time_s;
    settling_time_s(idx) = item.settling_time_s;
    overshoot_a(idx) = item.overshoot_a;
    rmse_a(idx) = item.rmse_a;
    steady_state_error_a(idx) = item.steady_state_error_a;
    max_abs_cross_axis_a(idx) = item.max_abs_cross_axis_a;
    voltage_utilization(idx) = item.voltage_utilization;
end

summary = table(case_name, ref_axis, ref_waveform, bandwidth_hz, phase_margin_deg, ...
    kp, ki, rise_time_s, settling_time_s, overshoot_a, rmse_a, ...
    steady_state_error_a, max_abs_cross_axis_a, voltage_utilization);
end

function best_case = local_pick_best(cases, metric_name)
metric = nan(numel(cases), 1);
for idx = 1:numel(cases)
    metric(idx) = cases(idx).(metric_name);
end

valid_mask = isfinite(metric);
if ~any(valid_mask)
    best_case = cases(1);
    return;
end

[~, local_idx] = min(metric(valid_mask));
valid_indices = find(valid_mask);
best_case = cases(valid_indices(local_idx));
end

function local_save_heatmap(bandwidth_list, phase_margin_list, value_grid, colorbar_label, file_name)
fig = figure('Color', 'w');
imagesc(bandwidth_list, phase_margin_list, value_grid);
set(gca, 'YDir', 'normal');
grid on;
xlabel('Bandwidth (Hz)');
ylabel('Phase Margin (deg)');
title(strrep(file_name, '_', ' '));
cb = colorbar;
ylabel(cb, colorbar_label);
saveas(fig, file_name);
close(fig);
end