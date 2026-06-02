%% Single joint dynamics control lab
% Tests:
%   1. position_step: q_ref step, no external load.
%   2. load_step: q_ref step, then external load torque step.
%   3. delay_scan: position step under 0..N sample command delay.

clear; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
root_dir = fileparts(script_dir);
results_dir = fullfile(root_dir, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

addpath(script_dir);

cfg = config_default();

fprintf('Single joint dynamics control lab\n');
fprintf('Plant: J = %.4f kg*m^2, b = %.4f N*m*s/rad, Ts = %.4f s\n\n', ...
    cfg.plant.J, cfg.plant.b, cfg.sim.Ts);

%% Test 1: position step without external load
position_results = struct();
for i = 1:numel(cfg.controllers)
    name = cfg.controllers{i};
    out = simulate_single_joint(cfg, name, 'position_step', 0);
    metrics = compute_metrics(out, cfg);
    position_results.(name).out = out;
    position_results.(name).metrics = metrics;
    print_metrics(sprintf('position_step/%s', name), metrics);
end
plot_comparison(position_results, cfg.controllers, 'position_step', results_dir);

%% Test 2: load step during motion/holding
load_results = struct();
for i = 1:numel(cfg.controllers)
    name = cfg.controllers{i};
    out = simulate_single_joint(cfg, name, 'load_step', 0);
    metrics = compute_metrics(out, cfg);
    load_results.(name).out = out;
    load_results.(name).metrics = metrics;
    print_metrics(sprintf('load_step/%s', name), metrics);
end
plot_comparison(load_results, cfg.controllers, 'load_step', results_dir);

%% Test 3: delay scan
delay_rows = [];
for i = 1:numel(cfg.controllers)
    name = cfg.controllers{i};
    for d = cfg.delay_scan_samples
        out = simulate_single_joint(cfg, name, 'position_step', d);
        metrics = compute_metrics(out, cfg);
        delay_rows = [delay_rows; make_delay_row(name, d, metrics)]; %#ok<AGROW>
    end
end

delay_table = struct2table(delay_rows);
disp('Delay scan summary:');
disp(delay_table);

writetable(delay_table, fullfile(results_dir, 'delay_scan_metrics.csv'));
plot_delay_scan(delay_table, results_dir);

save(fullfile(results_dir, 'single_joint_dynamics_study.mat'), ...
    'cfg', 'position_results', 'load_results', 'delay_table');

fprintf('\nSaved results to:\n  %s\n', results_dir);

function row = make_delay_row(controller, delay_samples, metrics)
    row = struct();
    row.controller = string(controller);
    row.delay_samples = delay_samples;
    row.delay_ms = delay_samples;
    row.overshoot_pct = metrics.overshoot_pct;
    row.settling_time = metrics.settling_time;
    row.e_rms = metrics.e_rms;
    row.e_peak = metrics.e_peak;
    row.tau_rms = metrics.tau_rms;
    row.tau_peak = metrics.tau_peak;
    row.I_rms = metrics.I_rms;
    row.heat_energy = metrics.heat_energy;
    row.stable = metrics.stable;
end
