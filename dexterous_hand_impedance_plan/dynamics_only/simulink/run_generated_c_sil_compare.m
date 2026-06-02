%% Compare Simulink controller behavior with generated C behavior
% This is a host-side generated-C-in-the-loop check.
%
% It compiles Controller_ert_rtw/Controller.c into a MEX function and feeds
% the same fixed-point raw input sequence into:
%   1. single_joint_controller_only.slx
%   2. generated C code through controller_ert_mex
%
% Pass condition:
%   generated C output and Simulink output differ by no more than 1 raw LSB.

clear; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
root_dir = fileparts(script_dir);
matlab_dir = fullfile(root_dir, 'matlab');
results_dir = fullfile(root_dir, 'results', 'sil');
sil_dir = fullfile(script_dir, 'sil_generated_c');
ert_dir = fullfile(script_dir, 'Controller_ert_rtw');

addpath(matlab_dir);
addpath(sil_dir);

if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

cfg = config_default();
init_controller_fixed_point_types(cfg);

model = 'single_joint_controller_only';
model_file = fullfile(script_dir, [model '.slx']);
if ~exist(model_file, 'file')
    error('Missing %s. Run build_controller_only_model.m first.', model_file);
end
if ~exist(fullfile(ert_dir, 'Controller.c'), 'file')
    error('Missing generated C code in %s. Generate ERT code first.', ert_dir);
end

build_controller_mex(sil_dir, ert_dir);

N = 800;
Ts = cfg.sim.Ts;
t = (0:N-1)' * Ts;
raw_input = make_test_vectors(N);
input_dataset = make_external_input_dataset(raw_input, t);

load_system(model_file);
simIn = Simulink.SimulationInput(model);
simIn = simIn.setVariable('input_dataset', input_dataset);
simIn = simIn.setModelParameter( ...
    'LoadExternalInput', 'on', ...
    'ExternalInput', 'input_dataset', ...
    'StopTime', num2str(t(end), 17), ...
    'SaveOutput', 'on', ...
    'OutputSaveName', 'yout', ...
    'SaveFormat', 'Dataset');

simOut = sim(simIn);
yout = simOut.yout;
[sim_tau_cmd_raw, sim_tau_load_hat_raw] = get_simulink_raw_outputs(yout);

c_raw_output = controller_ert_mex(double(raw_input));
c_tau_cmd_raw = int16(c_raw_output(:, 1));
c_tau_load_hat_raw = int16(c_raw_output(:, 2));

diff_tau_cmd = double(c_tau_cmd_raw) - double(sim_tau_cmd_raw);
diff_tau_load_hat = double(c_tau_load_hat_raw) - double(sim_tau_load_hat_raw);

metrics.max_abs_tau_cmd_lsb = max(abs(diff_tau_cmd));
metrics.max_abs_tau_load_hat_lsb = max(abs(diff_tau_load_hat));
metrics.num_tau_cmd_mismatch = nnz(abs(diff_tau_cmd) > 1);
metrics.num_tau_load_hat_mismatch = nnz(abs(diff_tau_load_hat) > 1);
metrics.pass = metrics.num_tau_cmd_mismatch == 0 && metrics.num_tau_load_hat_mismatch == 0;

save(fullfile(results_dir, 'generated_c_sil_compare.mat'), ...
    'cfg', 't', 'raw_input', 'sim_tau_cmd_raw', 'sim_tau_load_hat_raw', ...
    'c_tau_cmd_raw', 'c_tau_load_hat_raw', 'diff_tau_cmd', ...
    'diff_tau_load_hat', 'metrics');

write_compare_csv(fullfile(results_dir, 'generated_c_sil_compare.csv'), ...
    t, raw_input, sim_tau_cmd_raw, sim_tau_load_hat_raw, ...
    c_tau_cmd_raw, c_tau_load_hat_raw, diff_tau_cmd, diff_tau_load_hat);

plot_compare(fullfile(results_dir, 'generated_c_sil_compare.png'), ...
    t, sim_tau_cmd_raw, c_tau_cmd_raw, diff_tau_cmd, ...
    sim_tau_load_hat_raw, c_tau_load_hat_raw, diff_tau_load_hat);

fprintf('Generated C SIL compare\n');
fprintf('  max_abs_tau_cmd_lsb      = %.0f\n', metrics.max_abs_tau_cmd_lsb);
fprintf('  max_abs_tau_load_hat_lsb = %.0f\n', metrics.max_abs_tau_load_hat_lsb);
fprintf('  tau_cmd mismatches >1 LSB      = %d\n', metrics.num_tau_cmd_mismatch);
fprintf('  tau_load_hat mismatches >1 LSB = %d\n', metrics.num_tau_load_hat_mismatch);
if metrics.pass
    fprintf('  PASS\n');
else
    fprintf('  FAIL\n');
    error('Generated C SIL compare failed. See %s', results_dir);
end

%% Local functions
function build_controller_mex(sil_dir, ert_dir)
    mex_name = ['controller_ert_mex.' mexext];
    mex_file = fullfile(sil_dir, mex_name);
    src_file = fullfile(sil_dir, 'controller_ert_mex.c');
    controller_c = fullfile(ert_dir, 'Controller.c');

    rebuild = ~exist(mex_file, 'file') || ...
        dir(src_file).datenum > dir(mex_file).datenum || ...
        dir(controller_c).datenum > dir(mex_file).datenum;

    if rebuild
        fprintf('Building %s\n', mex_file);
        mex('-R2018a', ...
            ['-I' ert_dir], ...
            '-outdir', sil_dir, ...
            src_file, controller_c);
    end
end

function raw_input = make_test_vectors(N)
    rng(7);
    raw_input = zeros(N, 6, 'int16');

    k = (1:N)';
    raw_input(:, 1) = int16(1200 * sin(2*pi*k/180) + 800 * (k > N/4));
    raw_input(:, 2) = int16(900 * sin(2*pi*k/230));
    raw_input(:, 3) = int16(250 * sin(2*pi*k/75));
    raw_input(:, 4) = int16(80 * sin(2*pi*k/51));
    raw_input(:, 5) = int16(300 * sin(2*pi*k/97));

    mode = ones(N, 1, 'uint8');
    mode(k > N/3) = uint8(2);
    mode(k > 2*N/3) = uint8(3);
    raw_input(:, 6) = int16(mode);

    noise = int16(randi([-25, 25], N, 5));
    raw_input(:, 1:5) = raw_input(:, 1:5) + noise;

    raw_input(50:55, 1) = int16(6000);
    raw_input(250:255, 2) = int16(-5000);
    raw_input(450:455, 5) = int16(4000);
end

function input_dataset = make_external_input_dataset(raw_input, t)
    input_dataset = Simulink.SimulationData.Dataset();

    input_dataset = add_signal(input_dataset, 'q_ref', ...
        fi(double(raw_input(:, 1)) / 2^12, 1, 16, 12), t);
    input_dataset = add_signal(input_dataset, 'q', ...
        fi(double(raw_input(:, 2)) / 2^12, 1, 16, 12), t);
    input_dataset = add_signal(input_dataset, 'qdot', ...
        fi(double(raw_input(:, 3)) / 2^8, 1, 16, 8), t);
    input_dataset = add_signal(input_dataset, 'qddot', ...
        fi(double(raw_input(:, 4)) / 2^4, 1, 16, 4), t);
    input_dataset = add_signal(input_dataset, 'tau_prev', ...
        fi(double(raw_input(:, 5)) / 2^12, 1, 16, 12), t);
    input_dataset = add_signal(input_dataset, 'mode', ...
        uint8(raw_input(:, 6)), t);
end

function dataset = add_signal(dataset, name, values, t)
    signal = Simulink.SimulationData.Signal();
    signal.Name = name;
    signal.Values = timeseries(values, t);
    dataset = dataset.addElement(signal);
end

function [tau_cmd_raw, tau_load_hat_raw] = get_simulink_raw_outputs(yout)
    tau_cmd_ts = get_dataset_signal(yout, 'tau_cmd', 1);
    tau_load_hat_ts = get_dataset_signal(yout, 'tau_load_hat', 2);

    tau_cmd_raw = int16(round(double(tau_cmd_ts.Data(:)) * 2^13));
    tau_load_hat_raw = int16(round(double(tau_load_hat_ts.Data(:)) * 2^13));
end

function ts = get_dataset_signal(dataset, name, index)
    sig = [];
    for k = 1:dataset.numElements
        candidate = dataset{k};
        if strcmp(candidate.Name, name)
            sig = candidate;
            break;
        end
    end

    if isempty(sig)
        sig = dataset{index};
    end

    ts = sig.Values;
end

function write_compare_csv(filename, t, raw_input, sim_tau_cmd_raw, ...
    sim_tau_load_hat_raw, c_tau_cmd_raw, c_tau_load_hat_raw, ...
    diff_tau_cmd, diff_tau_load_hat)

    header = ['t,q_ref_raw,q_raw,qdot_raw,qddot_raw,tau_prev_raw,mode,' ...
        'sim_tau_cmd_raw,c_tau_cmd_raw,diff_tau_cmd,' ...
        'sim_tau_load_hat_raw,c_tau_load_hat_raw,diff_tau_load_hat'];

    data = [t, double(raw_input), ...
        double(sim_tau_cmd_raw), double(c_tau_cmd_raw), diff_tau_cmd, ...
        double(sim_tau_load_hat_raw), double(c_tau_load_hat_raw), diff_tau_load_hat];

    fid = fopen(filename, 'w');
    if fid < 0
        error('Cannot open %s', filename);
    end
    fprintf(fid, '%s\n', header);
    fclose(fid);
    dlmwrite(filename, data, '-append');
end

function plot_compare(filename, t, sim_tau_cmd_raw, c_tau_cmd_raw, ...
    diff_tau_cmd, sim_tau_load_hat_raw, c_tau_load_hat_raw, diff_tau_load_hat)

    fig = figure('Name', 'Generated C SIL compare', 'Color', 'w');
    tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    plot(t*1000, double(sim_tau_cmd_raw), 'b', 'LineWidth', 1.1); hold on;
    plot(t*1000, double(c_tau_cmd_raw), 'r--', 'LineWidth', 1.0);
    grid on;
    ylabel('tau cmd raw');
    legend('Simulink', 'Generated C', 'Location', 'best');

    nexttile;
    plot(t*1000, double(sim_tau_load_hat_raw), 'b', 'LineWidth', 1.1); hold on;
    plot(t*1000, double(c_tau_load_hat_raw), 'r--', 'LineWidth', 1.0);
    grid on;
    ylabel('load hat raw');

    nexttile;
    plot(t*1000, diff_tau_cmd, 'k', 'LineWidth', 1.0); hold on;
    plot(t*1000, diff_tau_load_hat, 'Color', [0.8 0.25 0.1], 'LineWidth', 1.0);
    grid on;
    xlabel('time (ms)');
    ylabel('diff raw LSB');
    legend('tau cmd diff', 'load hat diff', 'Location', 'best');

    exportgraphics(fig, filename, 'Resolution', 160);
end
