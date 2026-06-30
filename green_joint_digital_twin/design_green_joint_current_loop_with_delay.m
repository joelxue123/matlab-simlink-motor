%% Design green-joint current-loop PI with explicit delay and phase margin
%
% Mainline current-loop design entry for green-joint digital twin.
% It reads the firmware variant JSON contracts, computes the delay-aware
% bandwidth + phase-margin PI formula, and validates 1620 candidates with the
% same explicit PI and voltage saturation structure used by the MBD core.

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

repo_dir = fileparts(script_dir);
workspace_dir = fileparts(repo_dir);
firmware_dir = fullfile(workspace_dir, 'green-joint');

target_bandwidth_env = getenv('GJDT_CURRENT_TARGET_BW_HZ');
if isempty(target_bandwidth_env)
    cfg.target_bandwidth_hz = 2000.0;
else
    cfg.target_bandwidth_hz = str2double(target_bandwidth_env);
    if isnan(cfg.target_bandwidth_hz) || cfg.target_bandwidth_hz <= 0
        error('Invalid GJDT_CURRENT_TARGET_BW_HZ: %s', target_bandwidth_env);
    end
end
cfg.phase_margin_deg = 60.0;
cfg.delay_s_list = [50e-6 75e-6 100e-6];
cfg.default_design_delay_s = 75e-6;
cfg.ts_ctrl = 50e-6;
cfg.ts_plant = 1e-6;
cfg.stop_time = 0.010;
cfg.vbus = 12.0;
cfg.square_period = 0.001;
cfg.square_amplitude = 0.3;
cfg.step_time = 0.001;
cfg.step_final = 1.5;
cfg.measure_start = 0.002;

module_ids = ["1615", "1620"];
design_rows = [];
for module_id = module_ids
    module_cfg = load_green_joint_module_config(firmware_dir, module_id);
    for delay_s = cfg.delay_s_list
        design = design_delay_pm_pi( ...
            module_cfg.phase_resistance_ohm, ...
            module_cfg.phase_inductance_h, ...
            cfg.target_bandwidth_hz, ...
            cfg.phase_margin_deg, ...
            delay_s);
        design_rows = [design_rows; { ...
            char(module_id), ...
            module_cfg.phase_resistance_ohm, ...
            module_cfg.phase_inductance_h, ...
            cfg.target_bandwidth_hz, ...
            cfg.phase_margin_deg, ...
            delay_s * 1e6, ...
            design.feasible, ...
            design.pi_phase_lag_deg, ...
            design.kp, ...
            design.ki, ...
            design.pole_cancel_kp, ...
            design.pole_cancel_ki, ...
            design.pole_cancel_pm_deg}]; %#ok<AGROW>
    end
end

design_table = cell2table(design_rows, 'VariableNames', { ...
    'module_id', 'phase_resistance_ohm', 'phase_inductance_h', ...
    'target_bandwidth_hz', 'phase_margin_deg', 'delay_us', ...
    'feasible', 'pi_phase_lag_deg', 'kp_v_per_a', ...
    'ki_v_per_a_s', 'pole_cancel_kp_v_per_a', ...
    'pole_cancel_ki_v_per_a_s', 'pole_cancel_pm_deg'});

results_dir = fullfile(script_dir, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

target_bw_token = bandwidth_token(cfg.target_bandwidth_hz);
design_csv = fullfile(results_dir, ...
    ['green_joint_current_loop_' target_bw_token '_delay_pm_design.csv']);
writetable(design_table, design_csv);

module_1620 = load_green_joint_module_config(firmware_dir, "1620");
simcfg = build_sim_config(cfg, module_1620);
validation_table = validate_1620_candidates(cfg, simcfg, module_1620);
validation_csv = fullfile(results_dir, ...
    ['green_joint_1620_current_loop_' target_bw_token ...
    '_delay_pm_validation.csv']);
writetable(validation_table, validation_csv);

plot_file = fullfile(results_dir, ...
    ['green_joint_1620_current_loop_' target_bw_token ...
    '_delay_pm_validation.png']);
plot_1620_validation(simcfg, validation_table, plot_file);

fprintf('\nGreen-joint current-loop delay-aware PI design\n');
fprintf('  target bandwidth = %.9g Hz\n', cfg.target_bandwidth_hz);
fprintf('  target PM        = %.9g deg\n', cfg.phase_margin_deg);
fprintf('  default delay    = %.9g us\n', cfg.default_design_delay_s * 1e6);
fprintf('  design csv       = %s\n', design_csv);
fprintf('  validation csv   = %s\n', validation_csv);
fprintf('  validation plot  = %s\n\n', plot_file);

disp(design_table(:, {'module_id', 'target_bandwidth_hz', ...
    'phase_margin_deg', 'delay_us', 'kp_v_per_a', ...
    'ki_v_per_a_s', 'pole_cancel_pm_deg'}));

fprintf('\n1620 validation candidates:\n');
disp(validation_table(:, {'case_name', 'design_delay_us', 'sim_delay_us', ...
    'kp', 'ki', 'square_stable', 'square_rmse_a', ...
    'square_iq_peak_pos_a', 'square_iq_peak_neg_a', ...
    'square_voltage_norm_max', 'step_stable', ...
    'step_overshoot_ratio', 'step_settling_time_ms'}));

default_row = design_table(strcmp(design_table.module_id, '1620') ...
    & abs(design_table.delay_us - cfg.default_design_delay_s * 1e6) < 1e-9, :);
fprintf('\nRecommended 1620 %.9g Hz delay-aware design candidate:\n', ...
    cfg.target_bandwidth_hz);
fprintf('  Td = %.9g us, PM = %.9g deg\n', ...
    default_row.delay_us, default_row.phase_margin_deg);
fprintf('  Kp = %.9g V/A\n', default_row.kp_v_per_a);
fprintf('  Ki = %.9g V/(A*s)\n', default_row.ki_v_per_a_s);
fprintf('  Pole-cancel Kp=L*w/Ki=R*w would only have PM %.9g deg at this delay.\n', ...
    default_row.pole_cancel_pm_deg);

function simcfg = build_sim_config(cfg, module_cfg)
simcfg.ts_ctrl = cfg.ts_ctrl;
simcfg.ts_plant = cfg.ts_plant;
simcfg.stop_time = cfg.stop_time;
simcfg.vbus = cfg.vbus;
simcfg.rs = module_cfg.phase_resistance_ohm;
simcfg.lq = module_cfg.phase_inductance_h;
simcfg.voltage_limit = cfg.vbus ...
    * module_cfg.current_loop.voltage_limit_ratio ...
    * module_cfg.current_loop.voltage_modulation_ratio;
simcfg.pi_correction_gain = module_cfg.current_loop.pi_correction_gain;
simcfg.square_period = cfg.square_period;
simcfg.square_amplitude = cfg.square_amplitude;
simcfg.step_time = cfg.step_time;
simcfg.step_final = cfg.step_final;
simcfg.measure_start = cfg.measure_start;
simcfg.feedback_alpha = 0.95;
end

function table_out = validate_1620_candidates(cfg, simcfg, module_cfg)
wc_800 = 2 * pi * 800.0;
wc_target = 2 * pi * cfg.target_bandwidth_hz;
target_bw_token = bandwidth_token(cfg.target_bandwidth_hz);

cases = [ ...
    struct('name', "1625_borrowed_kp1_ki20000", ...
        'kp', 1.0, 'ki', 20000.0, 'design_delay_s', NaN, ...
        'note', "BANNED: 1625-derived tuning"); ...
    struct('name', "1620_800hz_current_contract", ...
        'kp', module_cfg.current_loop.cur_d_kp, ...
        'ki', module_cfg.current_loop.cur_d_ki, ...
        'design_delay_s', NaN, ...
        'note', "Current 1620 variant contract"); ...
    struct('name', "1620_800hz_pole_cancel", ...
        'kp', module_cfg.phase_inductance_h * wc_800, ...
        'ki', module_cfg.phase_resistance_ohm * wc_800, ...
        'design_delay_s', NaN, ...
        'note', "800 Hz no-delay pole-cancel reference"); ...
    struct('name', string(['1620_' target_bw_token '_pole_cancel']), ...
        'kp', module_cfg.phase_inductance_h * wc_target, ...
        'ki', module_cfg.phase_resistance_ohm * wc_target, ...
        'design_delay_s', 0.0, ...
        'note', string([target_bw_token ' no-delay pole-cancel reference']))];

for delay_s = cfg.delay_s_list
    design = design_delay_pm_pi(module_cfg.phase_resistance_ohm, ...
        module_cfg.phase_inductance_h, cfg.target_bandwidth_hz, ...
        cfg.phase_margin_deg, delay_s);
    cases = [cases; struct('name', ...
        string(sprintf("1620_%s_pm60_td%03dus", target_bw_token, ...
        round(delay_s * 1e6))), ...
        'kp', design.kp, 'ki', design.ki, ...
        'design_delay_s', delay_s, ...
        'note', string([target_bw_token ...
        ' delay-aware bandwidth + PM design']))]; %#ok<AGROW>
end

rows = [];
sim_delay_list = cfg.delay_s_list;
for i = 1:numel(cases)
    for sim_delay_s = sim_delay_list
        square_result = simulate_square(simcfg, cases(i), sim_delay_s);
        step_result = simulate_step(simcfg, cases(i), sim_delay_s);
        rows = [rows; { ...
            char(cases(i).name), ...
            delay_to_us(cases(i).design_delay_s), ...
            sim_delay_s * 1e6, ...
            cases(i).kp, cases(i).ki, ...
            square_result.stable, square_result.rmse_a, ...
            square_result.max_abs_error_a, square_result.iq_peak_pos_a, ...
            square_result.iq_peak_neg_a, square_result.voltage_norm_max, ...
            step_result.stable, step_result.overshoot_ratio, ...
            step_result.settling_time_ms, step_result.final_iq_a, ...
            char(cases(i).note)}]; %#ok<AGROW>
    end
end

table_out = cell2table(rows, 'VariableNames', { ...
    'case_name', 'design_delay_us', 'sim_delay_us', 'kp', 'ki', ...
    'square_stable', 'square_rmse_a', 'square_max_abs_error_a', ...
    'square_iq_peak_pos_a', 'square_iq_peak_neg_a', ...
    'square_voltage_norm_max', 'step_stable', 'step_overshoot_ratio', ...
    'step_settling_time_ms', 'step_final_iq_a', 'note'});
end

function value_us = delay_to_us(delay_s)
if isnan(delay_s)
    value_us = NaN;
else
    value_us = delay_s * 1e6;
end
end

function result = simulate_square(cfg, case_cfg, feedback_delay_s)
t = (0:cfg.ts_plant:cfg.stop_time)';
ref = square_ref(t, cfg.square_period, cfg.square_amplitude);
[iq, vq] = simulate_current_loop(cfg, case_cfg.kp, case_cfg.ki, ref, ...
    feedback_delay_s);

measure = t >= cfg.measure_start;
err = ref(measure) - iq(measure);
result.rmse_a = sqrt(mean(err.^2));
result.max_abs_error_a = max(abs(err));
result.iq_peak_pos_a = max(iq(measure));
result.iq_peak_neg_a = min(iq(measure));
result.voltage_norm_max = max(abs(vq)) / cfg.voltage_limit;
result.stable = result.voltage_norm_max < 0.95 ...
    && result.iq_peak_pos_a < 1.0 ...
    && result.iq_peak_neg_a > -1.0;
end

function result = simulate_step(cfg, case_cfg, feedback_delay_s)
t = (0:cfg.ts_plant:0.006)';
ref = zeros(size(t));
ref(t >= cfg.step_time) = cfg.step_final;
[iq, vq] = simulate_current_loop(cfg, case_cfg.kp, case_cfg.ki, ref, ...
    feedback_delay_s);

post = t >= cfg.step_time;
result.overshoot_ratio = (max(iq(post)) - cfg.step_final) / cfg.step_final;
result.final_iq_a = iq(end);
result.voltage_norm_max = max(abs(vq)) / cfg.voltage_limit;
result.settling_time_ms = first_settle_time_ms(t, iq, cfg.step_final, ...
    cfg.step_time, 0.02 * cfg.step_final);
result.stable = result.voltage_norm_max < 0.95 ...
    && result.overshoot_ratio < 0.5 ...
    && ~isnan(result.settling_time_ms) ...
    && abs(result.final_iq_a - cfg.step_final) < 0.05;
end

function [iq_fbk, vq] = simulate_current_loop(cfg, kp, ki, ref, feedback_delay_s)
t = (0:cfg.ts_plant:(numel(ref) - 1) * cfg.ts_plant)';
iq = zeros(size(t));
iq_fbk = zeros(size(t));
vq = zeros(size(t));
integrator = 0.0;
last_vq = 0.0;

ctrl_steps = round(cfg.ts_ctrl / cfg.ts_plant);
feedback_delay_samples = round(feedback_delay_s / cfg.ts_plant);
for k = 2:numel(t)
    if mod(k - 2, ctrl_steps) == 0
        feedback_index = max(1, k - 1 - feedback_delay_samples);
        err = ref(k - 1) - iq_fbk(feedback_index);
        pre_sat = kp * err + integrator;
        cmd = min(max(pre_sat, -cfg.voltage_limit), cfg.voltage_limit);
        integrator = integrator + ...
            (ki * err + cfg.pi_correction_gain * (cmd - pre_sat)) ...
            * cfg.ts_ctrl;
        last_vq = cmd;
    end

    vq(k) = last_vq;
    iq(k) = iq(k - 1) ...
        + cfg.ts_plant * (vq(k) - cfg.rs * iq(k - 1)) / cfg.lq;
    iq_fbk(k) = iq_fbk(k - 1) ...
        + cfg.feedback_alpha * (iq(k) - iq_fbk(k - 1));
end
end

function ref = square_ref(t, period, amplitude)
phase = mod(t, period);
ref = -amplitude * ones(size(t));
ref(phase >= period / 2) = amplitude;
end

function settle_ms = first_settle_time_ms(t, y, target, step_time, band)
settle_ms = NaN;
start_index = find(t >= step_time, 1, 'first');
for i = start_index:numel(t)
    if all(abs(y(i:end) - target) <= band)
        settle_ms = (t(i) - step_time) * 1e3;
        return;
    end
end
end

function plot_1620_validation(simcfg, validation_table, plot_file)
target_bw_token = infer_target_bw_token(validation_table);
plot_names = ["1620_800hz_current_contract", ...
    string(['1620_' target_bw_token '_pole_cancel']), ...
    string(['1620_' target_bw_token '_pm60_td075us'])];
sim_delay_s = 75e-6;
t = (0:simcfg.ts_plant:simcfg.stop_time)';
ref = square_ref(t, simcfg.square_period, simcfg.square_amplitude);

figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1200 760]);
subplot(2, 1, 1);
plot(t * 1e3, ref, 'k-', 'LineWidth', 1.0);
hold on;
legend_entries = {'Iq ref'};
for i = 1:numel(plot_names)
    row = validation_table(strcmp(validation_table.case_name, plot_names(i)) ...
        & abs(validation_table.sim_delay_us - sim_delay_s * 1e6) < 1e-9, :);
    if isempty(row)
        continue;
    end
    case_cfg.kp = row.kp(1);
    case_cfg.ki = row.ki(1);
    [iq, ~] = simulate_current_loop(simcfg, case_cfg.kp, case_cfg.ki, ref, ...
        sim_delay_s);
    plot(t * 1e3, iq, 'LineWidth', 1.2);
    legend_entries{end + 1} = char(plot_names(i)); %#ok<AGROW>
end
grid on;
xlabel('Time (ms)');
ylabel('Iq feedback (A)');
title(sprintf('1620 current-loop candidates, 75 us simulated delay, 1 kHz square'));
legend(legend_entries, 'Interpreter', 'none', 'Location', 'best');
xlim([2 simcfg.stop_time * 1e3]);

subplot(2, 1, 2);
hold on;
for i = 1:numel(plot_names)
    row = validation_table(strcmp(validation_table.case_name, plot_names(i)) ...
        & abs(validation_table.sim_delay_us - sim_delay_s * 1e6) < 1e-9, :);
    if isempty(row)
        continue;
    end
    case_cfg.kp = row.kp(1);
    case_cfg.ki = row.ki(1);
    [~, vq] = simulate_current_loop(simcfg, case_cfg.kp, case_cfg.ki, ref, ...
        sim_delay_s);
    plot(t * 1e3, vq, 'LineWidth', 1.2);
end
yline(simcfg.voltage_limit, 'k--');
yline(-simcfg.voltage_limit, 'k--');
grid on;
xlabel('Time (ms)');
ylabel('Vq command (V)');
title('Voltage command');
legend([cellstr(plot_names), {'Limit'}], 'Interpreter', 'none', ...
    'Location', 'best');
xlim([2 simcfg.stop_time * 1e3]);

exportgraphics(gcf, plot_file, 'Resolution', 160);
close(gcf);
end

function token = bandwidth_token(bandwidth_hz)
if abs(bandwidth_hz - round(bandwidth_hz)) < 1e-9
    token = sprintf('%dhz', round(bandwidth_hz));
else
    token = sprintf('%.3fhz', bandwidth_hz);
    token = regexprep(token, '0+hz$', 'hz');
    token = regexprep(token, '\.hz$', 'hz');
    token = strrep(token, '.', 'p');
end
end

function token = infer_target_bw_token(validation_table)
case_names = string(validation_table.case_name);
matches = regexp(case_names, '^1620_(\d+(?:p\d+)?hz)_pm60_td075us$', ...
    'tokens', 'once');
token = '';
for i = 1:numel(matches)
    if ~isempty(matches{i})
        token = char(matches{i}{1});
        return;
    end
end
error('Could not infer target bandwidth token from validation table.');
end

function design = design_delay_pm_pi(resistance_ohm, inductance_h, ...
    bandwidth_hz, phase_margin_deg, delay_s)
omega_c = 2 * pi * bandwidth_hz;
plant_phase_lag_rad = atan2(omega_c * inductance_h, resistance_ohm);
delay_phase_lag_rad = omega_c * delay_s;
pi_phase_lag_rad = pi - deg2rad(phase_margin_deg) ...
    - plant_phase_lag_rad - delay_phase_lag_rad;

design.feasible = pi_phase_lag_rad > 0 && pi_phase_lag_rad < (pi / 2);
if ~design.feasible
    design.kp = NaN;
    design.ki = NaN;
else
    plant_den_mag = sqrt(resistance_ohm^2 + (omega_c * inductance_h)^2);
    design.kp = plant_den_mag * cos(pi_phase_lag_rad);
    design.ki = design.kp * omega_c * tan(pi_phase_lag_rad);
end

design.pi_phase_lag_deg = rad2deg(pi_phase_lag_rad);
design.pole_cancel_kp = inductance_h * omega_c;
design.pole_cancel_ki = resistance_ohm * omega_c;
design.pole_cancel_pm_deg = 90.0 - rad2deg(delay_phase_lag_rad);
end

function cfg = load_green_joint_module_config(fw_dir, motor_type)
config_file = fullfile(fw_dir, 'Module', 'Config', ...
    ['green_joint_' char(motor_type) '_config.json']);
if ~exist(config_file, 'file')
    error('Missing green-joint module config: %s', config_file);
end

cfg = jsondecode(fileread(config_file));

expected_phase_r = cfg.line_to_line_resistance_ohm / 2.0;
expected_phase_l = cfg.line_to_line_inductance_h / 2.0;
if abs(cfg.phase_resistance_ohm - expected_phase_r) > 1e-9
    error('Invalid phase resistance in %s.', config_file);
end
if abs(cfg.phase_inductance_h - expected_phase_l) > 1e-12
    error('Invalid phase inductance in %s.', config_file);
end
end
