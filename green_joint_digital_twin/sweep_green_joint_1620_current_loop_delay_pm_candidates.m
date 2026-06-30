%% Sweep green-joint delay-aware current-loop PI candidates
%
% Fast numerical sweep before running heavier V1 Simulink tests. This keeps
% the same explicit PI, voltage saturation, and back-calculation structure as
% the green-joint current-loop MBD core.

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

repo_dir = fileparts(script_dir);
workspace_dir = fileparts(repo_dir);
firmware_dir = fullfile(workspace_dir, 'green-joint');
module_id = getenv('GJDT_MOTOR_TYPE');
if isempty(module_id)
    module_id = '1620';
end
module_cfg = load_green_joint_module_config(firmware_dir, module_id);

cfg.bandwidth_hz_list = [1200 1500];
cfg.phase_margin_deg_list = [60 65 70];
cfg.design_delay_s = 75e-6;
cfg.sim_delay_s_list = [50e-6 75e-6 100e-6];
cfg.ts_ctrl = 50e-6;
cfg.ts_plant = 1e-6;
cfg.stop_time = 0.010;
cfg.vbus = 12.0;
cfg.rs = module_cfg.phase_resistance_ohm;
cfg.lq = module_cfg.phase_inductance_h;
cfg.voltage_limit = cfg.vbus ...
    * module_cfg.current_loop.voltage_limit_ratio ...
    * module_cfg.current_loop.voltage_modulation_ratio;
cfg.pi_correction_gain = module_cfg.current_loop.pi_correction_gain;
cfg.square_period = 0.001;
cfg.square_amplitude = 0.3;
cfg.step_time = 0.001;
cfg.step_final = 1.5;
cfg.measure_start = 0.002;
cfg.feedback_alpha = 0.95;

rows = [];
for bandwidth_hz = cfg.bandwidth_hz_list
    for phase_margin_deg = cfg.phase_margin_deg_list
        design = design_delay_pm_pi(cfg.rs, cfg.lq, bandwidth_hz, ...
            phase_margin_deg, cfg.design_delay_s);
        for sim_delay_s = cfg.sim_delay_s_list
            square_result = simulate_square(cfg, design.kp, design.ki, ...
                sim_delay_s);
            step_result = simulate_step(cfg, design.kp, design.ki, ...
                sim_delay_s);
            score = score_candidate(square_result, step_result, sim_delay_s);
            rows = [rows; { ...
                sprintf('%s_%dhz_pm%d_td075us', module_id, ...
                bandwidth_hz, phase_margin_deg), ...
                bandwidth_hz, phase_margin_deg, cfg.design_delay_s * 1e6, ...
                sim_delay_s * 1e6, design.kp, design.ki, ...
                design.pi_phase_lag_deg, design.pole_cancel_pm_deg, ...
                square_result.stable, square_result.rmse_a, ...
                square_result.max_abs_error_a, square_result.iq_peak_pos_a, ...
                square_result.iq_peak_neg_a, square_result.voltage_norm_max, ...
                step_result.stable, step_result.overshoot_ratio, ...
                step_result.settling_time_ms, step_result.final_iq_a, ...
                score}]; %#ok<AGROW>
        end
    end
end

results = cell2table(rows, 'VariableNames', { ...
    'case_name', 'bandwidth_hz', 'phase_margin_deg', 'design_delay_us', ...
    'sim_delay_us', 'kp', 'ki', 'pi_phase_lag_deg', ...
    'pole_cancel_pm_deg', 'square_stable', 'square_rmse_a', ...
    'square_max_abs_error_a', 'square_iq_peak_pos_a', ...
    'square_iq_peak_neg_a', 'square_voltage_norm_max', ...
    'step_stable', 'step_overshoot_ratio', 'step_settling_time_ms', ...
    'step_final_iq_a', 'score'});

results_dir = fullfile(script_dir, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end
csv_file = fullfile(results_dir, ...
    ['green_joint_' module_id ...
    '_current_loop_delay_pm_sweep_1200_1500.csv']);
writetable(results, csv_file);

nominal = results(results.sim_delay_us == 75, :);
nominal = sortrows(nominal, 'score', 'ascend');

robust = summarize_robustness(results);
robust = sortrows(robust, 'score_75us', 'ascend');
robust_csv = fullfile(results_dir, ...
    ['green_joint_' module_id ...
    '_current_loop_delay_pm_sweep_robust_summary.csv']);
writetable(robust, robust_csv);

fprintf('\nGreen-joint %s current-loop delay/PM sweep\n', module_id);
fprintf('  bandwidths     = %s Hz\n', mat2str(cfg.bandwidth_hz_list));
fprintf('  phase margins  = %s deg\n', mat2str(cfg.phase_margin_deg_list));
fprintf('  design delay   = %.6g us\n', cfg.design_delay_s * 1e6);
fprintf('  sim delays     = %s us\n', mat2str(cfg.sim_delay_s_list * 1e6));
fprintf('  csv            = %s\n', csv_file);
fprintf('  robust summary = %s\n\n', robust_csv);

fprintf('Nominal 75us ranking:\n');
disp(nominal(:, {'case_name', 'kp', 'ki', 'square_rmse_a', ...
    'square_iq_peak_pos_a', 'square_iq_peak_neg_a', ...
    'step_overshoot_ratio', 'step_settling_time_ms', 'score'}));

fprintf('\nRobust summary across 50/75/100us simulated delay:\n');
disp(robust(:, {'case_name', 'kp', 'ki', 'square_stable_all', ...
    'step_stable_all', 'worst_square_peak_abs_a', ...
    'worst_step_overshoot_ratio', 'score_75us'}));

function score = score_candidate(square_result, step_result, sim_delay_s)
score = square_result.rmse_a ...
    + 0.20 * max(0, abs(square_result.iq_peak_pos_a) - 0.40) ...
    + 0.20 * max(0, abs(square_result.iq_peak_neg_a) - 0.40) ...
    + 0.25 * max(0, step_result.overshoot_ratio - 0.25) ...
    + 0.0005 * max(0, step_result.settling_time_ms - 0.80) ...
    + 0.02 * abs(sim_delay_s - 75e-6) / 25e-6;
if ~square_result.stable
    score = score + 10;
end
if ~step_result.stable
    score = score + 5;
end
end

function summary = summarize_robustness(results)
case_names = unique(string(results.case_name), 'stable');
rows = [];
for i = 1:numel(case_names)
    item = results(string(results.case_name) == case_names(i), :);
    nominal = item(item.sim_delay_us == 75, :);
    rows = [rows; { ...
        char(case_names(i)), nominal.kp(1), nominal.ki(1), ...
        all(item.square_stable), all(item.step_stable), ...
        max(max(abs(item.square_iq_peak_pos_a)), ...
        max(abs(item.square_iq_peak_neg_a))), ...
        max(item.step_overshoot_ratio), nominal.score(1)}]; %#ok<AGROW>
end

summary = cell2table(rows, 'VariableNames', { ...
    'case_name', 'kp', 'ki', 'square_stable_all', 'step_stable_all', ...
    'worst_square_peak_abs_a', 'worst_step_overshoot_ratio', ...
    'score_75us'});
end

function result = simulate_square(cfg, kp, ki, feedback_delay_s)
t = (0:cfg.ts_plant:cfg.stop_time)';
ref = square_ref(t, cfg.square_period, cfg.square_amplitude);
[iq, vq] = simulate_current_loop(cfg, kp, ki, ref, feedback_delay_s);

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

function result = simulate_step(cfg, kp, ki, feedback_delay_s)
t = (0:cfg.ts_plant:0.006)';
ref = zeros(size(t));
ref(t >= cfg.step_time) = cfg.step_final;
[iq, vq] = simulate_current_loop(cfg, kp, ki, ref, feedback_delay_s);

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

function design = design_delay_pm_pi(resistance_ohm, inductance_h, ...
    bandwidth_hz, phase_margin_deg, delay_s)
omega_c = 2 * pi * bandwidth_hz;
plant_phase_lag_rad = atan2(omega_c * inductance_h, resistance_ohm);
delay_phase_lag_rad = omega_c * delay_s;
pi_phase_lag_rad = pi - deg2rad(phase_margin_deg) ...
    - plant_phase_lag_rad - delay_phase_lag_rad;

if pi_phase_lag_rad <= 0 || pi_phase_lag_rad >= (pi / 2)
    error('Infeasible PI design: %.6g Hz, PM %.6g, Td %.6g us.', ...
        bandwidth_hz, phase_margin_deg, delay_s * 1e6);
end

plant_den_mag = sqrt(resistance_ohm^2 + (omega_c * inductance_h)^2);
design.kp = plant_den_mag * cos(pi_phase_lag_rad);
design.ki = design.kp * omega_c * tan(pi_phase_lag_rad);
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
