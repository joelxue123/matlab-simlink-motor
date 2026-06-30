%% V1-equivalent high-speed voltage-limit test for the green-joint speed loop
%
% Purpose:
%   Reproduce the field symptom where the speed loop has saturated near 4 A,
%   the motor is already at the high-speed protection region, and the actual
%   q-axis current is small because the available Vq is consumed by back EMF.
%
% This script mirrors the V1 average-voltage path in equations:
%   Speed PI -> Current PI with circular voltage limit -> dq PMSM plant.
%
% It intentionally holds speed feedback on a high-speed protection plateau so
% we can isolate the controller behavior seen in logs:
%   SPEED_IQ_REF_A slowly decays while MOTOR_IQ remains small.

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

run(fullfile(script_dir, 'setup_green_joint_current_loop_twin.m'));
run(fullfile(script_dir, 'design_green_joint_speed_loop.m'));

cfg.Ts_current_s = GJDT_Ts;
cfg.Ts_speed_s = GJDT_TsSpeed;
cfg.Ts_plant_s = GJDT_TsPlant;
cfg.stop_time_s = 12.0;

cfg.vbus_v = double(GJDT_Vbus_V);
cfg.voltage_limit_ratio = 0.577;
cfg.voltage_modulation_ratio = 0.9;
cfg.voltage_limit_v = cfg.vbus_v * cfg.voltage_limit_ratio * ...
    cfg.voltage_modulation_ratio;

cfg.R_ohm = double(GJDT_Rs_Ohm);
cfg.Ld_h = double(GJDT_Ld_H);
cfg.Lq_h = double(GJDT_Lq_H);
cfg.pole_pairs = motor.pole_pairs;
cfg.psi_wb = motor.psi_f;
cfg.Kt_nm_per_a = motor.torque_constant;
cfg.gear_ratio = motor.gear_ratio;

cfg.speed_kp = double(GJDT_SpeedKp);
cfg.speed_ki = double(GJDT_SpeedKi);
cfg.speed_kaw = double(GJDT_SpeedKaw);
cfg.iq_limit_a = 4.0;
cfg.speed_integrator_initial_a = 4.0;

% Firmware current-loop bring-up defaults currently used by the adapter.
cfg.cur_d_kp = 1.0;
cfg.cur_d_ki = 20000.0;
cfg.cur_q_kp = 1.0;
cfg.cur_q_ki = 20000.0;
cfg.cur_kaw = 400.0;

% High-speed protection plateau. At this speed, back EMF is close to the Vq
% available from the 12 V bus, so the current loop cannot deliver large Iq.
cfg.no_load_motor_speed_rad_s = cfg.voltage_limit_v / ...
    (cfg.pole_pairs * cfg.psi_wb);
cfg.motor_speed_feedback_rad_s = 0.98 * cfg.no_load_motor_speed_rad_s;
cfg.joint_speed_feedback_rad_s = cfg.motor_speed_feedback_rad_s / ...
    cfg.gear_ratio;

% A small negative speed error gives a slow linear unwind, matching the field
% trace shape where speed is almost flat while SPEED_IQ_REF_A ramps down.
cfg.speed_error_rad_s = -3.0;
cfg.speed_ref_rad_s = cfg.joint_speed_feedback_rad_s + cfg.speed_error_rad_s;

result = simulate_high_speed_voltage_limited_case(cfg, false);
tracking_result = simulate_high_speed_voltage_limited_case(cfg, true);

results_dir = fullfile(script_dir, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

result_file = fullfile(results_dir, ...
    'green_joint_speed_loop_v1_high_speed_voltage_limit.csv');
writetable(result.signals, result_file);

summary = table( ...
    ["baseline_no_iq_tracking_aw"; "with_iq_tracking_aw"], ...
    [false; true], ...
    [result.metrics.time_to_iq_ref_below_0p5_s; ...
     tracking_result.metrics.time_to_iq_ref_below_0p5_s], ...
    [result.metrics.iq_ref_final_a; tracking_result.metrics.iq_ref_final_a], ...
    [result.metrics.iq_actual_mean_a; tracking_result.metrics.iq_actual_mean_a], ...
    [result.metrics.iq_actual_max_abs_a; ...
     tracking_result.metrics.iq_actual_max_abs_a], ...
    [result.metrics.voltage_norm_mean; ...
     tracking_result.metrics.voltage_norm_mean], ...
    [result.metrics.voltage_norm_max; tracking_result.metrics.voltage_norm_max], ...
    'VariableNames', { ...
    'case_name', ...
    'tracking_aw_enabled', ...
    'time_to_iq_ref_below_0p5_s', ...
    'iq_ref_final_a', ...
    'iq_actual_mean_a', ...
    'iq_actual_max_abs_a', ...
    'voltage_norm_mean', ...
    'voltage_norm_max'});

summary_file = fullfile(results_dir, ...
    'green_joint_speed_loop_v1_high_speed_voltage_limit_summary.csv');
writetable(summary, summary_file);

plot_file = fullfile(results_dir, ...
    'green_joint_speed_loop_v1_high_speed_voltage_limit.png');
plot_high_speed_result(result, tracking_result, cfg, plot_file);

fprintf('\nV1 high-speed voltage-limit speed-loop test:\n');
fprintf('  vbus                         = %.6g V\n', cfg.vbus_v);
fprintf('  voltage_limit                = %.6g V\n', cfg.voltage_limit_v);
fprintf('  no-load motor speed at Vlimit = %.6g rad/s\n', ...
    cfg.no_load_motor_speed_rad_s);
fprintf('  protected motor speed         = %.6g rad/s\n', ...
    cfg.motor_speed_feedback_rad_s);
fprintf('  protected joint speed         = %.6g rad/s\n', ...
    cfg.joint_speed_feedback_rad_s);
fprintf('  joint speed reference         = %.6g rad/s\n', ...
    cfg.speed_ref_rad_s);
fprintf('  joint speed error             = %.6g rad/s\n', ...
    cfg.speed_error_rad_s);
fprintf('  speed Ki                     = %.6g A/rad\n', cfg.speed_ki);
fprintf('  expected unwind slope         = %.6g A/s\n', ...
    cfg.speed_ki * cfg.speed_error_rad_s);
fprintf('  plant electrical step         = %.6g us\n', ...
    cfg.Ts_plant_s * 1e6);
fprintf('\nSummary:\n');
disp(summary);
fprintf('\nWrote signals:\n  %s\n', result_file);
fprintf('Wrote summary:\n  %s\n', summary_file);
fprintf('Wrote plot:\n  %s\n', plot_file);

function result = simulate_high_speed_voltage_limited_case(cfg, enable_tracking_aw)
num_steps = floor(cfg.stop_time_s / cfg.Ts_current_s) + 1;
speed_steps = max(1, round(cfg.Ts_speed_s / cfg.Ts_current_s));
plant_substeps = max(1, round(cfg.Ts_current_s / cfg.Ts_plant_s));
plant_dt = cfg.Ts_current_s / plant_substeps;

id = 0.0;
iq = 0.0;
d_int = 0.0;
q_int = 0.0;
speed_int = cfg.speed_integrator_initial_a;
iq_ref = cfg.speed_integrator_initial_a;

time_log = zeros(num_steps, 1);
iq_ref_log = zeros(num_steps, 1);
iq_log = zeros(num_steps, 1);
id_log = zeros(num_steps, 1);
vd_log = zeros(num_steps, 1);
vq_log = zeros(num_steps, 1);
voltage_norm_log = zeros(num_steps, 1);
speed_error_log = zeros(num_steps, 1);

omega_e = cfg.pole_pairs * cfg.motor_speed_feedback_rad_s;
tracking_aw_gain = cfg.speed_kaw;

% The V1 Simulink harness uses a faster plant step than the 50 us current
% controller. Use an exact discrete PMSM electrical update per plant substep
% so the high-speed/back-EMF test does not fail due to explicit Euler
% instability. The voltage command is held constant within one current tick,
% matching the zero-order hold behavior of the controller output.
Aelec = [-cfg.R_ohm / cfg.Ld_h, omega_e * cfg.Lq_h / cfg.Ld_h; ...
    -omega_e * cfg.Ld_h / cfg.Lq_h, -cfg.R_ohm / cfg.Lq_h];
Felec = expm(Aelec * plant_dt);
Gelec = Aelec \ (Felec - eye(2));

for k = 1:num_steps
    if mod(k - 1, speed_steps) == 0
        speed_error = cfg.speed_ref_rad_s - cfg.joint_speed_feedback_rad_s;
        iq_pre_sat = cfg.speed_kp * speed_error + speed_int;
        iq_ref = min(max(iq_pre_sat, -cfg.iq_limit_a), cfg.iq_limit_a);

        tracking_term = 0.0;
        if enable_tracking_aw
            tracking_term = tracking_aw_gain * (iq - iq_ref);
        end

        speed_int = speed_int + cfg.Ts_speed_s * ...
            (cfg.speed_ki * speed_error ...
             + cfg.speed_kaw * (iq_ref - iq_pre_sat) ...
             + tracking_term);
    end

    d_err = 0.0 - id;
    q_err = iq_ref - iq;
    vd_pre = cfg.cur_d_kp * d_err + d_int;
    vq_pre = cfg.cur_q_kp * q_err + q_int;

    [vd, vq, voltage_norm] = vd_priority_limit(vd_pre, vq_pre, ...
        cfg.voltage_limit_v);

    d_int = d_int + cfg.Ts_current_s * ...
        (cfg.cur_d_ki * d_err + cfg.cur_kaw * (vd - vd_pre));
    q_int = q_int + cfg.Ts_current_s * ...
        (cfg.cur_q_ki * q_err + cfg.cur_kaw * (vq - vq_pre));

    for plant_k = 1:plant_substeps
        forcing = [vd / cfg.Ld_h; ...
            (vq - omega_e * cfg.psi_wb) / cfg.Lq_h];
        x_dq = Felec * [id; iq] + Gelec * forcing;
        id = x_dq(1);
        iq = x_dq(2);
    end

    if ~isfinite(id) || ~isfinite(iq)
        error('Non-finite dq current at t=%.9g s: id=%.9g, iq=%.9g', ...
            (k - 1) * cfg.Ts_current_s, id, iq);
    end

    time_log(k) = (k - 1) * cfg.Ts_current_s;
    iq_ref_log(k) = iq_ref;
    iq_log(k) = iq;
    id_log(k) = id;
    vd_log(k) = vd;
    vq_log(k) = vq;
    voltage_norm_log(k) = voltage_norm;
    speed_error_log(k) = cfg.speed_ref_rad_s - cfg.speed_feedback_rad_s;
end

signals = table(time_log, iq_ref_log, iq_log, id_log, vd_log, vq_log, ...
    voltage_norm_log, speed_error_log, ...
    'VariableNames', {'time_s', 'iq_ref_a', 'iq_actual_a', ...
    'id_actual_a', 'vd_cmd_v', 'vq_cmd_v', 'voltage_norm', ...
    'speed_error_rad_s'});

below = find(abs(iq_ref_log) < 0.5, 1);
if isempty(below)
    time_to_iq_ref_below_0p5_s = nan;
else
    time_to_iq_ref_below_0p5_s = time_log(below);
end

metrics.time_to_iq_ref_below_0p5_s = time_to_iq_ref_below_0p5_s;
metrics.iq_ref_final_a = iq_ref_log(end);
metrics.iq_actual_mean_a = mean(iq_log(round(num_steps / 2):end));
metrics.iq_actual_max_abs_a = max(abs(iq_log));
metrics.voltage_norm_mean = mean(voltage_norm_log(round(num_steps / 2):end));
metrics.voltage_norm_max = max(voltage_norm_log);

result.signals = signals;
result.metrics = metrics;
end

function [vd, vq, voltage_norm] = vd_priority_limit(vd_pre, vq_pre, vlimit)
vd = min(max(vd_pre, -vlimit), vlimit);
vq_limit_sq = max(vlimit * vlimit - vd * vd, 0.0);
vq_limit = sqrt(vq_limit_sq);
vq = min(max(vq_pre, -vq_limit), vq_limit);
voltage_norm = sqrt(vd * vd + vq * vq) / vlimit;
end

function plot_high_speed_result(result, tracking_result, cfg, plot_file)
fig = figure('Visible', 'off');
t = result.signals.time_s;

subplot(3, 1, 1);
plot(t, result.signals.iq_ref_a, 'LineWidth', 1.2);
hold on;
plot(t, tracking_result.signals.iq_ref_a, '--', 'LineWidth', 1.2);
grid on;
ylabel('Iq ref (A)');
legend('baseline', 'with iq tracking AW', 'Location', 'best');
title('Speed-loop Iq reference at high-speed voltage limit');

subplot(3, 1, 2);
plot(t, result.signals.iq_actual_a, 'LineWidth', 1.2);
hold on;
plot(t, tracking_result.signals.iq_actual_a, '--', 'LineWidth', 1.2);
grid on;
ylabel('Iq actual (A)');

subplot(3, 1, 3);
plot(t, result.signals.voltage_norm, 'LineWidth', 1.2);
hold on;
plot(t, tracking_result.signals.voltage_norm, '--', 'LineWidth', 1.2);
yline(1.0, ':');
grid on;
xlabel('Time (s)');
ylabel('|Vdq| / limit');
sgtitle(sprintf('Vbus %.1fV, speed %.1frad/s, speed error %.1frad/s', ...
    cfg.vbus_v, cfg.speed_feedback_rad_s, cfg.speed_error_rad_s));

saveas(fig, plot_file);
close(fig);
end
