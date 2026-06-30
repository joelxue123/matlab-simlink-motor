%% Green-joint 1615 MIT mode numeric simulation
%
% This scenario mirrors the current firmware MIT path:
%   iq_cmd = MIT_kp * position_error
%          + MIT_kd * speed_error
%          + ff_torque / (Kt * gear_ratio)
%
% Important: in current firmware MIT_kp/MIT_kd produce motor-side Iq directly
% in A/rad and A/(rad/s). The feed-forward torque command is output-side Nm.
% The plant uses the 1615 output-side identified J/B/Tc/Tbias from the module
% variant contract.

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

run(fullfile(script_dir, 'setup_green_joint_current_loop_twin.m'));

cfg.Ts_current_s = GJDT_Ts;
cfg.stop_time_s = 0.45;
cfg.gear_ratio = motor.gear_ratio;
cfg.kt_motor_nm_per_a = motor.torque_constant;
cfg.kt_output_nm_per_a = cfg.kt_motor_nm_per_a * cfg.gear_ratio;
cfg.current_bandwidth_hz = GJDT_CurrentBandwidth_Hz;
cfg.current_time_constant_s = 1 / (2 * pi * cfg.current_bandwidth_hz);
cfg.iq_limit_a = GJDT_ModuleConfig.defaults.torque_limit_a;
cfg.J_output_kg_m2 = motor.output_equivalent_inertia_kg_m2;
cfg.B_output_nm_s_per_rad = motor.output_viscous_damping_nm_s_per_rad;
cfg.Tc_output_nm = motor.output_coulomb_friction_nm;
cfg.Tbias_output_nm = motor.output_torque_bias_nm;
cfg.friction_smoothing_rad_s = 0.02;
cfg.settling_band_rad = 0.002;

mit_15hz_zeta1 = design_mit_current_gains(cfg, 15.0, 1.0);
mit_20hz_zeta1 = design_mit_current_gains(cfg, 20.0, 1.0);
critical_kd_a_per_radps = estimate_critical_kd(cfg, 12.0);

scenarios = [
    struct('name', "mit_default_step_0p05rad", ...
        'pos_target_rad', 0.05, 'vel_target_rad_s', 0.0, ...
        'ff_torque_nm', 0.0, 'mit_kp', 12.0, 'mit_kd', 0.1, ...
        'design_bandwidth_hz', NaN, 'design_zeta', NaN)
    struct('name', "mit_default_step_0p2rad", ...
        'pos_target_rad', 0.20, 'vel_target_rad_s', 0.0, ...
        'ff_torque_nm', 0.0, 'mit_kp', 12.0, 'mit_kd', 0.1, ...
        'design_bandwidth_hz', NaN, 'design_zeta', NaN)
    struct('name', "mit_kd_near_critical_step_0p2rad", ...
        'pos_target_rad', 0.20, 'vel_target_rad_s', 0.0, ...
        'ff_torque_nm', 0.0, 'mit_kp', 12.0, ...
        'mit_kd', critical_kd_a_per_radps, ...
        'design_bandwidth_hz', NaN, 'design_zeta', 1.0)
    struct('name', "mit_bw15hz_zeta1_step_0p2rad", ...
        'pos_target_rad', 0.20, 'vel_target_rad_s', 0.0, ...
        'ff_torque_nm', 0.0, 'mit_kp', mit_15hz_zeta1.kp_a_per_rad, ...
        'mit_kd', mit_15hz_zeta1.kd_a_per_radps, ...
        'design_bandwidth_hz', 15.0, 'design_zeta', 1.0)
    struct('name', "mit_bw20hz_zeta1_step_0p2rad", ...
        'pos_target_rad', 0.20, 'vel_target_rad_s', 0.0, ...
        'ff_torque_nm', 0.0, 'mit_kp', mit_20hz_zeta1.kp_a_per_rad, ...
        'mit_kd', mit_20hz_zeta1.kd_a_per_radps, ...
        'design_bandwidth_hz', 20.0, 'design_zeta', 1.0)
    struct('name', "mit_ff_torque_0p2Nm_hold", ...
        'pos_target_rad', 0.0, 'vel_target_rad_s', 0.0, ...
        'ff_torque_nm', 0.20, 'mit_kp', 0.0, 'mit_kd', 0.0, ...
        'design_bandwidth_hz', NaN, 'design_zeta', NaN)
];

results_dir = fullfile(script_dir, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

summary = table();
for i = 1:numel(scenarios)
    result = simulate_mit_scenario(cfg, scenarios(i));
    signal_file = fullfile(results_dir, ...
        char(scenarios(i).name + "_signals.csv"));
    writetable(result.signals, signal_file);

    row = result.summary;
    row.signal_file = string(signal_file);
    summary = [summary; row]; %#ok<AGROW>
end

summary_file = fullfile(results_dir, ...
    'green_joint_mit_mode_1615_summary.csv');
writetable(summary, summary_file);

plot_file = fullfile(results_dir, 'green_joint_mit_mode_1615.png');
plot_mit_summary(cfg, scenarios, results_dir, plot_file);

fprintf('\nGreen-joint 1615 MIT mode simulation:\n');
fprintf('  module                  = %s\n', string(GJDT_ModuleConfig.module_name));
fprintf('  gear ratio              = %.9g\n', cfg.gear_ratio);
fprintf('  Kt motor                = %.9g N*m/A\n', cfg.kt_motor_nm_per_a);
fprintf('  Kt output               = %.9g N*m/A\n', cfg.kt_output_nm_per_a);
fprintf('  J output                = %.12g kg*m^2\n', cfg.J_output_kg_m2);
fprintf('  B output                = %.12g N*m*s/rad\n', ...
    cfg.B_output_nm_s_per_rad);
fprintf('  Tc / Tbias output       = %.12g / %.12g N*m\n', ...
    cfg.Tc_output_nm, cfg.Tbias_output_nm);
fprintf('  current loop bandwidth  = %.9g Hz\n', ...
    cfg.current_bandwidth_hz);
fprintf('  iq limit                = %.9g A\n', cfg.iq_limit_a);
fprintf('  default MIT Kp/Kd       = 12 / 0.1 (A/rad, A/(rad/s))\n');
fprintf('  near-critical Kd for Kp=12 = %.9g A/(rad/s)\n', ...
    critical_kd_a_per_radps);
fprintf('  MIT 15Hz zeta1 Kp/Kd    = %.9g / %.9g\n', ...
    mit_15hz_zeta1.kp_a_per_rad, mit_15hz_zeta1.kd_a_per_radps);
fprintf('  MIT 20Hz zeta1 Kp/Kd    = %.9g / %.9g\n', ...
    mit_20hz_zeta1.kp_a_per_rad, mit_20hz_zeta1.kd_a_per_radps);
fprintf('\nSummary:\n');
disp(summary(:, {'scenario', 'mit_kp_a_per_rad', ...
    'mit_kd_a_per_radps', 'design_bandwidth_hz', 'design_zeta', ...
    'pos_target_rad', 'ff_torque_nm', ...
    'output_stiffness_nm_per_rad', 'damping_ratio_est', ...
    'overshoot_pct', 'settling_time_s', 'final_pos_error_rad', ...
    'iq_cmd_abs_max_a', 'iq_saturated'}));
fprintf('\nWrote summary:\n  %s\n', summary_file);
fprintf('Wrote plot:\n  %s\n', plot_file);

function result = simulate_mit_scenario(cfg, scenario)
num_steps = floor(cfg.stop_time_s / cfg.Ts_current_s) + 1;
time = (0:num_steps - 1)' * cfg.Ts_current_s;

pos = 0.0;
vel = 0.0;
iq_actual = 0.0;
current_alpha = 1 - exp(-cfg.Ts_current_s / cfg.current_time_constant_s);

pos_log = zeros(num_steps, 1);
vel_log = zeros(num_steps, 1);
pos_err_log = zeros(num_steps, 1);
iq_cmd_log = zeros(num_steps, 1);
iq_ref_log = zeros(num_steps, 1);
iq_actual_log = zeros(num_steps, 1);
tau_motor_log = zeros(num_steps, 1);
tau_friction_log = zeros(num_steps, 1);
tau_net_log = zeros(num_steps, 1);

for k = 1:num_steps
    pos_err = wrap_pi_loop(scenario.pos_target_rad - pos);
    vel_err = scenario.vel_target_rad_s - vel;
    iq_cmd = scenario.mit_kp * pos_err ...
        + scenario.mit_kd * vel_err ...
        + scenario.ff_torque_nm / cfg.kt_output_nm_per_a;
    iq_ref = min(max(iq_cmd, -cfg.iq_limit_a), cfg.iq_limit_a);
    iq_actual = iq_actual + current_alpha * (iq_ref - iq_actual);

    tau_motor = cfg.kt_output_nm_per_a * iq_actual;
    tau_friction = cfg.B_output_nm_s_per_rad * vel ...
        + cfg.Tc_output_nm * tanh(vel / cfg.friction_smoothing_rad_s) ...
        + cfg.Tbias_output_nm;
    tau_net = tau_motor - tau_friction;
    accel = tau_net / cfg.J_output_kg_m2;

    vel = vel + cfg.Ts_current_s * accel;
    pos = pos + cfg.Ts_current_s * vel;

    pos_log(k) = pos;
    vel_log(k) = vel;
    pos_err_log(k) = pos_err;
    iq_cmd_log(k) = iq_cmd;
    iq_ref_log(k) = iq_ref;
    iq_actual_log(k) = iq_actual;
    tau_motor_log(k) = tau_motor;
    tau_friction_log(k) = tau_friction;
    tau_net_log(k) = tau_net;
end

signals = table(time, pos_log, vel_log, pos_err_log, iq_cmd_log, ...
    iq_ref_log, iq_actual_log, tau_motor_log, tau_friction_log, ...
    tau_net_log, ...
    'VariableNames', {'time_s', 'joint_pos_rad', 'joint_vel_rad_s', ...
    'pos_error_rad', 'iq_cmd_a', 'iq_ref_a', 'iq_actual_a', ...
    'motor_output_torque_nm', 'friction_torque_nm', 'net_torque_nm'});

final_error = wrap_pi_loop(scenario.pos_target_rad - pos_log(end));
if abs(scenario.pos_target_rad) > 1e-9
    overshoot = max(pos_log) - scenario.pos_target_rad;
    overshoot_pct = max(0.0, overshoot) / abs(scenario.pos_target_rad) * 100;
else
    overshoot_pct = 0.0;
end

settling_time_s = settling_time(time, pos_log, scenario.pos_target_rad, ...
    cfg.settling_band_rad);

output_stiffness = cfg.kt_output_nm_per_a * scenario.mit_kp;
output_damping = cfg.B_output_nm_s_per_rad ...
    + cfg.kt_output_nm_per_a * scenario.mit_kd;
if output_stiffness > 0
    damping_ratio = output_damping / ...
        (2 * sqrt(cfg.J_output_kg_m2 * output_stiffness));
    natural_frequency_hz = sqrt(output_stiffness / cfg.J_output_kg_m2) ...
        / (2 * pi);
else
    damping_ratio = NaN;
    natural_frequency_hz = NaN;
end

summary = table( ...
    string(scenario.name), ...
    scenario.mit_kp, ...
    scenario.mit_kd, ...
    scenario.pos_target_rad, ...
    scenario.vel_target_rad_s, ...
    scenario.ff_torque_nm, ...
    output_stiffness, ...
    output_damping, ...
    damping_ratio, ...
    natural_frequency_hz, ...
    overshoot_pct, ...
    settling_time_s, ...
    final_error, ...
    max(abs(vel_log)), ...
    max(abs(iq_cmd_log)), ...
    max(abs(iq_ref_log)), ...
    max(abs(iq_actual_log)), ...
    any(abs(iq_cmd_log) > cfg.iq_limit_a + 1e-6), ...
    scenario.design_bandwidth_hz, ...
    scenario.design_zeta, ...
    'VariableNames', {'scenario', 'mit_kp_a_per_rad', ...
    'mit_kd_a_per_radps', 'pos_target_rad', 'vel_target_rad_s', ...
    'ff_torque_nm', 'output_stiffness_nm_per_rad', ...
    'output_damping_nm_s_per_rad', 'damping_ratio_est', ...
    'natural_frequency_hz_est', 'overshoot_pct', 'settling_time_s', ...
    'final_pos_error_rad', 'vel_abs_max_rad_s', 'iq_cmd_abs_max_a', ...
    'iq_ref_abs_max_a', 'iq_actual_abs_max_a', 'iq_saturated', ...
    'design_bandwidth_hz', 'design_zeta'});

result.signals = signals;
result.summary = summary;
end

function kd = estimate_critical_kd(cfg, mit_kp_a_per_rad)
output_stiffness = cfg.kt_output_nm_per_a * mit_kp_a_per_rad;
critical_output_damping = 2 * sqrt(cfg.J_output_kg_m2 * output_stiffness);
kd = max(0.0, ...
    (critical_output_damping - cfg.B_output_nm_s_per_rad) ...
    / cfg.kt_output_nm_per_a);
end

function gains = design_mit_current_gains(cfg, bandwidth_hz, zeta)
wn = 2 * pi * bandwidth_hz;
kp_phys = cfg.J_output_kg_m2 * wn ^ 2;
kd_phys = 2 * zeta * cfg.J_output_kg_m2 * wn ...
    - cfg.B_output_nm_s_per_rad;

gains.bandwidth_hz = bandwidth_hz;
gains.zeta = zeta;
gains.kp_nm_per_rad = kp_phys;
gains.kd_nm_s_per_rad = kd_phys;
gains.kp_a_per_rad = kp_phys / cfg.kt_output_nm_per_a;
gains.kd_a_per_radps = max(0.0, kd_phys / cfg.kt_output_nm_per_a);
end

function result = settling_time(time, value, target, band)
inside = abs(value - target) <= band;
result = NaN;
for i = 1:numel(time)
    if inside(i) && all(inside(i:end))
        result = time(i);
        return;
    end
end
end

function y = wrap_pi_loop(x)
y = x;
while y >= pi
    y = y - 2 * pi;
end
while y < -pi
    y = y + 2 * pi;
end
end

function plot_mit_summary(cfg, scenarios, results_dir, plot_file)
figure('Visible', 'off', 'Position', [100 100 1100 760]);
tiledlayout(3, 1);

colors = lines(numel(scenarios));
for i = 1:numel(scenarios)
    data = readtable(fullfile(results_dir, ...
        char(scenarios(i).name + "_signals.csv")));

    nexttile(1);
    hold on;
    plot(data.time_s, data.joint_pos_rad, 'Color', colors(i, :), ...
        'DisplayName', char(scenarios(i).name));
    yline(scenarios(i).pos_target_rad, '--', 'Color', colors(i, :), ...
        'HandleVisibility', 'off');

    nexttile(2);
    hold on;
    plot(data.time_s, data.joint_vel_rad_s, 'Color', colors(i, :), ...
        'DisplayName', char(scenarios(i).name));

    nexttile(3);
    hold on;
    plot(data.time_s, data.iq_ref_a, 'Color', colors(i, :), ...
        'DisplayName', char(scenarios(i).name));
end

nexttile(1);
grid on;
ylabel('position (rad)');
title('green-joint 1615 MIT mode position response');
legend('Location', 'best');

nexttile(2);
grid on;
ylabel('velocity (rad/s)');
title('Output-side joint speed');

nexttile(3);
grid on;
ylabel('Iq ref (A)');
xlabel('time (s)');
title(sprintf('MIT current command, iq limit %.3g A', cfg.iq_limit_a));

saveas(gcf, plot_file);
close(gcf);
end
