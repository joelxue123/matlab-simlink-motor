%% Sweep speed-loop saturation exit with the current green-joint parameters
%
% This is a transparent numeric simulation of the generated SpeedPiStep
% equation plus a first-order mechanical plant:
%
%   iq_pre_sat = Kp * (wm_ref - wm) + integrator
%   iq_ref = clamp(iq_pre_sat, -iq_limit, iq_limit)
%   integrator += Ts * (Ki * error + Kaw * (iq_ref - iq_pre_sat))
%   J * d(wm)/dt = Kt * iq_ref - B * wm
%
% It is intentionally separate from the full plant wrapper so we can quickly
% tell whether long speed response is dominated by current saturation.

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

run(fullfile(script_dir, 'setup_green_joint_current_loop_twin.m'));
run(fullfile(script_dir, 'design_green_joint_speed_loop.m'));

speed_ref_rad_s = 40.0;
sim_stop_s = 0.120;
Ts_speed_s = GJDT_TsSpeed;
J_kg_m2 = GJDT_SpeedLoopEquivalentInertia_kg_m2;
Kt_nm_per_a = motor.torque_constant;
Kp_speed = double(GJDT_SpeedKp);
Ki_speed = double(GJDT_SpeedKi);
Kaw_speed = double(GJDT_SpeedKaw);

iq_limits_a = [0.02; 0.05; 0.1; ...
    motor.rated_current_a; motor.peak_current_a; 4.0];
damping_cases = table( ...
    ["B0_pure_inertia"; "B_setup_template"], ...
    [0.0; motor.B], ...
    'VariableNames', {'case_name', 'B_nm_s_per_rad'});

rows = table();
for bidx = 1:height(damping_cases)
    B_nm_s_per_rad = damping_cases.B_nm_s_per_rad(bidx);
    for i = 1:numel(iq_limits_a)
        result = simulate_speed_step(speed_ref_rad_s, iq_limits_a(i), ...
            Kp_speed, Ki_speed, Kaw_speed, Ts_speed_s, sim_stop_s, ...
            J_kg_m2, Kt_nm_per_a, B_nm_s_per_rad);

        if B_nm_s_per_rad > 0
            no_load_speed_limit_rad_s = Kt_nm_per_a * iq_limits_a(i) / ...
                B_nm_s_per_rad;
        else
            no_load_speed_limit_rad_s = inf;
        end

        ideal_accel_time_ms = speed_ref_rad_s * J_kg_m2 / ...
            (Kt_nm_per_a * iq_limits_a(i)) * 1e3;

        row = table( ...
            damping_cases.case_name(bidx), ...
            B_nm_s_per_rad, ...
            iq_limits_a(i), ...
            no_load_speed_limit_rad_s, ...
            ideal_accel_time_ms, ...
            result.exit_saturation_ms, ...
            result.saturated_time_ms, ...
            result.reach_98pct_ms, ...
            result.final_speed_rad_s, ...
            result.final_error_rad_s, ...
            result.max_iq_ref_a, ...
            result.final_iq_ref_a, ...
            result.saturated_at_end, ...
            'VariableNames', { ...
            'case_name', ...
            'B_nm_s_per_rad', ...
            'iq_limit_a', ...
            'no_load_speed_limit_rad_s', ...
            'ideal_B0_accel_time_ms', ...
            'exit_saturation_ms', ...
            'saturated_time_ms', ...
            'reach_98pct_ms', ...
            'final_speed_rad_s', ...
            'final_error_rad_s', ...
            'max_iq_ref_a', ...
            'final_iq_ref_a', ...
            'saturated_at_end'});
        rows = [rows; row]; %#ok<AGROW>
    end
end

results_dir = fullfile(script_dir, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end
result_file = fullfile(results_dir, ...
    'green_joint_speed_loop_saturation_sweep.csv');
writetable(rows, result_file);

iq4_rows = rows(abs(rows.iq_limit_a - 4.0) < 1e-6, :);
iq4_result_file = fullfile(results_dir, ...
    'green_joint_speed_loop_iq_limit_4a_summary.csv');
writetable(iq4_rows, iq4_result_file);

fprintf('\nGreen-joint speed-loop saturation sweep:\n');
fprintf('  speed ref = %.6g rad/s output-side joint speed\n', speed_ref_rad_s);
fprintf('  gear ratio= %.9g\n', GJDT_MotorGearRatio);
fprintf('  J equiv   = %.9g kg*m^2 for joint-speed input to motor-Iq output\n', J_kg_m2);
fprintf('  Kt        = %.9g N*m/A motor-side\n', Kt_nm_per_a);
fprintf('  Ts_speed  = %.9g s\n', Ts_speed_s);
fprintf('  Kp/Ki/Kaw = %.9g / %.9g / %.9g\n', ...
    Kp_speed, Ki_speed, Kaw_speed);
disp(rows(:, {'case_name', 'iq_limit_a', 'no_load_speed_limit_rad_s', ...
    'ideal_B0_accel_time_ms', 'exit_saturation_ms', 'reach_98pct_ms', ...
    'final_speed_rad_s', 'saturated_at_end'}));
fprintf('\nFocused iq_limit = 4A cases:\n');
disp(iq4_rows(:, {'case_name', 'iq_limit_a', ...
    'ideal_B0_accel_time_ms', 'exit_saturation_ms', 'reach_98pct_ms', ...
    'final_speed_rad_s', 'final_iq_ref_a', 'saturated_at_end'}));
fprintf('\nWrote saturation sweep:\n  %s\n', result_file);
fprintf('Wrote iq_limit=4A summary:\n  %s\n', iq4_result_file);

function result = simulate_speed_step(speed_ref_rad_s, iq_limit_a, ...
    Kp_speed, Ki_speed, Kaw_speed, Ts_speed_s, sim_stop_s, ...
    J_kg_m2, Kt_nm_per_a, B_nm_s_per_rad)

num_steps = floor(sim_stop_s / Ts_speed_s) + 1;
wm = 0.0;
integrator = 0.0;

sat = false(num_steps, 1);
iq = zeros(num_steps, 1);
speed = zeros(num_steps, 1);
time = (0:num_steps - 1)' * Ts_speed_s;

for k = 1:num_steps
    err = speed_ref_rad_s - wm;
    iq_pre_sat = Kp_speed * err + integrator;
    iq_ref = min(max(iq_pre_sat, -iq_limit_a), iq_limit_a);
    sat(k) = abs(iq_ref - iq_pre_sat) > 1e-8;
    iq(k) = iq_ref;
    speed(k) = wm;

    integrator = integrator + Ts_speed_s * ...
        (Ki_speed * err + Kaw_speed * (iq_ref - iq_pre_sat));
    wm = wm + Ts_speed_s * ...
        (Kt_nm_per_a * iq_ref - B_nm_s_per_rad * wm) / J_kg_m2;
end

sat_idx = find(sat);
if isempty(sat_idx)
    exit_saturation_ms = 0.0;
else
    exit_saturation_ms = time(sat_idx(end)) * 1e3;
end

reach_idx = find(speed >= 0.98 * speed_ref_rad_s, 1);
if isempty(reach_idx)
    reach_98pct_ms = nan;
else
    reach_98pct_ms = time(reach_idx) * 1e3;
end

result.exit_saturation_ms = exit_saturation_ms;
result.saturated_time_ms = sum(sat) * Ts_speed_s * 1e3;
result.reach_98pct_ms = reach_98pct_ms;
result.final_speed_rad_s = speed(end);
result.final_error_rad_s = speed_ref_rad_s - speed(end);
result.max_iq_ref_a = max(abs(iq));
result.final_iq_ref_a = iq(end);
result.saturated_at_end = sat(end);
end
