%% Motor impedance control simulation: no external torque vs torque disturbance
% Model:
%   J*qddot + b*qdot = tau_cmd + tau_ext
%
% Controller:
%   tau_cmd = K*(q_ref - q) + D*(qdot_ref - qdot)
%
% The reference is a quintic trajectory so q_ref, qdot_ref and qddot_ref
% are smooth. Two cases are simulated:
%   1. No external torque.
%   2. A step external torque is applied during motion.

clear; close all; clc;

%% Parameters
p.J = 0.01;          % kg*m^2, equivalent motor/load inertia
p.b = 0.02;          % N*m*s/rad, viscous damping
p.K = 4.0;           % N*m/rad, virtual stiffness
p.zeta = 1.0;        % damping ratio target
p.D = 2*p.zeta*sqrt(p.J*p.K) - p.b;  % controller damping

p.q0 = 0.0;          % rad
p.qf = 1.0;          % rad
p.T_move = 2.0;      % s
p.T_end = 3.0;       % s

p.tau_limit = 2.0;   % N*m, torque saturation
p.Kt = 0.08;         % N*m/A, motor torque constant
p.R = 1.2;           % Ohm, phase resistance approximation

% External torque for case 2.
p.disturb_time = 1.0;      % s
p.disturb_tau = -0.25;     % N*m

%% Derived bandwidth values
p.omega_n = sqrt(p.K / p.J);
p.bandwidth_hz_approx = p.omega_n / (2*pi);

fprintf('Impedance parameters:\n');
fprintf('  J = %.4f kg*m^2, b = %.4f N*m*s/rad\n', p.J, p.b);
fprintf('  K = %.4f N*m/rad, D = %.4f N*m*s/rad\n', p.K, p.D);
fprintf('  omega_n = %.3f rad/s, approx bandwidth = %.3f Hz, zeta = %.3f\n\n', ...
    p.omega_n, p.bandwidth_hz_approx, p.zeta);

%% Run simulations
case_no_ext = run_case(p, false);
case_with_ext = run_case(p, true);

metrics_no_ext = calc_metrics(case_no_ext, p);
metrics_with_ext = calc_metrics(case_with_ext, p);

disp('Metrics:');
print_metrics('No external torque', metrics_no_ext);
print_metrics('External torque during motion', metrics_with_ext);

%% Save results
script_dir = fileparts(mfilename('fullpath'));
root_dir = fileparts(script_dir);
results_dir = fullfile(root_dir, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

save(fullfile(results_dir, 'motor_impedance_two_cases.mat'), ...
    'p', 'case_no_ext', 'case_with_ext', 'metrics_no_ext', 'metrics_with_ext');

write_result_csv(fullfile(results_dir, 'motor_impedance_no_external_torque.csv'), case_no_ext);
write_result_csv(fullfile(results_dir, 'motor_impedance_with_external_torque.csv'), case_with_ext);

%% Plot
fig = figure('Name', 'Motor impedance control: two cases', 'Color', 'w');
tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(case_no_ext.t, case_no_ext.q_ref, 'k--', 'LineWidth', 1.2); hold on;
plot(case_no_ext.t, case_no_ext.q, 'b', 'LineWidth', 1.4);
plot(case_with_ext.t, case_with_ext.q, 'r', 'LineWidth', 1.4);
grid on;
ylabel('q / rad');
legend('q ref', 'no ext', 'with ext', 'Location', 'best');
title('Position response');

nexttile;
plot(case_no_ext.t, case_no_ext.e, 'b', 'LineWidth', 1.4); hold on;
plot(case_with_ext.t, case_with_ext.e, 'r', 'LineWidth', 1.4);
grid on;
ylabel('e / rad');
legend('no ext', 'with ext', 'Location', 'best');
title('Tracking error e = q_{ref} - q');

nexttile;
plot(case_no_ext.t, case_no_ext.tau_cmd, 'b', 'LineWidth', 1.4); hold on;
plot(case_with_ext.t, case_with_ext.tau_cmd, 'r', 'LineWidth', 1.4);
plot(case_with_ext.t, case_with_ext.tau_ext, 'k--', 'LineWidth', 1.2);
grid on;
ylabel('torque / N*m');
legend('tau cmd no ext', 'tau cmd with ext', 'tau ext', 'Location', 'best');
title('Command torque and external torque');

nexttile;
plot(case_no_ext.t, case_no_ext.current, 'b', 'LineWidth', 1.4); hold on;
plot(case_with_ext.t, case_with_ext.current, 'r', 'LineWidth', 1.4);
grid on;
ylabel('I / A');
xlabel('time / s');
legend('no ext', 'with ext', 'Location', 'best');
title('Estimated motor current');

exportgraphics(fig, fullfile(results_dir, 'motor_impedance_two_cases.png'), 'Resolution', 160);

fprintf('\nSaved results to:\n  %s\n', results_dir);

%% Local functions
function out = run_case(p, use_external_torque)
    x0 = [p.q0; 0.0];
    opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);
    [t, x] = ode45(@(t, x) motor_ode(t, x, p, use_external_torque), [0 p.T_end], x0, opts);

    q = x(:, 1);
    qdot = x(:, 2);
    n = numel(t);
    q_ref = zeros(n, 1);
    qdot_ref = zeros(n, 1);
    qddot_ref = zeros(n, 1);
    tau_cmd = zeros(n, 1);
    tau_ext = zeros(n, 1);

    for k = 1:n
        [q_ref(k), qdot_ref(k), qddot_ref(k)] = quintic_ref(t(k), p);
        tau_ext(k) = external_torque(t(k), p, use_external_torque);
        tau_nom = p.K*(q_ref(k) - q(k)) + p.D*(qdot_ref(k) - qdot(k));
        tau_cmd(k) = clamp(tau_nom, -p.tau_limit, p.tau_limit);
    end

    out.t = t;
    out.q = q;
    out.qdot = qdot;
    out.q_ref = q_ref;
    out.qdot_ref = qdot_ref;
    out.qddot_ref = qddot_ref;
    out.e = q_ref - q;
    out.tau_cmd = tau_cmd;
    out.tau_ext = tau_ext;
    out.current = tau_cmd / p.Kt;
    out.heat_power = out.current.^2 * p.R;
end

function dx = motor_ode(t, x, p, use_external_torque)
    q = x(1);
    qdot = x(2);
    [q_ref, qdot_ref, ~] = quintic_ref(t, p);

    tau_nom = p.K*(q_ref - q) + p.D*(qdot_ref - qdot);
    tau_cmd = clamp(tau_nom, -p.tau_limit, p.tau_limit);
    tau_ext = external_torque(t, p, use_external_torque);

    qddot = (tau_cmd + tau_ext - p.b*qdot) / p.J;
    dx = [qdot; qddot];
end

function tau_ext = external_torque(t, p, use_external_torque)
    if use_external_torque && t >= p.disturb_time
        tau_ext = p.disturb_tau;
    else
        tau_ext = 0.0;
    end
end

function [q_ref, qdot_ref, qddot_ref] = quintic_ref(t, p)
    if t <= 0
        q_ref = p.q0;
        qdot_ref = 0.0;
        qddot_ref = 0.0;
        return;
    end

    if t >= p.T_move
        q_ref = p.qf;
        qdot_ref = 0.0;
        qddot_ref = 0.0;
        return;
    end

    tau = t / p.T_move;
    dq = p.qf - p.q0;

    s = 10*tau^3 - 15*tau^4 + 6*tau^5;
    sdot = (30*tau^2 - 60*tau^3 + 30*tau^4) / p.T_move;
    sddot = (60*tau - 180*tau^2 + 120*tau^3) / (p.T_move^2);

    q_ref = p.q0 + dq*s;
    qdot_ref = dq*sdot;
    qddot_ref = dq*sddot;
end

function y = clamp(x, lower, upper)
    y = min(max(x, lower), upper);
end

function metrics = calc_metrics(out, p)
    metrics.e_rms = sqrt(mean(out.e.^2));
    metrics.e_peak = max(abs(out.e));
    metrics.tau_rms = sqrt(mean(out.tau_cmd.^2));
    metrics.tau_peak = max(abs(out.tau_cmd));
    metrics.i_rms = sqrt(mean(out.current.^2));
    metrics.i_peak = max(abs(out.current));
    metrics.heat_mean = mean(out.heat_power);
    metrics.heat_energy = trapz(out.t, out.heat_power);
    metrics.final_error = out.q_ref(end) - out.q(end);
    metrics.theory_static_error = -p.disturb_tau / p.K;
end

function print_metrics(name, m)
    fprintf('  %s:\n', name);
    fprintf('    e_rms       = %.6f rad\n', m.e_rms);
    fprintf('    e_peak      = %.6f rad\n', m.e_peak);
    fprintf('    tau_rms     = %.6f N*m\n', m.tau_rms);
    fprintf('    tau_peak    = %.6f N*m\n', m.tau_peak);
    fprintf('    I_rms       = %.6f A\n', m.i_rms);
    fprintf('    I_peak      = %.6f A\n', m.i_peak);
    fprintf('    heat_mean   = %.6f W\n', m.heat_mean);
    fprintf('    heat_energy = %.6f J\n', m.heat_energy);
    fprintf('    final_error = %.6f rad\n', m.final_error);
end

function write_result_csv(filename, out)
    header = 't,q,qdot,q_ref,qdot_ref,qddot_ref,error,tau_cmd,tau_ext,current,heat_power';
    data = [out.t, out.q, out.qdot, out.q_ref, out.qdot_ref, out.qddot_ref, ...
        out.e, out.tau_cmd, out.tau_ext, out.current, out.heat_power];

    fid = fopen(filename, 'w');
    if fid < 0
        error('Cannot open CSV file: %s', filename);
    end
    fprintf(fid, '%s\n', header);
    fclose(fid);

    dlmwrite(filename, data, '-append');
end
