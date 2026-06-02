function report = analyze_speedloop_pi_openloop(varargin)
% Analyze the speed-loop PI open-loop transfer function.
%
% Usage:
%   report = analyze_speedloop_pi_openloop;
%   report = analyze_speedloop_pi_openloop('make_plots', false);
%
% The main open-loop object is from speed error to measured mechanical speed:
%
%   L(s) = C_speed(s) * G_iq_to_w(s)
%
% where C_speed(s) = Kp + Ki/s and G_iq_to_w(s) includes the q-axis current
% loop, torque constant, mechanical plant, and an optional small delay.

opts = local_parse_options(varargin{:});

project_root = fileparts(mfilename('fullpath'));
init_project_paths(project_root);
motor_control_params;

if exist('tf', 'file') ~= 2
    error('Control System Toolbox is required because this analysis uses tf/margin/bode.');
end

s = tf('s');

Kp = control.pi_speed.Kp;
Ki = control.pi_speed.Ki;
Kt = motor.torque_constant;
J = motor.J;
B = motor.B;
tau_current = control.tau_current_cl;
tau_sigma = control.tau_sigma;

speed_pi = Kp + Ki / s;
mechanical_plant = Kt / (J * s + B);
current_closed_loop = 1 / (tau_current * s + 1);

% A first-order Pade model is enough for loop-shape exploration at this
% bandwidth. Keep it separate from the nominal small-time-constant model.
[delay_num, delay_den] = pade(tau_sigma, 1);
delay_pade = tf(delay_num, delay_den);

plant_ideal = mechanical_plant;
plant_with_current = current_closed_loop * mechanical_plant;
plant_with_current_delay = delay_pade * plant_with_current;

open_loop_ideal = speed_pi * plant_ideal;
open_loop_with_current = speed_pi * plant_with_current;
open_loop_with_current_delay = speed_pi * plant_with_current_delay;

closed_loop_ideal = feedback(open_loop_ideal, 1);
closed_loop_with_current = feedback(open_loop_with_current, 1);
closed_loop_with_current_delay = feedback(open_loop_with_current_delay, 1);

Ts = simcfg.Ts_speed;
z = tf('z', Ts);
speed_pi_discrete_forward_euler = Kp + Ki * Ts * z / (z - 1);
speed_pi_discrete_tustin = c2d(speed_pi, Ts, 'tustin');

report = struct();
report.parameters = struct( ...
    'Kp', Kp, ...
    'Ki', Ki, ...
    'zero_rad_s', Ki / Kp, ...
    'zero_hz', Ki / Kp / (2 * pi), ...
    'Kt_Nm_per_A', Kt, ...
    'J_kg_m2', J, ...
    'B_Nm_s_per_rad', B, ...
    'mechanical_pole_rad_s', B / J, ...
    'mechanical_pole_hz', B / J / (2 * pi), ...
    'tau_current_s', tau_current, ...
    'tau_sigma_s', tau_sigma, ...
    'Ts_speed_s', Ts, ...
    'design_bw_hz', control.speed_bandwidth_hz, ...
    'design_bw_rad_s', control.speed_bandwidth_rad_s);
report.speed_pi = speed_pi;
report.speed_pi_discrete_forward_euler = speed_pi_discrete_forward_euler;
report.speed_pi_discrete_tustin = speed_pi_discrete_tustin;
report.plant_ideal = plant_ideal;
report.plant_with_current = plant_with_current;
report.plant_with_current_delay = plant_with_current_delay;
report.open_loop_ideal = open_loop_ideal;
report.open_loop_with_current = open_loop_with_current;
report.open_loop_with_current_delay = open_loop_with_current_delay;
report.closed_loop_ideal = closed_loop_ideal;
report.closed_loop_with_current = closed_loop_with_current;
report.closed_loop_with_current_delay = closed_loop_with_current_delay;
report.margins = struct( ...
    'ideal', local_margin_summary(open_loop_ideal), ...
    'with_current', local_margin_summary(open_loop_with_current), ...
    'with_current_delay', local_margin_summary(open_loop_with_current_delay));
report.stepinfo = struct( ...
    'ideal', stepinfo(closed_loop_ideal), ...
    'with_current', stepinfo(closed_loop_with_current), ...
    'with_current_delay', stepinfo(closed_loop_with_current_delay));

fprintf('\n=== Speed-loop PI open-loop analysis ===\n');
fprintf('C_speed(s) = Kp + Ki/s = %.9g + %.9g/s\n', Kp, Ki);
fprintf('PI zero    = %.3f rad/s (%.3f Hz)\n', report.parameters.zero_rad_s, report.parameters.zero_hz);
fprintf('G_mech(s)  = Kt/(J*s + B) = %.9g/(%.9g*s + %.9g)\n', Kt, J, B);
fprintf('Mechanical pole = %.3f rad/s (%.3f Hz)\n', ...
    report.parameters.mechanical_pole_rad_s, report.parameters.mechanical_pole_hz);
fprintf('Current-loop lag tau = %.6g s, tau_sigma = %.6g s\n', tau_current, tau_sigma);
fprintf('Design speed BW = %.3f Hz (%.3f rad/s)\n\n', ...
    control.speed_bandwidth_hz, control.speed_bandwidth_rad_s);

fprintf('Open-loop forms:\n');
fprintf('  Ideal:              L0(s) = (Kp*s + Ki)/s * Kt/(J*s + B)\n');
fprintf('  With current lag:   L1(s) = L0(s) * 1/(tau_current*s + 1)\n');
fprintf('  With lag + delay:   L2(s) = L1(s) * pade(exp(-tau_sigma*s), 1)\n\n');

local_print_margin('Ideal', report.margins.ideal);
local_print_margin('With current lag', report.margins.with_current);
local_print_margin('With current lag + Pade delay', report.margins.with_current_delay);

fprintf('\nClosed-loop step estimates:\n');
local_print_stepinfo('Ideal', report.stepinfo.ideal);
local_print_stepinfo('With current lag', report.stepinfo.with_current);
local_print_stepinfo('With current lag + Pade delay', report.stepinfo.with_current_delay);

fprintf('\nDiscrete PI used by speed_pi_fcn:\n');
fprintf('  C(z) = Kp + Ki*Ts*z/(z - 1), Ts = %.9g s\n', Ts);
fprintf('  This is the forward-Euler integrator implementation in algorithms/speed_pi_fcn.m.\n');

if opts.make_plots
    local_make_plots(report);
end

assignin('base', 'speedloop_pi_openloop_report', report);
end

function opts = local_parse_options(varargin)
opts = struct('make_plots', true);
if mod(nargin, 2) ~= 0
    error('Options must be name/value pairs.');
end
for idx = 1:2:nargin
    name = lower(string(varargin{idx}));
    value = varargin{idx + 1};
    switch name
        case "make_plots"
            opts.make_plots = logical(value);
        otherwise
            error('Unknown option: %s', name);
    end
end
end

function summary = local_margin_summary(loop_tf)
[gain_margin, phase_margin, wg, wp] = margin(loop_tf);
summary = struct( ...
    'gain_margin_abs', gain_margin, ...
    'gain_margin_db', 20 * log10(gain_margin), ...
    'phase_margin_deg', phase_margin, ...
    'gain_cross_rad_s', wp, ...
    'gain_cross_hz', wp / (2 * pi), ...
    'phase_cross_rad_s', wg, ...
    'phase_cross_hz', wg / (2 * pi));
end

function local_print_margin(name, summary)
fprintf('%s:\n', name);
fprintf('  gain crossover  = %.3f rad/s (%.3f Hz)\n', ...
    summary.gain_cross_rad_s, summary.gain_cross_hz);
fprintf('  phase margin    = %.3f deg\n', summary.phase_margin_deg);
fprintf('  gain margin     = %.3f dB\n', summary.gain_margin_db);
end

function local_print_stepinfo(name, info)
fprintf('  %-32s rise %.6f s, settle %.6f s, overshoot %.3f %%\n', ...
    name + ":", info.RiseTime, info.SettlingTime, info.Overshoot);
end

function local_make_plots(report)
figure('Name', 'Speed PI Open-Loop Bode');
margin(report.open_loop_ideal);
hold on;
margin(report.open_loop_with_current);
margin(report.open_loop_with_current_delay);
grid on;
legend('Ideal', 'With current lag', 'With current lag + Pade delay', 'Location', 'best');
title('Speed-Loop Open-Loop L(s)');

figure('Name', 'Speed Loop Closed-Loop Step');
step(report.closed_loop_ideal, report.closed_loop_with_current, report.closed_loop_with_current_delay);
grid on;
legend('Ideal', 'With current lag', 'With current lag + Pade delay', 'Location', 'best');
title('Speed-Loop Closed-Loop Response');

figure('Name', 'Speed PI Root Locus');
rlocus(report.open_loop_with_current_delay);
grid on;
title('Root Locus of Speed PI Loop, Including Current Lag and Delay');
end
