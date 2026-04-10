function [w_est, theta_est] = kalman_speed_estimator(theta_meas, params)
% Discrete Kalman filter for speed estimation from position measurement.
%
% State:       x = [theta; omega]
% Model:       x(k+1) = A * x(k),  A = [1 Ts; 0 1]
% Measurement: z = H * x,           H = [1 0]
%
% params = [Ts, q_theta, q_omega, r_theta]
%   Ts      — sample time
%   q_theta — process noise on position (tune small, e.g. 1e-6)
%   q_omega — process noise on speed    (tune larger, e.g. 0.1)
%   r_theta — measurement noise on position (e.g. 1e-4)

persistent x_est P
if isempty(x_est)
    x_est = [0; 0];
    P = eye(2);
end

Ts = params(1);
Q  = [params(2) 0; 0 params(3)];
R  = params(4);

A = [1 Ts; 0 1];
H = [1 0];

% --- Predict ---
x_pred = A * x_est;
P_pred = A * P * A' + Q;

% --- Update ---
S = H * P_pred * H' + R;
K = P_pred * H' / S;
y_innov = theta_meas - H * x_pred;
x_est = x_pred + K * y_innov;
P = (eye(2) - K * H) * P_pred;

% --- Output ---
theta_est = x_est(1);
w_est     = x_est(2);
