function [id, iq] = abc2dq_fcn(ia, ib, ic, theta_e)
% abc to dq: Clarke + Park transform

% 1. Clarke: abc -> alpha-beta (equal-amplitude)
i_alpha = (2/3) * (ia - 0.5*ib - 0.5*ic);
i_beta  = (2/3) * (sqrt(3)/2*ib - sqrt(3)/2*ic);

% 2. Park: alpha-beta -> dq
id = i_alpha * cos(theta_e) + i_beta * sin(theta_e);
iq = -i_alpha * sin(theta_e) + i_beta * cos(theta_e);
