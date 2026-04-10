function [da, db, dc] = dq2abc_fcn(vd, vq, theta_e, Vdc)
% dq -> 占空比: 逆Park + 逆Clarke + SVPWM + 归一化

% 1. 逆Park: dq -> alpha-beta
v_alpha = vd * cos(theta_e) - vq * sin(theta_e);
v_beta  = vd * sin(theta_e) + vq * cos(theta_e);

% 2. 逆Clarke: alpha-beta -> abc
va = v_alpha;
vb = -0.5 * v_alpha + sqrt(3)/2 * v_beta;
vc = -0.5 * v_alpha - sqrt(3)/2 * v_beta;

% 3. SVPWM零序注入
v_max = max(max(va, vb), vc);
v_min = min(min(va, vb), vc);
v_n0  = -0.5 * (v_max + v_min);
va = va + v_n0;
vb = vb + v_n0;
vc = vc + v_n0;

%% 4. 占空比 [0,1]
da = va / Vdc + 0.5;
db = vb / Vdc + 0.5;
dc = vc / Vdc + 0.5;
