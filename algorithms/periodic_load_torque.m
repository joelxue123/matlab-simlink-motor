function T_load = periodic_load_torque(theta_meas, params)
% Synthetic compressor-like periodic load torque profile.
%
% params = [base, amp1, harmonic1, phase1_deg, amp2, harmonic2, phase2_deg]

base = params(1);
amp1 = params(2);
harmonic1 = params(3);
phase1 = params(4) * pi / 180;
amp2 = params(5);
harmonic2 = params(6);
phase2 = params(7) * pi / 180;

theta_wrap = mod(theta_meas, 2 * pi);
if theta_wrap < 0
    theta_wrap = theta_wrap + 2 * pi;
end

T_load = base + ...
    amp1 * sin(harmonic1 * theta_wrap + phase1) + ...
    amp2 * sin(harmonic2 * theta_wrap + phase2);
end
