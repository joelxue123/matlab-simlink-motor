function result = identify_pmsm_electrical_params(data, cfg)
%IDENTIFY_PMSM_ELECTRICAL_PARAMS Estimate Rs/Ld/Lq/flux/encoder offset.
% Interactive usage:
%   result = identify_pmsm_electrical_params()
%
% Explicit usage:
%   cfg = pmsm_electrical_id_config();
%   data = synthesize_pmsm_electrical_id_data(cfg);
%   result = identify_pmsm_electrical_params(data, cfg);

if nargin < 2 || isempty(cfg)
    cfg = pmsm_electrical_id_config();
end

if nargin < 1 || isempty(data)
    data = synthesize_pmsm_electrical_id_data(cfg);
end

[RsD, Ld] = estimate_axis_step(data.step, "d", cfg);
[RsQ, Lq] = estimate_axis_step(data.step, "q", cfg);
Rs = 0.5 * (RsD + RsQ);

psiSamples = (data.flux.vq_V - Rs * data.flux.iq_A) ./ data.flux.we_radps - ...
    Ld * data.flux.id_A;
psi = mean(psiSamples);

angleDelta = wrap_to_pi(data.angle.theta_encoder_rad - data.angle.theta_sensorless_rad);
encoderOffset = circular_mean(angleDelta);
angleResidual = wrap_to_pi(angleDelta - encoderOffset);

result = struct;
result.Rs_ohm = Rs;
result.Rs_d_ohm = RsD;
result.Rs_q_ohm = RsQ;
result.Ld_H = Ld;
result.Lq_H = Lq;
result.psi_f_Wb = psi;
result.encoderOffset_rad = encoderOffset;
result.encoderResidual1x_rad = harmonic_amplitude_by_angle(angleResidual, data.angle.theta_true_rad, 1);
result.encoderResidual2x_rad = harmonic_amplitude_by_angle(angleResidual, data.angle.theta_true_rad, 2);
result.angleResidualRms_rad = sqrt(mean(angleResidual.^2));
result.psiSamples = psiSamples;
result.angleResidual = angleResidual;
result.truth = cfg.motor;
result.relative_error = struct( ...
    "Rs", relative_error(Rs, cfg.motor.Rs_ohm), ...
    "Ld", relative_error(Ld, cfg.motor.Ld_H), ...
    "Lq", relative_error(Lq, cfg.motor.Lq_H), ...
    "psi_f", relative_error(psi, cfg.motor.psi_f_Wb), ...
    "encoderOffset", absolute_error(wrap_to_pi(encoderOffset - cfg.motor.encoderOffset_rad)));
end

function [Rs, L] = estimate_axis_step(stepData, axisName, cfg)
idx = stepData.axis == string(axisName);
T = stepData(idx, :);

tail = T.t_s >= (max(T.t_s) - cfg.ident.tailWindow_s);
vInf = mean(T.v_V(tail));
iInf = mean(T.i_A(tail));
Rs = vInf / iInf;

response = 1 - T.i_A / iInf;
fit = T.t_s >= cfg.ident.edgeSkip_s & ...
    response > cfg.ident.expFitMin & response < cfg.ident.expFitMax;

logResponse = log(response(fit));
Phi = [ones(sum(fit), 1), T.t_s(fit)];
p = Phi \ logResponse;
slope = p(2);
L = -Rs / slope;
end

function mu = circular_mean(x)
mu = angle(mean(exp(1j * x)));
end

function amp = harmonic_amplitude_by_angle(x, theta, order)
x = x(:) - mean(x(:));
coef = mean(x .* exp(-1j * order * theta(:)));
amp = 2 * abs(coef);
end

function y = wrap_to_pi(x)
y = mod(x + pi, 2 * pi) - pi;
end

function e = relative_error(estimate, truth)
e = (estimate - truth) / truth;
end

function e = absolute_error(estimate)
e = estimate;
end
