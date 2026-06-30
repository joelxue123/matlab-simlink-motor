function data = synthesize_pmsm_electrical_id_data(cfg)
%SYNTHESIZE_PMSM_ELECTRICAL_ID_DATA Generate synthetic bench-like data.

rng(cfg.noise.randomSeed);

data.step = [make_step_table("d", cfg.step.vdStep_V, cfg.motor.Rs_ohm, cfg.motor.Ld_H, cfg); ...
             make_step_table("q", cfg.step.vqStep_V, cfg.motor.Rs_ohm, cfg.motor.Lq_H, cfg)];
data.flux = make_flux_table(cfg);
data.angle = make_angle_table(cfg);
end

function T = make_step_table(axisName, voltageStep, Rs, L, cfg)
t = (0:cfg.step.Ts_s:cfg.step.duration_s).';
iTrue = voltageStep / Rs * (1 - exp(-t * Rs / L));
iMeas = iTrue + cfg.noise.currentStd_A * randn(size(t));
vMeas = voltageStep + cfg.noise.voltageStd_V * randn(size(t));

T = table;
T.axis = repmat(string(axisName), numel(t), 1);
T.t_s = t;
T.v_V = vMeas;
T.i_A = iMeas;
end

function T = make_flux_table(cfg)
we = repelem(cfg.flux.we_radps(:), cfg.flux.samplesPerSpeed, 1);
n = numel(we);
id = 0.02 * randn(n, 1);
iq = 0.02 * randn(n, 1);
did_dt = zeros(n, 1);
diq_dt = zeros(n, 1);

vd = cfg.motor.Rs_ohm * id + cfg.motor.Ld_H * did_dt - we .* cfg.motor.Lq_H .* iq;
vq = cfg.motor.Rs_ohm * iq + cfg.motor.Lq_H * diq_dt + ...
    we .* (cfg.motor.Ld_H * id + cfg.motor.psi_f_Wb);

T = table;
T.we_radps = we;
T.id_A = id;
T.iq_A = iq;
T.vd_V = vd + cfg.noise.voltageStd_V * randn(n, 1);
T.vq_V = vq + cfg.noise.voltageStd_V * randn(n, 1);
end

function T = make_angle_table(cfg)
n = cfg.angle.sampleCount;
thetaTrue = linspace(0, 2 * pi * cfg.angle.electricalTurns, n).';
thetaWrapped = wrap_to_pi(thetaTrue);

nonlinear = cfg.motor.encoderNonlinear1_rad * sin(thetaTrue) + ...
    cfg.motor.encoderNonlinear2_rad * sin(2 * thetaTrue);
thetaEncoder = wrap_to_pi(thetaTrue + cfg.motor.encoderOffset_rad + nonlinear + ...
    cfg.noise.angleStd_rad * randn(n, 1));
thetaSensorless = wrap_to_pi(thetaTrue + cfg.noise.angleStd_rad * randn(n, 1));

T = table;
T.sample = (0:n - 1).';
T.theta_true_rad = thetaWrapped;
T.theta_encoder_rad = thetaEncoder;
T.theta_sensorless_rad = thetaSensorless;
end

function y = wrap_to_pi(x)
y = mod(x + pi, 2 * pi) - pi;
end
