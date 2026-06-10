function data = synthesize_coreless_motor_12v_data(cfg)
%SYNTHESIZE_CORELESS_MOTOR_12V_DATA Generate HPM-like identification data.
% The plant is intentionally simple: current-loop driven PMDC mechanics with
% current deadband, current-loop lag, voltage requirement logging, friction,
% and sensor noise. Mechanical identification should use measured current.

rng(cfg.sensor.randomSeed);

[t, iCmd, segmentId, isPulse] = build_current_pulse_train(cfg);
n = numel(t);

iActual = zeros(n, 1);
theta = zeros(n, 1);
w = zeros(n, 1);
diDt = zeros(n, 1);
voltageRequired = zeros(n, 1);
voltageSaturated = false(n, 1);

Ts = cfg.experiment.Ts_s;

for k = 2:n
    iTarget = apply_current_deadband(iCmd(k), cfg.driver.currentDeadband_A);
    iTarget = saturate(iTarget, -cfg.driver.currentLimit_A, cfg.driver.currentLimit_A);

    diDt(k) = (iTarget - iActual(k - 1)) / cfg.driver.currentLoopTau_s;
    iActual(k) = iActual(k - 1) + Ts * diDt(k);
    iActual(k) = saturate(iActual(k), -cfg.driver.currentLimit_A, cfg.driver.currentLimit_A);

    voltageRequired(k) = cfg.motor.R_ohm * iActual(k) + ...
        cfg.motor.L_H * diDt(k) + ...
        cfg.motor.Ke_V_per_radps * w(k - 1);
    voltageSaturated(k) = abs(voltageRequired(k)) > ...
        cfg.driver.voltageMargin * cfg.driver.Vdc_V;

    torqueNm = cfg.motor.Kt_Nm_per_A * iActual(k);
    frictionNm = cfg.motor.B_Nm_per_radps * w(k - 1) + ...
        cfg.motor.Tc_Nm * tanh(w(k - 1) / cfg.motor.frictionSpeedEps_radps) + ...
        cfg.motor.Tbias_Nm;
    accel = (torqueNm - frictionNm) / cfg.motor.J_kgm2;

    w(k) = w(k - 1) + Ts * accel;
    theta(k) = theta(k - 1) + Ts * w(k);
end

iMeas = iActual + cfg.sensor.currentNoiseStd_A * randn(n, 1);
thetaMeas = theta + cfg.sensor.positionNoiseStd_rad * randn(n, 1);
wMeas = w + cfg.sensor.speedNoiseStd_radps * randn(n, 1);
torqueMeas = cfg.motor.Kt_Nm_per_A * iMeas;

data = table;
data.cmd_seq = (0:n - 1).';
data.t_s = t;
data.feedback_valid = true(n, 1);
data.error_code = zeros(n, 1);
data.segment_id = segmentId;
data.is_pulse = isPulse;
data.i_cmd_A = iCmd;
data.i_meas_A = iMeas;
data.torque_nm = torqueMeas;
data.position_rad = thetaMeas;
data.speed_rad_s = wMeas;
data.v_required_V = voltageRequired;
data.Vdc_V = cfg.driver.Vdc_V * ones(n, 1);
data.voltage_saturated = voltageSaturated;
end

function [t, iCmd, segmentId, isPulse] = build_current_pulse_train(cfg)
Ts = cfg.experiment.Ts_s;
currents = [];
pulseMask = [];
segmentIds = [];
seg = 0;

append_segment(0.0, cfg.experiment.holdTime_s, false);

for r = 1:cfg.experiment.repeatCount
    append_segment(+cfg.experiment.pulseCurrent_A, cfg.experiment.pulseTime_s, true);
    append_segment(0.0, cfg.experiment.restTime_s, false);
    append_segment(-cfg.experiment.pulseCurrent_A, cfg.experiment.pulseTime_s, true);
    append_segment(0.0, cfg.experiment.restTime_s, false);
end

t = (0:numel(currents) - 1).' * Ts;
iCmd = currents(:);
segmentId = segmentIds(:);
isPulse = pulseMask(:);

    function append_segment(currentA, durationS, pulse)
        count = max(1, round(durationS / Ts));
        seg = seg + 1;
        currents = [currents; currentA * ones(count, 1)]; %#ok<AGROW>
        pulseMask = [pulseMask; repmat(logical(pulse), count, 1)]; %#ok<AGROW>
        segmentIds = [segmentIds; seg * ones(count, 1)]; %#ok<AGROW>
    end
end

function y = apply_current_deadband(u, deadband)
if abs(u) <= deadband
    y = 0.0;
else
    y = sign(u) * (abs(u) - deadband);
end
end

function y = saturate(u, lo, hi)
y = min(max(u, lo), hi);
end
