function result = identify_coreless_motor_mechanics(data, cfg)
%IDENTIFY_CORELESS_MOTOR_MECHANICS Estimate J, B, Tc, and Tbias.
% The estimator uses measured current to compute torque and fits acceleration
% from position over pulse plateaus. It intentionally avoids diff(speed).

requiredVars = ["t_s", "segment_id", "is_pulse", "i_meas_A", ...
    "position_rad", "speed_rad_s", "feedback_valid", "error_code"];
assert(all(ismember(requiredVars, string(data.Properties.VariableNames))), ...
    "Input data table is missing required variables.");

valid = data.feedback_valid == 1 & data.error_code == 0 & ...
    isfinite(data.t_s) & isfinite(data.i_meas_A) & ...
    isfinite(data.position_rad) & isfinite(data.speed_rad_s);

candidateSegments = unique(data.segment_id(valid));
rows = zeros(0, 5);
windowNames = {'segment_id', 'window_id', 'sample_count', 't_start_s', 't_end_s', ...
    'i_mean_A', 'torque_mean_Nm', 'speed_mean_rad_s', ...
    'accel_fit_rad_s2', 'naive_J_kgm2', 'is_pulse_segment'};
windows = table('Size', [0, numel(windowNames)], ...
    'VariableTypes', repmat({'double'}, 1, numel(windowNames)), ...
    'VariableNames', windowNames);

for k = 1:numel(candidateSegments)
    segId = candidateSegments(k);
    idx0 = find(valid & data.segment_id == segId);
    isPulseSegment = any(data.is_pulse(idx0));
    idx = idx0;
    if isempty(idx)
        continue;
    end

    t0 = data.t_s(idx(1));
    localTime = data.t_s(idx) - t0;
    keep = localTime >= cfg.ident.edgeSkip_s;
    idx = idx(keep);

    if numel(idx) < cfg.ident.minPulseSamples
        continue;
    end

    [windowStart, windowStop] = make_window_bounds(data.t_s(idx), cfg);

    for wi = 1:numel(windowStart)
        widx = idx(data.t_s(idx) >= windowStart(wi) & data.t_s(idx) <= windowStop(wi));
        if numel(widx) < cfg.ident.minPulseSamples
            continue;
        end

        tp = data.t_s(widx) - data.t_s(widx(1));
        theta = data.position_rad(widx);
        P = [ones(size(tp)), tp, 0.5 * tp.^2];
        c = P \ theta;
        accel = c(3);

        iMean = mean(data.i_meas_A(widx));
        wMean = mean(data.speed_rad_s(widx));

        isUsefulPulse = isPulseSegment && abs(iMean) >= cfg.ident.minAbsCurrent_A;
        isUsefulCoast = ~isPulseSegment && ...
            abs(iMean) <= cfg.ident.maxCoastCurrent_A && ...
            abs(wMean) >= cfg.ident.minCoastSpeed_radps;

        if ~(isUsefulPulse || isUsefulCoast) || ...
                abs(accel) < cfg.ident.minAbsAcceleration_radps2
            continue;
        end

        torqueMean = cfg.ident.Kt_Nm_per_A * iMean;
        rows = [rows; accel, wMean, sign_with_zero(wMean), 1.0, torqueMean]; %#ok<AGROW>

        newRow = table(segId, wi, numel(widx), data.t_s(widx(1)), data.t_s(widx(end)), ...
            iMean, torqueMean, wMean, accel, torqueMean / accel, double(isPulseSegment), ...
            'VariableNames', windowNames);
        windows = [windows; newRow]; %#ok<AGROW>
    end
end

assert(size(rows, 1) >= 4, ...
    "Not enough valid pulse windows to identify J, B, Tc, and Tbias.");

Phi = rows(:, 1:4);
y = rows(:, 5);
p = Phi \ y;
yFit = Phi * p;
residual = y - yFit;

result = struct;
result.J_kgm2 = p(1);
result.B_Nm_per_radps = p(2);
result.Tc_Nm = p(3);
result.Tbias_Nm = p(4);
result.rmse_Nm = sqrt(mean(residual.^2));
result.window_count = size(rows, 1);
result.windows = windows;
result.truth = cfg.motor;
result.relative_error = struct( ...
    "J", relative_error(p(1), cfg.motor.J_kgm2), ...
    "B", relative_error(p(2), cfg.motor.B_Nm_per_radps), ...
    "Tc", relative_error(p(3), cfg.motor.Tc_Nm), ...
    "Tbias", absolute_error(p(4), cfg.motor.Tbias_Nm));
end

function s = sign_with_zero(x)
if abs(x) < 1e-9
    s = 0.0;
else
    s = sign(x);
end
end

function [windowStart, windowStop] = make_window_bounds(t, cfg)
duration = t(end) - t(1);
if duration <= cfg.ident.fitWindow_s
    windowStart = t(1);
    windowStop = t(end);
    return;
end

windowStart = (t(1):cfg.ident.fitHop_s:(t(end) - cfg.ident.fitWindow_s)).';
windowStop = windowStart + cfg.ident.fitWindow_s;
end

function e = relative_error(estimate, truth)
if abs(truth) < eps
    e = NaN;
else
    e = (estimate - truth) / truth;
end
end

function e = absolute_error(estimate, truth)
e = estimate - truth;
end
