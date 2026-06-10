function R = analyze_hpm_inertia_pulse(run_csv, opts)
%ANALYZE_HPM_INERTIA_PULSE Estimate inertia from HPM EtherCAT pulse data.
%
% R = analyze_hpm_inertia_pulse(run_csv)
% R = analyze_hpm_inertia_pulse(run_csv, opts)
%
% Required time rule:
%   t = cmd_seq * 0.001

arguments
    run_csv
    opts.Period (1, 1) double = 0.001
    opts.DropEdgeMs (1, 1) double = 5
    opts.TailDropMs (1, 1) double = 5
    opts.MinPositionSpanRad (1, 1) double = 1e-4
    opts.MinAccelRadS2 (1, 1) double = 0.05
    opts.UseFeedbackTorque (1, 1) logical = false
end

run_csv = char(run_csv);
T = readtable(run_csv);
t = double(T.cmd_seq) * opts.Period;
positionUnwrapped = unwrap(double(T.position_rad));
valid = T.feedback_valid == 1 & T.error_code == 0 & ...
    isfinite(T.position_rad) & isfinite(T.speed_rad_s) & isfinite(T.torque_nm);

u = double(T.cmd_torque_nm);
active = abs(u) > 1e-4;
edges = find(diff([false; active]) == 1);
ends = find(diff([active; false]) == -1);

drop = round(opts.DropEdgeMs / 1000 / opts.Period);
tailDrop = round(opts.TailDropMs / 1000 / opts.Period);

pulse = [];
startSeq = [];
endSeq = [];
segment = [];
cmdTorque = [];
sampleCount = [];
positionSpan = [];
speedSpan = [];
meanSpeed = [];
meanTorque = [];
accelFit = [];
usableFlag = [];

for k = 1:numel(edges)
    i0 = edges(k);
    i1 = ends(k);
    j0 = i0 + drop;
    j1 = i1 - tailDrop;
    if j0 > j1
        continue;
    end

    idx = j0:j1;
    idx = idx(valid(idx));
    if numel(idx) < 20
        continue;
    end

    tau = t(idx) - t(idx(1));
    theta = positionUnwrapped(idx);
    speed = double(T.speed_rad_s(idx));
    p = polyfit(tau, theta, 2);
    accelVal = 2 * p(1);
    posSpan = max(theta) - min(theta);
    usable = posSpan >= opts.MinPositionSpanRad && abs(accelVal) >= opts.MinAccelRadS2;

    if opts.UseFeedbackTorque
        torqueMean = mean(double(T.torque_nm(idx)));
    else
        torqueMean = mean(double(T.cmd_torque_nm(idx)));
    end

    pulse(end + 1, 1) = k; %#ok<AGROW>
    startSeq(end + 1, 1) = T.cmd_seq(i0); %#ok<AGROW>
    endSeq(end + 1, 1) = T.cmd_seq(i1); %#ok<AGROW>
    segment(end + 1, 1) = T.segment(i0); %#ok<AGROW>
    cmdTorque(end + 1, 1) = u(i0); %#ok<AGROW>
    sampleCount(end + 1, 1) = numel(idx); %#ok<AGROW>
    positionSpan(end + 1, 1) = posSpan; %#ok<AGROW>
    speedSpan(end + 1, 1) = max(speed) - min(speed); %#ok<AGROW>
    meanSpeed(end + 1, 1) = mean(speed); %#ok<AGROW>
    meanTorque(end + 1, 1) = torqueMean; %#ok<AGROW>
    accelFit(end + 1, 1) = accelVal; %#ok<AGROW>
    usableFlag(end + 1, 1) = usable; %#ok<AGROW>
end

R = struct();
R.input_csv = run_csv;
R.motion.position_span_rad = max(positionUnwrapped(valid)) - min(positionUnwrapped(valid));
R.motion.speed_span_rad_s = max(T.speed_rad_s(valid)) - min(T.speed_rad_s(valid));
if isempty(pulse)
    pulseRows = table();
else
    pulseRows = table(pulse, startSeq, endSeq, segment, cmdTorque, sampleCount, ...
        positionSpan, speedSpan, meanSpeed, meanTorque, accelFit, logical(usableFlag), ...
        'VariableNames', {'pulse','start_seq','end_seq','segment','cmd_torque_nm', ...
        'samples','position_span_rad','speed_span_rad_s','mean_speed_rad_s', ...
        'mean_torque_nm','accel_rad_s2','usable'});
end
R.pulses = pulseRows;

if isempty(pulseRows)
    good = false(0, 1);
else
    good = pulseRows.usable == true;
end
if nnz(good) >= 4
    speedSign = sign(pulseRows.mean_speed_rad_s(good));
    nearZero = abs(pulseRows.mean_speed_rad_s(good)) < 1e-3;
    speedSign(nearZero) = sign(pulseRows.cmd_torque_nm(good));
    X = [pulseRows.accel_rad_s2(good), pulseRows.mean_speed_rad_s(good), speedSign, ones(nnz(good), 1)];
    y = pulseRows.mean_torque_nm(good);
    beta = X \ y;
    R.fit.J_kg_m2 = beta(1);
    R.fit.B_nm_s_per_rad = beta(2);
    R.fit.Tc_nm = beta(3);
    R.fit.Tbias_nm = beta(4);
    residual = y - X * beta;
    R.fit.residual_rms_nm = sqrt(mean(residual .* residual));
else
    R.fit = [];
end

disp(R.motion)
disp(R.pulses)
if ~isempty(R.fit)
    disp(R.fit)
else
    disp("No valid inertia fit: torque pulse did not create enough measurable motion.")
end

figure;
tiledlayout(3, 1);
nexttile;
plot(t, T.cmd_torque_nm);
grid on;
ylabel("cmd torque Nm");
nexttile;
plot(t, positionUnwrapped);
grid on;
ylabel("position rad");
nexttile;
plot(t, T.speed_rad_s);
grid on;
ylabel("speed rad/s");
xlabel("time s");
end
