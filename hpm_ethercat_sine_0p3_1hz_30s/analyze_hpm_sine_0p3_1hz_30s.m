clear; close all; clc;

baseDir = fileparts(mfilename('fullpath'));
dataDir = fullfile(baseDir, 'data');
outDir = fullfile(baseDir, 'output');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end
summaryFile = fullfile(outDir, 'hpm_sine_0p3_1hz_30s_summary.txt');
if exist(summaryFile, 'file')
    delete(summaryFile);
end
diary(summaryFile);
cleanupObj = onCleanup(@() diary('off'));

csvFile = fullfile(dataDir, ...
    'hpm_traj_sine_amp0p3_freq1hz_kp12_kd0p2_30s_20260609_185429.csv');

ampCmd = 0.3;
freqHz = 1.0;
ts = 1e-3;
w = 2 * pi * freqHz;

T = readtable(csvFile);

seq = T.frame_index - 1;
t = seq * ts;
pos = T.position_rad;
vel = T.speed_rad;
torque = T.torque_nm;

posCmd = ampCmd * sin(w * t);
velCmd = ampCmd * w * cos(w * t);
posErr = pos - posCmd;

% Estimate the measured 1 Hz component:
% pos ~= b1*sin(wt) + b2*cos(wt) + b3
X = [sin(w * t), cos(w * t), ones(size(t))];
b = X \ pos;
posFit = X * b;
ampEst = hypot(b(1), b(2));
phaseRad = atan2(b(2), b(1));
delayMs = -phaseRad / w * 1000;
offsetRad = b(3);

rmsErr = sqrt(mean(posErr.^2));
maxAbsErr = max(abs(posErr));
posMin = min(pos);
posMax = max(pos);
velMin = min(vel);
velMax = max(vel);
torqueMin = min(torque);
torqueMax = max(torque);

frameDiff = diff(T.frame_index);
cycleDiff = diff(T.cycle);
frameGapCount = sum(frameDiff ~= 1);
cycleDupCount = sum(cycleDiff == 0);
cycleGapEvents = sum(cycleDiff > 1);
missingCycleCount = sum(cycleDiff(cycleDiff > 1) - 1);
cycleBackCount = sum(cycleDiff < 0);
if isnumeric(T.error_code)
    nonzeroErrorCount = sum(T.error_code ~= 0);
else
    errorText = string(T.error_code);
    nonzeroErrorCount = sum(errorText ~= "0x0000" & errorText ~= "0");
end

fprintf('HPM sine test 0.3 rad, 1 Hz, 30 s\n');
fprintf('samples: %d\n', height(T));
fprintf('frame_index: %d .. %d, non-continuous steps: %d\n', ...
    min(T.frame_index), max(T.frame_index), frameGapCount);
fprintf('cycle: %d .. %d, duplicate steps: %d, gap events: %d, missing cycle count: %d, backward steps: %d\n', ...
    min(T.cycle), max(T.cycle), cycleDupCount, cycleGapEvents, missingCycleCount, cycleBackCount);
fprintf('nonzero error_code samples: %d\n', nonzeroErrorCount);
fprintf('position range: %.6f .. %.6f rad\n', posMin, posMax);
fprintf('speed range: %.6f .. %.6f rad/s\n', velMin, velMax);
fprintf('torque range: %.6f .. %.6f Nm\n', torqueMin, torqueMax);
fprintf('command position range: %.6f .. %.6f rad\n', min(posCmd), max(posCmd));
fprintf('command speed range: %.6f .. %.6f rad/s\n', min(velCmd), max(velCmd));
fprintf('nominal error RMS: %.6f rad, max abs: %.6f rad\n', rmsErr, maxAbsErr);
fprintf('estimated 1Hz amp: %.6f rad\n', ampEst);
fprintf('estimated phase: %.6f rad, delay: %.3f ms\n', phaseRad, delayMs);
fprintf('estimated offset: %.6f rad\n', offsetRad);

fig = figure('Color', 'w', 'Name', 'HPM sine 0.3 rad 1 Hz 30 s');
tiledlayout(fig, 4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(t, posCmd, 'k--', 'LineWidth', 1.0); hold on;
plot(t, pos, 'b', 'LineWidth', 1.0);
plot(t, posFit, 'r:', 'LineWidth', 1.0);
grid on;
ylabel('pos rad');
legend('cmd', 'feedback', '1Hz fit', 'Location', 'best');
title('Position');

nexttile;
plot(t, posErr, 'Color', [0.6 0.1 0.1], 'LineWidth', 1.0);
grid on;
ylabel('err rad');
title(sprintf('Nominal tracking error, RMS %.5f rad', rmsErr));

nexttile;
plot(t, velCmd, 'k--', 'LineWidth', 1.0); hold on;
plot(t, vel, 'Color', [0.1 0.45 0.1], 'LineWidth', 1.0);
grid on;
ylabel('speed rad/s');
legend('cmd', 'feedback', 'Location', 'best');
title('Speed');

nexttile;
plot(t, torque, 'Color', [0.45 0.2 0.65], 'LineWidth', 1.0);
grid on;
xlabel('time s');
ylabel('torque Nm');
title('Torque');

pngFile = fullfile(outDir, 'hpm_sine_0p3_1hz_30s_analysis.png');
saveas(fig, pngFile);
fprintf('saved figure: %s\n', pngFile);
fprintf('saved summary: %s\n', summaryFile);
