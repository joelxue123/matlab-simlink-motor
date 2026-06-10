function plot_coreless_motor_identification_results()
%PLOT_CORELESS_MOTOR_IDENTIFICATION_RESULTS Plot generated CSV results.
% Reads the CSV files produced by run_coreless_motor_identification_demo.m and
% writes PNG figures into the results folder.

thisDir = fileparts(mfilename("fullpath"));
oldPath = path;
cleanup = onCleanup(@() path(oldPath));
addpath(thisDir);

cfg = coreless_motor_12v_config();
resultsDir = fullfile(thisDir, "results");
dataPath = fullfile(resultsDir, "synthetic_coreless_motor_12v_data.csv");
windowPath = fullfile(resultsDir, "identification_windows.csv");

assert(exist(dataPath, "file") == 2, "Missing data CSV: %s", dataPath);
assert(exist(windowPath, "file") == 2, "Missing window CSV: %s", windowPath);

data = readtable(dataPath);
windows = readtable(windowPath);

plot_time_series(data, cfg, fullfile(resultsDir, "coreless_identification_timeseries.png"));
plot_windows(windows, cfg, fullfile(resultsDir, "coreless_identification_windows.png"));

fprintf("Generated PNG figures:\n");
fprintf("  %s\n", fullfile(resultsDir, "coreless_identification_timeseries.png"));
fprintf("  %s\n", fullfile(resultsDir, "coreless_identification_windows.png"));
end

function plot_time_series(data, cfg, outputPath)
fig = figure("Visible", "off", "Color", "w", "Position", [100, 100, 1200, 900]);
t = data.t_s;

subplot(5, 1, 1);
plot(t, data.i_cmd_A, "k--", "LineWidth", 1.0); hold on;
plot(t, data.i_meas_A, "b", "LineWidth", 1.0);
grid on;
ylabel("Current (A)");
legend("i cmd", "i meas", "Location", "best");
title("12V coreless motor synthetic identification process");

subplot(5, 1, 2);
plot(t, data.torque_nm, "m", "LineWidth", 1.0);
grid on;
ylabel("Torque (Nm)");

subplot(5, 1, 3);
plot(t, data.speed_rad_s, "r", "LineWidth", 1.0);
grid on;
ylabel("Speed (rad/s)");

subplot(5, 1, 4);
plot(t, data.position_rad, "Color", [0.0, 0.45, 0.2], "LineWidth", 1.0);
grid on;
ylabel("Position (rad)");

subplot(5, 1, 5);
plot(t, data.v_required_V, "Color", [0.2, 0.2, 0.2], "LineWidth", 1.0); hold on;
yline(cfg.driver.Vdc_V, "r--", "LineWidth", 0.8);
yline(-cfg.driver.Vdc_V, "r--", "LineWidth", 0.8);
satIdx = data.voltage_saturated ~= 0;
if any(satIdx)
    plot(t(satIdx), data.v_required_V(satIdx), "ro", "MarkerSize", 3);
end
grid on;
ylabel("V req (V)");
xlabel("Time (s)");

saveas(fig, outputPath);
close(fig);
end

function plot_windows(windows, cfg, outputPath)
fig = figure("Visible", "off", "Color", "w", "Position", [100, 100, 1200, 850]);
isPulse = windows.is_pulse_segment ~= 0;

subplot(3, 1, 1);
scatter(windows.speed_mean_rad_s(isPulse), windows.accel_fit_rad_s2(isPulse), ...
    35, "b", "filled"); hold on;
scatter(windows.speed_mean_rad_s(~isPulse), windows.accel_fit_rad_s2(~isPulse), ...
    35, [0.9, 0.35, 0.0], "filled");
grid on;
ylabel("Accel fit (rad/s^2)");
xlabel("Mean speed (rad/s)");
legend("pulse", "coast", "Location", "best");
title("Identification windows");

subplot(3, 1, 2);
scatter(windows.accel_fit_rad_s2(isPulse), windows.torque_mean_Nm(isPulse), ...
    35, "b", "filled"); hold on;
scatter(windows.accel_fit_rad_s2(~isPulse), windows.torque_mean_Nm(~isPulse), ...
    35, [0.9, 0.35, 0.0], "filled");
grid on;
ylabel("Mean torque (Nm)");
xlabel("Accel fit (rad/s^2)");
legend("pulse", "coast", "Location", "best");

subplot(3, 1, 3);
pulseWindows = windows(isPulse, :);
plot(pulseWindows.t_start_s, pulseWindows.naive_J_kgm2, "bo-", "LineWidth", 1.0); hold on;
yline(cfg.motor.J_kgm2, "k--", "True J", "LineWidth", 1.0);
grid on;
ylabel("Naive J per pulse");
xlabel("Window start time (s)");

saveas(fig, outputPath);
close(fig);
end
