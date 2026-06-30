function result = run_current_torque_curve()
%RUN_CURRENT_TORQUE_CURVE Build a current-torque curve from measured points.
% Run from this folder or through:
%   matlab -batch "run('motor_performance_characterization/run_current_torque_curve.m')"

thisDir = fileparts(mfilename("fullpath"));
resultsDir = fullfile(thisDir, "results");
if ~exist(resultsDir, "dir")
    mkdir(resultsDir);
end

cfg = local_config();
data = build_data_table(cfg);
result = fit_current_torque_curve(data, cfg);

writetable(data, fullfile(resultsDir, "current_torque_curve_data.csv"));
write_report(fullfile(resultsDir, "current_torque_curve_report.txt"), data, result, cfg);
plot_curve(fullfile(resultsDir, "current_torque_curve.png"), data, result, cfg);

fprintf("\nCurrent-torque curve generated.\n");
fprintf("Measured points      : %d\n", height(data));
fprintf("Kt (fit through origin, Apeak) = %.6f mNm/A_peak\n", result.kt_mNm_per_Apeak);
fprintf("Rated point          : %.3f Apeak -> %.3f mNm\n", ...
    result.rated.peakCurrent_A, cfg.rated.torque_mNm);
fprintf("Peak point           : %.3f Apeak -> %.3f mNm\n", ...
    result.peak.peakCurrent_A, cfg.peak.torque_mNm);
fprintf("Results written to:\n");
fprintf("  %s\n", fullfile(resultsDir, "current_torque_curve_data.csv"));
fprintf("  %s\n", fullfile(resultsDir, "current_torque_curve_report.txt"));
fprintf("  %s\n\n", fullfile(resultsDir, "current_torque_curve.png"));
end

function cfg = local_config()
cfg.phaseCurrentRmsFromPeak = @(x) x / sqrt(2);
cfg.phaseCurrentPeakFromRms = @(x) x * sqrt(2);

cfg.rated.phaseCurrentArms_A = 0.7;
cfg.rated.torque_mNm = 5.09;

cfg.peak.phaseCurrentArms_A = 2.1;
cfg.peak.torque_mNm = 14.67;

cfg.sim.phaseCurrentArms_A = [0.0; cfg.rated.phaseCurrentArms_A; cfg.peak.phaseCurrentArms_A];
cfg.sim.torque_mNm = [0.0; cfg.rated.torque_mNm; cfg.peak.torque_mNm];

cfg.meas.busVoltage_V = 12.0 * ones(11, 1);
cfg.meas.busCurrent_A = [ ...
    0.27; 0.42; 0.59; 0.73; 0.91; 1.05; 1.28; 1.48; 1.78; 2.08; 2.35];
cfg.meas.phaseCurrentPeak_A = [ ...
    0.30; 0.47; 0.70; 0.85; 1.00; 1.25; 1.54; 1.84; 2.20; 2.60; 2.95];
cfg.meas.speed_rpm = [ ...
    15000; 15000; 15000; 14400; 13700; 13000; 12000; 11000; 9300; 7800; 6300];
cfg.meas.mass_g = [10; 20; 30; 40; 50; 60; 75; 90; 110; 130; 150];
cfg.meas.torque_mNm = [1.0; 2.0; 3.0; 4.0; 5.0; 6.0; 7.5; 9.0; 11.0; 13.0; 15.0];
end

function data = build_data_table(cfg)
n = numel(cfg.meas.torque_mNm);

phaseCurrentPeak_A = cfg.meas.phaseCurrentPeak_A;
phaseCurrentArms_A = cfg.phaseCurrentRmsFromPeak(phaseCurrentPeak_A);
inputPower_W = cfg.meas.busVoltage_V .* cfg.meas.busCurrent_A;
omega_rad_s = cfg.meas.speed_rpm * 2 * pi / 60;
outputPower_W = cfg.meas.torque_mNm * 1e-3 .* omega_rad_s;
efficiency_pct = 100 * outputPower_W ./ inputPower_W;

data = table;
data.point_id = (1:n).';
data.bus_voltage_V = cfg.meas.busVoltage_V;
data.bus_current_A = cfg.meas.busCurrent_A;
data.input_power_W = inputPower_W;
data.phase_current_peak_A = phaseCurrentPeak_A;
data.phase_current_rms_A = phaseCurrentArms_A;
data.speed_rpm = cfg.meas.speed_rpm;
data.mass_g = cfg.meas.mass_g;
data.torque_mNm = cfg.meas.torque_mNm;
data.output_power_W = outputPower_W;
data.efficiency_pct = efficiency_pct;
end

function result = fit_current_torque_curve(data, cfg)
xPeak = data.phase_current_peak_A;
xRms = data.phase_current_rms_A;
y = data.torque_mNm;

result.kt_mNm_per_Apeak = xPeak \ y;
result.kt_mNm_per_Arms = xRms \ y;

peakFit = result.kt_mNm_per_Apeak * xPeak;
rmsFit = result.kt_mNm_per_Arms * xRms;

peakPoly = polyfit(xPeak, y, 1);
rmsPoly = polyfit(xRms, y, 1);

result.peak_poly_slope = peakPoly(1);
result.peak_poly_intercept = peakPoly(2);
result.rms_poly_slope = rmsPoly(1);
result.rms_poly_intercept = rmsPoly(2);
result.r2_origin_peak = compute_r2(y, peakFit);
result.r2_origin_rms = compute_r2(y, rmsFit);
result.r2_poly_peak = compute_r2(y, polyval(peakPoly, xPeak));
result.r2_poly_rms = compute_r2(y, polyval(rmsPoly, xRms));

result.rated.peakCurrent_A = cfg.phaseCurrentPeakFromRms(cfg.rated.phaseCurrentArms_A);
result.rated.phaseCurrentArms_A = cfg.rated.phaseCurrentArms_A;
result.rated.torque_mNm = cfg.rated.torque_mNm;

result.peak.peakCurrent_A = cfg.phaseCurrentPeakFromRms(cfg.peak.phaseCurrentArms_A);
result.peak.phaseCurrentArms_A = cfg.peak.phaseCurrentArms_A;
result.peak.torque_mNm = cfg.peak.torque_mNm;
end

function write_report(reportPath, data, result, cfg)
fid = fopen(reportPath, "w");
assert(fid > 0, "Unable to open report file: %s", reportPath);
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, "Current-torque curve from measured hanging-mass data\n\n");
fprintf(fid, "Assumption\n");
fprintf(fid, "  measured current points are peak current values\n");
fprintf(fid, "  phase_current_rms_A = phase_current_peak_A / sqrt(2)\n\n");

fprintf(fid, "Reference points\n");
fprintf(fid, "  rated : %.9g Arms -> %.9g mNm\n", ...
    cfg.rated.phaseCurrentArms_A, cfg.rated.torque_mNm);
fprintf(fid, "  peak  : %.9g Arms -> %.9g mNm\n\n", ...
    cfg.peak.phaseCurrentArms_A, cfg.peak.torque_mNm);

fprintf(fid, "Fit through origin\n");
fprintf(fid, "  Kt_Apeak = %.9g mNm/A_peak, R2 = %.9g\n", ...
    result.kt_mNm_per_Apeak, result.r2_origin_peak);
fprintf(fid, "  Kt_Arms  = %.9g mNm/A_rms,  R2 = %.9g\n\n", ...
    result.kt_mNm_per_Arms, result.r2_origin_rms);

fprintf(fid, "Affine fit\n");
fprintf(fid, "  torque_mNm = %.9g * Ipeak_A + %.9g, R2 = %.9g\n", ...
    result.peak_poly_slope, result.peak_poly_intercept, result.r2_poly_peak);
fprintf(fid, "  torque_mNm = %.9g * Irms_A  + %.9g, R2 = %.9g\n\n", ...
    result.rms_poly_slope, result.rms_poly_intercept, result.r2_poly_rms);

fprintf(fid, "Measured data with derived power and efficiency\n");
for k = 1:height(data)
    fprintf(fid, ...
        "  %2d: Ipeak=%.3f A, Irms=%.3f A, torque=%.3f mNm, speed=%.0f rpm, Pin=%.3f W, Pout=%.3f W, eta=%.2f%%\n", ...
        data.point_id(k), data.phase_current_peak_A(k), data.phase_current_rms_A(k), ...
        data.torque_mNm(k), data.speed_rpm(k), data.input_power_W(k), ...
        data.output_power_W(k), data.efficiency_pct(k));
end
end

function plot_curve(outputPath, data, result, cfg)
fig = figure("Visible", "off", "Color", "w", "Position", [100, 100, 820, 760]);
ax = axes(fig);
hold(ax, "on");

plot(ax, data.phase_current_peak_A, data.torque_mNm, "-o", ...
    "Color", "b", ...
    "LineWidth", 1.8, ...
    "MarkerSize", 5.5, ...
    "MarkerFaceColor", "b", ...
    "DisplayName", "实测");

plot(ax, cfg.phaseCurrentPeakFromRms(cfg.sim.phaseCurrentArms_A), cfg.sim.torque_mNm, "-o", ...
    "Color", "k", ...
    "LineWidth", 1.8, ...
    "MarkerSize", 5.5, ...
    "MarkerFaceColor", "k", ...
    "DisplayName", "仿真");

grid(ax, "on");
box(ax, "on");
ax.FontSize = 11;
ax.LineWidth = 0.8;
ax.XLim = [0, 4.5];
ax.YLim = [0, 16];
ax.XTick = 0:0.5:4.5;
ax.YTick = 0:2:16;

xlabel(ax, "电流/A", "FontSize", 12);
ylabel(ax, "转矩/mNm", "FontSize", 12);
legend(ax, "Location", "northwest");

set(fig, "PaperPositionMode", "auto");
print(fig, outputPath, "-dpng", "-r160");
close(fig);
end

function r2 = compute_r2(y, yFit)
ssRes = sum((y - yFit).^2);
ssTot = sum((y - mean(y)).^2);
if ssTot <= eps
    r2 = 1.0;
else
    r2 = 1.0 - ssRes / ssTot;
end
end
