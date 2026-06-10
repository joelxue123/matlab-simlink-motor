function metrics = run_current_sensor_harmonic_regression_test()
%RUN_CURRENT_SENSOR_HARMONIC_REGRESSION_TEST Current sensing harmonic checks.
% This test verifies the expected dq signatures:
%   current offset       -> 1x electrical ripple
%   current gain mismatch -> 2x electrical ripple

thisDir = fileparts(mfilename("fullpath"));
resultsDir = fullfile(thisDir, "results");
if ~exist(resultsDir, "dir")
    mkdir(resultsDir);
end

cfg = local_config();
base = synthesize_balanced_currents(cfg);
cases = build_fault_cases(cfg, base);
metrics = evaluate_cases(cfg, cases);

assert_regression_pass(metrics, cfg);

writetable(metrics, fullfile(resultsDir, "current_sensor_harmonic_metrics.csv"));
write_report(fullfile(resultsDir, "current_sensor_harmonic_regression_report.txt"), metrics, cfg);
plot_metrics(metrics, cfg, fullfile(resultsDir, "current_sensor_harmonic_regression.png"));
plot_dq_timeseries(cases, cfg, fullfile(resultsDir, "current_sensor_fault_dq_timeseries.png"));

fprintf("\nCurrent sensor harmonic regression passed.\n");
fprintf("Electrical frequency: %.3f Hz\n", cfg.fe_Hz);
fprintf("1x/2x metrics written to:\n");
fprintf("  %s\n", fullfile(resultsDir, "current_sensor_harmonic_metrics.csv"));
fprintf("Figures written to:\n");
fprintf("  %s\n", fullfile(resultsDir, "current_sensor_harmonic_regression.png"));
fprintf("  %s\n\n", fullfile(resultsDir, "current_sensor_fault_dq_timeseries.png"));
end

function cfg = local_config()
cfg.fs_Hz = 20000;
cfg.duration_s = 1.0;
cfg.fe_Hz = 50;
cfg.polePairs = 7;
cfg.idRef_A = 0.0;
cfg.iqRef_A = 1.5;
cfg.offsetFault_A = [0.080, -0.030, 0.020];
cfg.gainFault = [1.060, 0.970, 1.000];
cfg.noiseStd_A = 0.030;
cfg.pwmRipple_A = 0.030;
cfg.pwmFreq_Hz = 1000;
cfg.randomSeed = 11;
cfg.pass.nominalMax_A = 1.0e-10;
cfg.pass.minFaultRipple_A = 0.020;
cfg.pass.offsetDominanceRatio = 5.0;
cfg.pass.gainDominanceRatio = 5.0;
end

function base = synthesize_balanced_currents(cfg)
t = (0:1 / cfg.fs_Hz:cfg.duration_s - 1 / cfg.fs_Hz).';
theta = 2 * pi * cfg.fe_Hz * t;

alpha = cfg.idRef_A * cos(theta) - cfg.iqRef_A * sin(theta);
beta = cfg.idRef_A * sin(theta) + cfg.iqRef_A * cos(theta);

base.t_s = t;
base.theta_e_rad = theta;
base.ia_A = alpha;
base.ib_A = -0.5 * alpha + sqrt(3) / 2 * beta;
base.ic_A = -0.5 * alpha - sqrt(3) / 2 * beta;
end

function cases = build_fault_cases(cfg, base)
rng(cfg.randomSeed);

cases = make_case("nominal", base, base.ia_A, base.ib_A, base.ic_A);

cases(end + 1) = make_case("offset_fault", base, ...
    base.ia_A + cfg.offsetFault_A(1), ...
    base.ib_A + cfg.offsetFault_A(2), ...
    base.ic_A + cfg.offsetFault_A(3));

cases(end + 1) = make_case("gain_mismatch_fault", base, ...
    cfg.gainFault(1) * base.ia_A, ...
    cfg.gainFault(2) * base.ib_A, ...
    cfg.gainFault(3) * base.ic_A);

cases(end + 1) = make_case("noise_fault", base, ...
    base.ia_A + cfg.noiseStd_A * randn(size(base.t_s)), ...
    base.ib_A + cfg.noiseStd_A * randn(size(base.t_s)), ...
    base.ic_A + cfg.noiseStd_A * randn(size(base.t_s)));

phase = 2 * pi * cfg.pwmFreq_Hz * base.t_s;
cases(end + 1) = make_case("pwm_ripple_fault", base, ...
    base.ia_A + cfg.pwmRipple_A * sin(phase), ...
    base.ib_A + cfg.pwmRipple_A * sin(phase - 2 * pi / 3), ...
    base.ic_A + cfg.pwmRipple_A * sin(phase + 2 * pi / 3));
end

function c = make_case(name, base, ia, ib, ic)
[alpha, beta] = clarke(ia, ib, ic);
[id, iq] = park(alpha, beta, base.theta_e_rad);

c.name = char(name);
c.t_s = base.t_s;
c.theta_e_rad = base.theta_e_rad;
c.ia_A = ia;
c.ib_A = ib;
c.ic_A = ic;
c.id_A = id;
c.iq_A = iq;
end

function metrics = evaluate_cases(cfg, cases)
caseName = strings(numel(cases), 1);
h1_A = zeros(numel(cases), 1);
h2_A = zeros(numel(cases), 1);
h1_id_A = zeros(numel(cases), 1);
h1_iq_A = zeros(numel(cases), 1);
h2_id_A = zeros(numel(cases), 1);
h2_iq_A = zeros(numel(cases), 1);
rms_ac_id_A = zeros(numel(cases), 1);
rms_ac_iq_A = zeros(numel(cases), 1);

for k = 1:numel(cases)
    caseName(k) = string(cases(k).name);
    idAc = cases(k).id_A - mean(cases(k).id_A);
    iqAc = cases(k).iq_A - mean(cases(k).iq_A);

    h1_id_A(k) = harmonic_amplitude(idAc, cfg.fs_Hz, cfg.fe_Hz);
    h1_iq_A(k) = harmonic_amplitude(iqAc, cfg.fs_Hz, cfg.fe_Hz);
    h2_id_A(k) = harmonic_amplitude(idAc, cfg.fs_Hz, 2 * cfg.fe_Hz);
    h2_iq_A(k) = harmonic_amplitude(iqAc, cfg.fs_Hz, 2 * cfg.fe_Hz);

    h1_A(k) = hypot(h1_id_A(k), h1_iq_A(k));
    h2_A(k) = hypot(h2_id_A(k), h2_iq_A(k));
    rms_ac_id_A(k) = rms_no_toolbox(idAc);
    rms_ac_iq_A(k) = rms_no_toolbox(iqAc);
end

metrics = table(caseName, h1_A, h2_A, h1_id_A, h1_iq_A, h2_id_A, h2_iq_A, ...
    rms_ac_id_A, rms_ac_iq_A, ...
    'VariableNames', {'case_name', 'h1_A', 'h2_A', 'h1_id_A', 'h1_iq_A', ...
    'h2_id_A', 'h2_iq_A', 'rms_ac_id_A', 'rms_ac_iq_A'});
end

function assert_regression_pass(metrics, cfg)
nominal = metrics(metrics.case_name == "nominal", :);
offset = metrics(metrics.case_name == "offset_fault", :);
gain = metrics(metrics.case_name == "gain_mismatch_fault", :);

assert(nominal.h1_A < cfg.pass.nominalMax_A && nominal.h2_A < cfg.pass.nominalMax_A, ...
    "Nominal current case has unexpected 1x/2x ripple.");

assert(offset.h1_A > cfg.pass.minFaultRipple_A, ...
    "Offset fault did not create enough 1x electrical ripple.");
assert(offset.h1_A > cfg.pass.offsetDominanceRatio * max(offset.h2_A, eps), ...
    "Offset fault is not dominated by 1x electrical ripple.");

assert(gain.h2_A > cfg.pass.minFaultRipple_A, ...
    "Gain mismatch fault did not create enough 2x electrical ripple.");
assert(gain.h2_A > cfg.pass.gainDominanceRatio * max(gain.h1_A, eps), ...
    "Gain mismatch fault is not dominated by 2x electrical ripple.");
end

function write_report(pathname, metrics, cfg)
fid = fopen(pathname, "w");
assert(fid > 0, "Unable to open report: %s", pathname);
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, "Current sensor harmonic regression\n\n");
fprintf(fid, "fs_Hz       = %.9g\n", cfg.fs_Hz);
fprintf(fid, "fe_Hz       = %.9g\n", cfg.fe_Hz);
fprintf(fid, "polePairs   = %d\n", cfg.polePairs);
fprintf(fid, "fm_Hz       = %.9g\n\n", cfg.fe_Hz / cfg.polePairs);

fprintf(fid, "Rules\n");
fprintf(fid, "  offset fault        -> 1x electrical ripple in dq\n");
fprintf(fid, "  gain mismatch fault -> 2x electrical ripple in dq\n");
fprintf(fid, "  noise fault         -> broadband, not a fixed harmonic\n\n");

for k = 1:height(metrics)
    fprintf(fid, "%s\n", metrics.case_name(k));
    fprintf(fid, "  h1_A         = %.9g\n", metrics.h1_A(k));
    fprintf(fid, "  h2_A         = %.9g\n", metrics.h2_A(k));
    fprintf(fid, "  rms_ac_id_A  = %.9g\n", metrics.rms_ac_id_A(k));
    fprintf(fid, "  rms_ac_iq_A  = %.9g\n\n", metrics.rms_ac_iq_A(k));
end
end

function plot_metrics(metrics, cfg, outputPath)
fig = figure("Visible", "off", "Color", "w", "Position", [100, 100, 1100, 700]);
caseLabels = categorical(metrics.case_name, metrics.case_name, 'Ordinal', true);

subplot(2, 1, 1);
bar(caseLabels, [metrics.h1_A, metrics.h2_A]);
set(gca, "TickLabelInterpreter", "none");
grid on;
ylabel("dq ripple amplitude (A)");
legend("1x electrical", "2x electrical", "Location", "best");
title("Current sensor harmonic regression");

subplot(2, 1, 2);
bar(caseLabels, [metrics.rms_ac_id_A, metrics.rms_ac_iq_A]);
set(gca, "TickLabelInterpreter", "none");
grid on;
ylabel("AC RMS (A)");
legend("id ac rms", "iq ac rms", "Location", "best");
xlabel(sprintf("f_e = %.1f Hz, pole pairs = %d", cfg.fe_Hz, cfg.polePairs));

saveas(fig, outputPath);
close(fig);
end

function plot_dq_timeseries(cases, cfg, outputPath)
fig = figure("Visible", "off", "Color", "w", "Position", [100, 100, 1200, 850]);
namesToPlot = ["offset_fault", "gain_mismatch_fault", "noise_fault", "pwm_ripple_fault"];

for k = 1:numel(namesToPlot)
    idx = find(strcmp({cases.name}, char(namesToPlot(k))), 1);
    c = cases(idx);
    subplot(numel(namesToPlot), 1, k);
    plot(c.t_s, c.id_A - mean(c.id_A), "b", "LineWidth", 0.8); hold on;
    plot(c.t_s, c.iq_A - mean(c.iq_A), "r", "LineWidth", 0.8);
    grid on;
    xlim([0, min(0.12, cfg.duration_s)]);
    ylabel("AC dq (A)");
    title(strrep(c.name, "_", " "));
    if k == 1
        legend("id ac", "iq ac", "Location", "best");
    end
end

xlabel("Time (s)");
saveas(fig, outputPath);
close(fig);
end

function [alpha, beta] = clarke(ia, ib, ic)
alpha = 2 / 3 * (ia - 0.5 * ib - 0.5 * ic);
beta = 2 / 3 * (sqrt(3) / 2 * ib - sqrt(3) / 2 * ic);
end

function [id, iq] = park(alpha, beta, theta)
id = alpha .* cos(theta) + beta .* sin(theta);
iq = -alpha .* sin(theta) + beta .* cos(theta);
end

function amp = harmonic_amplitude(x, fs, targetHz)
n = numel(x);
t = (0:n - 1).' / fs;
coef = sum(x(:) .* exp(-1j * 2 * pi * targetHz * t)) / n;
amp = 2 * abs(coef);
end

function y = rms_no_toolbox(x)
y = sqrt(mean(x(:).^2));
end
