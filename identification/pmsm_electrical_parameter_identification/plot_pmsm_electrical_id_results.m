function plot_pmsm_electrical_id_results(data, result, cfg)
%PLOT_PMSM_ELECTRICAL_ID_RESULTS Plot estimator inputs and results.

thisDir = fileparts(mfilename("fullpath"));
resultsDir = fullfile(thisDir, "results");
if ~exist(resultsDir, "dir")
    mkdir(resultsDir);
end

fig = figure("Visible", "off", "Color", "w", "Position", [100, 100, 1200, 850]);

subplot(2, 2, 1);
hold on;
plot_axis_step(data.step, "d");
plot_axis_step(data.step, "q");
grid on;
xlabel("Time (s)");
ylabel("Current (A)");
legend("d-axis", "q-axis", "Location", "best");
title("Standstill voltage step response");

subplot(2, 2, 2);
scatter(data.flux.we_radps, data.flux.vq_V, 15, "filled"); hold on;
weLine = linspace(min(data.flux.we_radps), max(data.flux.we_radps), 100).';
plot(weLine, result.psi_f_Wb * weLine, "r", "LineWidth", 1.2);
grid on;
xlabel("Electrical speed we (rad/s)");
ylabel("vq (V)");
title("Flux linkage from spin test");
legend("samples", "psi_f * we", "Location", "best");

subplot(2, 2, 3);
theta = data.angle.theta_true_rad;
residual = result.angleResidual;
plot(theta, residual, ".", "MarkerSize", 4);
grid on;
xlabel("Electrical angle (rad)");
ylabel("Residual (rad)");
title("Encoder residual after offset removal");

subplot(2, 2, 4);
bar(categorical(["Rs", "Ld", "Lq", "psi"]), ...
    100 * [result.relative_error.Rs, result.relative_error.Ld, ...
    result.relative_error.Lq, result.relative_error.psi_f]);
grid on;
ylabel("Relative error (%)");
title("Estimator error");

saveas(fig, fullfile(resultsDir, "pmsm_electrical_id_summary.png"));
close(fig);
end

function plot_axis_step(stepData, axisName)
idx = stepData.axis == string(axisName);
plot(stepData.t_s(idx), stepData.i_A(idx), "LineWidth", 1.0);
end
