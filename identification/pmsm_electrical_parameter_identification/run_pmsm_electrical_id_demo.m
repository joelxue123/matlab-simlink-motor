function result = run_pmsm_electrical_id_demo()
%RUN_PMSM_ELECTRICAL_ID_DEMO End-to-end electrical ID demo.

thisDir = fileparts(mfilename("fullpath"));
oldPath = path;
cleanup = onCleanup(@() path(oldPath));
addpath(thisDir);

cfg = pmsm_electrical_id_config();
data = synthesize_pmsm_electrical_id_data(cfg);
result = identify_pmsm_electrical_params(data, cfg);

resultsDir = fullfile(thisDir, "results");
if ~exist(resultsDir, "dir")
    mkdir(resultsDir);
end

writetable(data.step, fullfile(resultsDir, "electrical_step_data.csv"));
writetable(data.flux, fullfile(resultsDir, "flux_spin_data.csv"));
writetable(data.angle, fullfile(resultsDir, "encoder_angle_data.csv"));
write_report(fullfile(resultsDir, "pmsm_electrical_id_report.txt"), result, cfg);
plot_pmsm_electrical_id_results(data, result, cfg);

fprintf("\nPMSM electrical parameter identification demo\n");
fprintf("---------------------------------------------\n");
fprintf("Rs true / estimate   : %.6g / %.6g ohm (error %.3f%%)\n", ...
    cfg.motor.Rs_ohm, result.Rs_ohm, 100 * result.relative_error.Rs);
fprintf("Ld true / estimate   : %.6g / %.6g H (error %.3f%%)\n", ...
    cfg.motor.Ld_H, result.Ld_H, 100 * result.relative_error.Ld);
fprintf("Lq true / estimate   : %.6g / %.6g H (error %.3f%%)\n", ...
    cfg.motor.Lq_H, result.Lq_H, 100 * result.relative_error.Lq);
fprintf("psi true / estimate  : %.6g / %.6g Wb (error %.3f%%)\n", ...
    cfg.motor.psi_f_Wb, result.psi_f_Wb, 100 * result.relative_error.psi_f);
fprintf("encoder offset true / estimate: %.6g / %.6g rad (error %.6g rad)\n", ...
    cfg.motor.encoderOffset_rad, result.encoderOffset_rad, ...
    result.relative_error.encoderOffset);
fprintf("Results written to   : %s\n\n", resultsDir);
end

function write_report(pathname, result, cfg)
fid = fopen(pathname, "w");
assert(fid > 0, "Unable to open report: %s", pathname);
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, "PMSM electrical parameter identification demo\n\n");
fprintf(fid, "Truth and estimate\n");
fprintf(fid, "  Rs_ohm        true %.9g estimate %.9g error %.6f\n", ...
    cfg.motor.Rs_ohm, result.Rs_ohm, result.relative_error.Rs);
fprintf(fid, "  Ld_H          true %.9g estimate %.9g error %.6f\n", ...
    cfg.motor.Ld_H, result.Ld_H, result.relative_error.Ld);
fprintf(fid, "  Lq_H          true %.9g estimate %.9g error %.6f\n", ...
    cfg.motor.Lq_H, result.Lq_H, result.relative_error.Lq);
fprintf(fid, "  psi_f_Wb      true %.9g estimate %.9g error %.6f\n", ...
    cfg.motor.psi_f_Wb, result.psi_f_Wb, result.relative_error.psi_f);
fprintf(fid, "  enc_offset    true %.9g estimate %.9g abs_error %.9g\n\n", ...
    cfg.motor.encoderOffset_rad, result.encoderOffset_rad, ...
    result.relative_error.encoderOffset);

fprintf(fid, "Encoder residual\n");
fprintf(fid, "  residual_rms_rad = %.9g\n", result.angleResidualRms_rad);
fprintf(fid, "  residual_1x_rad  = %.9g\n", result.encoderResidual1x_rad);
fprintf(fid, "  residual_2x_rad  = %.9g\n", result.encoderResidual2x_rad);
end
