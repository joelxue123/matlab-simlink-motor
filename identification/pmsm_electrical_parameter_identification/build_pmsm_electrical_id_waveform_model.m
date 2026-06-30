function build_pmsm_electrical_id_waveform_model()
%BUILD_PMSM_ELECTRICAL_ID_WAVEFORM_MODEL Create visual Simulink harness.
% The model is a waveform inspection harness for the synthetic electrical ID
% data. From Workspace blocks feed scopes so detailed waveforms can be viewed
% before building a full plant/test-bench model.

thisDir = fileparts(mfilename("fullpath"));
oldPath = path;
cleanup = onCleanup(@() path(oldPath));
addpath(thisDir);

cfg = pmsm_electrical_id_config();
data = synthesize_pmsm_electrical_id_data(cfg);
result = identify_pmsm_electrical_params(data, cfg);

model = "pmsm_electrical_id_waveform_model";
modelPath = fullfile(thisDir, model + ".slx");

if bdIsLoaded(model)
    close_system(model, 0);
end
if exist(modelPath, "file")
    delete(modelPath);
end

new_system(model);
open_system(model);

set_param(model, ...
    "StopTime", "0.45", ...
    "Solver", "FixedStepDiscrete", ...
    "FixedStep", "5e-5", ...
    "SaveOutput", "off", ...
    "SignalLogging", "on");

assign_waveform_variables(model, data, result, cfg);

annotationText = sprintf([ ...
    'PMSM Electrical ID Waveform Harness\n', ...
    'Step test: id step from vd=%.3g V, iq step from vq=%.3g V\n', ...
    'Flux spin: id ~= 0 A, iq ~= 0 A, we=[%s] rad/s\n', ...
    'Encoder: offset=%.3f rad, residual includes 1x=%.3f rad and 2x=%.3f rad'], ...
    cfg.step.vdStep_V, cfg.step.vqStep_V, char(join(string(cfg.flux.we_radps.'), ", ")), ...
    cfg.motor.encoderOffset_rad, cfg.motor.encoderNonlinear1_rad, ...
    cfg.motor.encoderNonlinear2_rad);
add_annotation(model, [40, 20, 760, 120], annotationText);

add_step_subsystem(model);
add_flux_subsystem(model);
add_encoder_subsystem(model);

save_system(model, modelPath);
write_test_condition_file(thisDir, cfg, result);

fprintf("Generated Simulink waveform model:\n  %s\n", modelPath);
fprintf("Recorded test conditions:\n  %s\n", fullfile(thisDir, "results", "pmsm_electrical_id_test_conditions.txt"));
end

function assign_waveform_variables(model, data, result, cfg)
mws = get_param(model, "ModelWorkspace");

stepD = data.step(data.step.axis == "d", :);
stepQ = data.step(data.step.axis == "q", :);
assignin(mws, "step_d_current", [stepD.t_s, stepD.i_A]);
assignin(mws, "step_q_current", [stepQ.t_s, stepQ.i_A]);
assignin(mws, "step_d_voltage", [stepD.t_s, stepD.v_V]);
assignin(mws, "step_q_voltage", [stepQ.t_s, stepQ.v_V]);

fluxTs = 1e-3;
fluxTime = (0:height(data.flux) - 1).' * fluxTs;
assignin(mws, "flux_we", [fluxTime, data.flux.we_radps]);
assignin(mws, "flux_vq", [fluxTime, data.flux.vq_V]);
assignin(mws, "flux_id", [fluxTime, data.flux.id_A]);
assignin(mws, "flux_iq", [fluxTime, data.flux.iq_A]);
assignin(mws, "flux_psi_est", [fluxTime, result.psiSamples]);

angleTs = 1e-4;
angleTime = (0:height(data.angle) - 1).' * angleTs;
angleDelta = wrap_to_pi(data.angle.theta_encoder_rad - data.angle.theta_sensorless_rad);
assignin(mws, "angle_true", [angleTime, data.angle.theta_true_rad]);
assignin(mws, "angle_encoder", [angleTime, data.angle.theta_encoder_rad]);
assignin(mws, "angle_sensorless", [angleTime, data.angle.theta_sensorless_rad]);
assignin(mws, "angle_delta", [angleTime, angleDelta]);
assignin(mws, "angle_residual", [angleTime, result.angleResidual]);
end

function add_step_subsystem(model)
sub = model + "/Standstill_RL_Step_Test";
add_block("built-in/Subsystem", sub, "Position", [60, 170, 360, 390]);
open_system(sub);

add_annotation(sub, [25, 15, 470, 55], ...
    "Standstill d/q voltage step. Use current exponential shape to estimate Rs/Ld/Lq.");

add_from_workspace(sub, "vd_step_V", "step_d_voltage", [40, 80, 180, 110]);
add_from_workspace(sub, "id_meas_A", "step_d_current", [40, 130, 180, 160]);
add_from_workspace(sub, "vq_step_V", "step_q_voltage", [40, 200, 180, 230]);
add_from_workspace(sub, "iq_meas_A", "step_q_current", [40, 250, 180, 280]);

add_mux_scope(sub, "StepScope", 4, [260, 130, 300, 250], [380, 120, 520, 260]);
connect_to_mux(sub, ["vd_step_V", "id_meas_A", "vq_step_V", "iq_meas_A"], "StepScope_mux");
end

function add_flux_subsystem(model)
sub = model + "/Flux_Linkage_Spin_Test";
add_block("built-in/Subsystem", sub, "Position", [430, 170, 730, 390]);
open_system(sub);

add_annotation(sub, [25, 15, 500, 55], ...
    "Spin test at id ~= 0 and iq ~= 0. vq/we slope gives psi_f.");

add_from_workspace(sub, "we_radps", "flux_we", [40, 80, 180, 110]);
add_from_workspace(sub, "vq_V", "flux_vq", [40, 130, 180, 160]);
add_from_workspace(sub, "id_A", "flux_id", [40, 200, 180, 230]);
add_from_workspace(sub, "iq_A", "flux_iq", [40, 250, 180, 280]);
add_from_workspace(sub, "psi_est_Wb", "flux_psi_est", [40, 320, 180, 350]);

add_mux_scope(sub, "FluxScope", 5, [260, 135, 300, 275], [380, 125, 520, 285]);
connect_to_mux(sub, ["we_radps", "vq_V", "id_A", "iq_A", "psi_est_Wb"], "FluxScope_mux");
end

function add_encoder_subsystem(model)
sub = model + "/Encoder_Alignment_Test";
add_block("built-in/Subsystem", sub, "Position", [800, 170, 1100, 390]);
open_system(sub);

add_annotation(sub, [25, 15, 520, 55], ...
    "Sensorless/sensored angle comparison. Mean delta gives offset; residual gives encoder nonlinearity.");

add_from_workspace(sub, "theta_true", "angle_true", [40, 80, 180, 110]);
add_from_workspace(sub, "theta_encoder", "angle_encoder", [40, 130, 180, 160]);
add_from_workspace(sub, "theta_sensorless", "angle_sensorless", [40, 180, 180, 210]);
add_from_workspace(sub, "theta_delta", "angle_delta", [40, 250, 180, 280]);
add_from_workspace(sub, "theta_residual", "angle_residual", [40, 300, 180, 330]);

add_mux_scope(sub, "AngleScope", 5, [260, 135, 300, 275], [380, 125, 520, 285]);
connect_to_mux(sub, ["theta_true", "theta_encoder", "theta_sensorless", "theta_delta", "theta_residual"], "AngleScope_mux");
end

function add_from_workspace(parent, name, variableName, pos)
add_block("simulink/Sources/From Workspace", parent + "/" + name, ...
    "Position", pos, ...
    "VariableName", variableName, ...
    "SampleTime", "0", ...
    "Interpolate", "on");
end

function add_mux_scope(parent, name, inputCount, muxPos, scopePos)
muxName = name + "_mux";
add_block("simulink/Signal Routing/Mux", parent + "/" + muxName, ...
    "Position", muxPos, ...
    "Inputs", num2str(inputCount));
add_block("simulink/Sinks/Scope", parent + "/" + name, ...
    "Position", scopePos);
add_line(parent, muxName + "/1", name + "/1", "autorouting", "on");
end

function connect_to_mux(parent, sourceNames, muxName)
for k = 1:numel(sourceNames)
    add_line(parent, sourceNames(k) + "/1", muxName + "/" + string(k), "autorouting", "on");
end
end

function add_annotation(parent, pos, text)
ann = Simulink.Annotation(parent, text);
ann.Position = pos;
end

function write_test_condition_file(thisDir, cfg, result)
resultsDir = fullfile(thisDir, "results");
if ~exist(resultsDir, "dir")
    mkdir(resultsDir);
end

pathName = fullfile(resultsDir, "pmsm_electrical_id_test_conditions.txt");
fid = fopen(pathName, "w");
assert(fid > 0, "Unable to open %s", pathName);
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, "PMSM electrical ID test conditions\n\n");
fprintf(fid, "Standstill RL step test\n");
fprintf(fid, "  d-axis: id starts near 0 A, vd_step = %.9g V, we = 0 rad/s\n", cfg.step.vdStep_V);
fprintf(fid, "  q-axis: iq starts near 0 A, vq_step = %.9g V, we = 0 rad/s\n", cfg.step.vqStep_V);
fprintf(fid, "  Ts = %.9g s, duration = %.9g s\n\n", cfg.step.Ts_s, cfg.step.duration_s);

fprintf(fid, "Flux linkage spin test\n");
fprintf(fid, "  id command ~= 0 A\n");
fprintf(fid, "  iq command ~= 0 A\n");
fprintf(fid, "  we test points rad/s = %s\n", char(join(string(cfg.flux.we_radps.'), ", ")));
fprintf(fid, "  samplesPerSpeed = %d\n\n", cfg.flux.samplesPerSpeed);

fprintf(fid, "Encoder alignment test\n");
fprintf(fid, "  electrical turns = %d\n", cfg.angle.electricalTurns);
fprintf(fid, "  sample count = %d\n", cfg.angle.sampleCount);
fprintf(fid, "  true offset = %.9g rad\n", cfg.motor.encoderOffset_rad);
fprintf(fid, "  estimated offset = %.9g rad\n", result.encoderOffset_rad);
fprintf(fid, "  offset error = %.9g rad\n", result.relative_error.encoderOffset);
fprintf(fid, "  residual 1x = %.9g rad\n", result.encoderResidual1x_rad);
fprintf(fid, "  residual 2x = %.9g rad\n", result.encoderResidual2x_rad);
end

function y = wrap_to_pi(x)
y = mod(x + pi, 2 * pi) - pi;
end
