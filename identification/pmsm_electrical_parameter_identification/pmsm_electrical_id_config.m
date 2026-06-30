function cfg = pmsm_electrical_id_config()
%PMSM_ELECTRICAL_ID_CONFIG Customer-editable synthetic test config.

cfg.motor.Rs_ohm = 0.420;
cfg.motor.Ld_H = 250e-6;
cfg.motor.Lq_H = 360e-6;
cfg.motor.psi_f_Wb = 0.0180;
cfg.motor.polePairs = 7;
cfg.motor.encoderOffset_rad = 0.350;
cfg.motor.encoderNonlinear1_rad = 0.020;
cfg.motor.encoderNonlinear2_rad = 0.010;

cfg.step.Ts_s = 50e-6;
cfg.step.duration_s = 0.050;
cfg.step.vdStep_V = 1.50;
cfg.step.vqStep_V = 1.80;

cfg.flux.we_radps = [100; 180; 260; 340; 420];
cfg.flux.samplesPerSpeed = 200;

cfg.angle.sampleCount = 4000;
cfg.angle.electricalTurns = 6;

cfg.noise.currentStd_A = 0.002;
cfg.noise.voltageStd_V = 0.003;
cfg.noise.angleStd_rad = 0.0015;
cfg.noise.randomSeed = 19;

cfg.ident.edgeSkip_s = 1.0e-3;
cfg.ident.tailWindow_s = 8.0e-3;
cfg.ident.expFitMin = 0.05;
cfg.ident.expFitMax = 0.90;
end
