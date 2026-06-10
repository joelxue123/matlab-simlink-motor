function cfg = coreless_motor_12v_config()
%CORELESS_MOTOR_12V_CONFIG Customer-editable demo configuration.
% This is a simulation-first configuration for a small 12 V coreless DC
% motor identification sandbox. Keep this file as the first place to tune
% motor, driver, sensor, and experiment assumptions.

cfg.motor.R_ohm = 4.0;
cfg.motor.L_H = 100e-6;
cfg.motor.Ke_V_per_radps = 6.0e-3;
cfg.motor.Kt_Nm_per_A = 6.0e-3;
cfg.motor.J_kgm2 = 1.20e-6;
cfg.motor.B_Nm_per_radps = 2.0e-6;
cfg.motor.Tc_Nm = 1.5e-4;
cfg.motor.Tbias_Nm = 0.0;
cfg.motor.frictionSpeedEps_radps = 0.5;

cfg.driver.Vdc_V = 12.0;
cfg.driver.currentLimit_A = 0.50;
cfg.driver.currentLoopTau_s = 2.0e-3;
cfg.driver.currentDeadband_A = 0.010;
cfg.driver.voltageMargin = 0.95;

cfg.experiment.Ts_s = 1.0e-3;
cfg.experiment.pulseCurrent_A = 0.25;
cfg.experiment.holdTime_s = 0.100;
cfg.experiment.pulseTime_s = 0.080;
cfg.experiment.restTime_s = 0.080;
cfg.experiment.repeatCount = 5;

cfg.sensor.positionNoiseStd_rad = 1.0e-4;
cfg.sensor.speedNoiseStd_radps = 0.08;
cfg.sensor.currentNoiseStd_A = 2.0e-3;
cfg.sensor.randomSeed = 7;

cfg.ident.edgeSkip_s = 8.0e-3;
cfg.ident.fitWindow_s = 30.0e-3;
cfg.ident.fitHop_s = 15.0e-3;
cfg.ident.minPulseSamples = 18;
cfg.ident.minAbsCurrent_A = 0.030;
cfg.ident.minAbsAcceleration_radps2 = 20.0;
cfg.ident.maxCoastCurrent_A = 0.020;
cfg.ident.minCoastSpeed_radps = 5.0;
cfg.ident.Kt_Nm_per_A = cfg.motor.Kt_Nm_per_A;
end
