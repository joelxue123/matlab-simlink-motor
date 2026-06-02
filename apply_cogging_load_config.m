function control = apply_cogging_load_config(control, cfg)
% Apply centralized cogging/load torque configuration to control.vib.

cfg = cogging_load_config(cfg);

control.vib.load_base_torque = cfg.load_base_torque;
control.vib.load_amp1 = cfg.amp1;
control.vib.load_harmonic1 = cfg.harmonic1;
control.vib.load_phase1_deg = cfg.phase1_deg;
control.vib.load_amp2 = cfg.amp2;
control.vib.load_harmonic2 = cfg.harmonic2;
control.vib.load_phase2_deg = cfg.phase2_deg;
end
