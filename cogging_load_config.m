function cfg = cogging_load_config(overrides)
% Central cogging/load torque configuration used by scan, validation, and demos.
%
% The default is one sinusoidal load-torque period per mechanical revolution:
%   T_load = load_base_torque + amp1 * sin(theta_mech + phase1_deg)
% The second harmonic is disabled by default.

if nargin < 1 || isempty(overrides)
    overrides = struct();
end

cfg = struct();
cfg.load_base_torque = 0;
cfg.amp1 = 0.06;
cfg.harmonic1 = 1;
cfg.phase1_deg = 0;
cfg.amp2 = 0;
cfg.harmonic2 = 0;
cfg.phase2_deg = 0;

cfg.load_base_torque = local_get(overrides, {'load_base_torque'}, cfg.load_base_torque);
cfg.amp1 = local_get(overrides, {'amp1', 'load_amp1'}, cfg.amp1);
cfg.harmonic1 = local_get(overrides, {'harmonic1', 'load_harmonic1'}, cfg.harmonic1);
cfg.phase1_deg = local_get(overrides, {'phase1_deg', 'load_phase1_deg'}, cfg.phase1_deg);
cfg.amp2 = local_get(overrides, {'amp2', 'load_amp2'}, cfg.amp2);
cfg.harmonic2 = local_get(overrides, {'harmonic2', 'load_harmonic2'}, cfg.harmonic2);
cfg.phase2_deg = local_get(overrides, {'phase2_deg', 'load_phase2_deg'}, cfg.phase2_deg);

if cfg.amp2 == 0
    cfg.harmonic2 = 0;
    cfg.phase2_deg = 0;
end
end

function value = local_get(source, names, fallback)
value = fallback;
for idx = 1:numel(names)
    name = names{idx};
    if isstruct(source) && isfield(source, name) && ~isempty(source.(name))
        value = source.(name);
        return;
    end
end
end
