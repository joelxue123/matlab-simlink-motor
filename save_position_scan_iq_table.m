function result = save_position_scan_iq_table(varargin)
% Extract and save per-step iq values from a completed position-scan run.
%
% Usage:
%   sim('average_inverter_foc');
%   result = save_position_scan_iq_table;
%
%   sim_out = sim('average_inverter_foc', 'ReturnWorkspaceOutputs', 'on');
%   result = save_position_scan_iq_table(sim_out);
%
%   result = save_position_scan_iq_table(cfg);
%   result = save_position_scan_iq_table(sim_out, cfg);
%
% Optional cfg fields:
%   avg_time, settle_time, use_iq_meas, use_last_sample, file_name

[source, cfg] = local_parse_inputs(varargin{:});

control = evalin('base', 'control');
pos_ref_ts = local_get_signal(source, 'log_pos_ref');
pos_ts = local_get_signal(source, 'log_pos');
wkf_ts = local_get_signal(source, 'log_wkf');
iqref_ts = local_get_signal(source, 'log_iq_ref');
iqmeas_ts = local_get_signal(source, 'log_iq_meas');

cfg = local_fill_defaults(cfg, control);

t_ref = pos_ref_ts.Time(:);
pos_ref = pos_ref_ts.Data(:);
pos_meas = interp1(pos_ts.Time(:), pos_ts.Data(:), t_ref, 'linear', 'extrap');
w_kf = interp1(wkf_ts.Time(:), wkf_ts.Data(:), t_ref, 'linear', 'extrap');
iq_ref = interp1(iqref_ts.Time(:), iqref_ts.Data(:), t_ref, 'linear', 'extrap');
iq_meas = interp1(iqmeas_ts.Time(:), iqmeas_ts.Data(:), t_ref, 'linear', 'extrap');

scan_points = control.pos_scan.points;
theta_cmd = control.pos_scan.theta_table(1:scan_points);

theta_samples = zeros(scan_points, 1);
iq_ref_samples = zeros(scan_points, 1);
iq_meas_samples = zeros(scan_points, 1);
w_samples = zeros(scan_points, 1);
pos_err_samples = zeros(scan_points, 1);
sample_counts = zeros(scan_points, 1);

for idx = 1:scan_points
    t0 = control.pos_scan.start_time + (idx - 1) * control.pos_scan.hold_time;
    t2 = t0 + control.pos_scan.hold_time;
    t1 = max(t0 + cfg.settle_time, t2 - cfg.avg_time);
    mask = t_ref >= t1 & t_ref <= t2;
    if ~any(mask)
        continue;
    end

    sample_counts(idx) = nnz(mask);
    if cfg.use_last_sample
        sample_idx = find(mask, 1, 'last');
        theta_samples(idx) = pos_meas(sample_idx);
        iq_ref_samples(idx) = iq_ref(sample_idx);
        iq_meas_samples(idx) = iq_meas(sample_idx);
        w_samples(idx) = w_kf(sample_idx);
        pos_err_samples(idx) = pos_ref(sample_idx) - pos_meas(sample_idx);
    else
        theta_samples(idx) = mean(pos_meas(mask));
        iq_ref_samples(idx) = mean(iq_ref(mask));
        iq_meas_samples(idx) = mean(iq_meas(mask));
        w_samples(idx) = mean(w_kf(mask));
        pos_err_samples(idx) = mean(pos_ref(mask) - pos_meas(mask));
    end
end

iq_selected = iq_ref_samples;
if cfg.use_iq_meas
    iq_selected = iq_meas_samples;
end

result = struct();
result.theta_command = theta_cmd(:);
result.theta_meas = theta_samples;
result.iq_ref = iq_ref_samples;
result.iq_meas = iq_meas_samples;
result.iq_selected = iq_selected;
result.w_kf = w_samples;
result.pos_err = pos_err_samples;
result.sample_counts = sample_counts;
result.avg_time = cfg.avg_time;
result.settle_time = cfg.settle_time;
result.use_last_sample = cfg.use_last_sample;
result.use_iq_meas = cfg.use_iq_meas;
result.disturbance = struct(...
    'load_base_torque', control.vib.load_base_torque, ...
    'load_amp1', control.vib.load_amp1, ...
    'load_harmonic1', control.vib.load_harmonic1, ...
    'load_phase1_deg', control.vib.load_phase1_deg, ...
    'load_amp2', control.vib.load_amp2, ...
    'load_harmonic2', control.vib.load_harmonic2, ...
    'load_phase2_deg', control.vib.load_phase2_deg);

save(cfg.file_name, 'result');
assignin('base', 'position_scan_iq_result', result);

fprintf('\nSaved position scan iq result: %s\n', cfg.file_name);
fprintf('Points          : %d\n', scan_points);
fprintf('Extraction mode : %s\n', local_mode_string(cfg));
fprintf('Signal used     : %s\n', ternary(cfg.use_iq_meas, 'iq_meas', 'iq_ref'));
end

function [source, cfg] = local_parse_inputs(varargin)
source = [];
cfg = struct();

if nargin == 0
    return;
end

first = varargin{1};
if local_is_sim_output_candidate(first)
    source = first;
    if nargin >= 2
        cfg = varargin{2};
    end
else
    cfg = first;
end
end

function tf = local_is_sim_output_candidate(value)
tf = isa(value, 'Simulink.SimulationOutput') || ...
    (isobject(value) && any(strcmp(methods(value), 'get')));
end

function signal = local_get_signal(source, name)
if ~isempty(source)
    try
        signal = source.get(name);
        return;
    catch
    end
end

if evalin('base', sprintf("exist('%s','var')", name))
    signal = evalin('base', name);
    return;
end

sim_output_names = local_find_sim_output_names();
for idx = 1:numel(sim_output_names)
    candidate = evalin('base', sim_output_names{idx});
    try
        signal = candidate.get(name);
        fprintf('Using signal %s from base workspace simulation output %s.\n', name, sim_output_names{idx});
        return;
    catch
    end
end

error(['Signal %s was not found. Run sim(''average_inverter_foc'') so the log_* ' ...
    'variables exist in base workspace, or call save_position_scan_iq_table(sim_out) ' ...
    'after sim(..., ''ReturnWorkspaceOutputs'', ''on'').'], name);
end

function names = local_find_sim_output_names()
base_vars = evalin('base', 'whos');
names = {};
for idx = 1:numel(base_vars)
    if strcmp(base_vars(idx).class, 'Simulink.SimulationOutput')
        names{end + 1} = base_vars(idx).name; %#ok<AGROW>
    end
end
end

function cfg = local_fill_defaults(cfg, control)
defaults = struct();
defaults.avg_time = min(0.015, 0.75 * control.pos_scan.hold_time);
defaults.settle_time = max(0, control.pos_scan.hold_time - defaults.avg_time);
defaults.use_iq_meas = false;
defaults.use_last_sample = false;
defaults.file_name = 'position_scan_iq_result.mat';

names = fieldnames(defaults);
for idx = 1:numel(names)
    name = names{idx};
    if ~isfield(cfg, name) || isempty(cfg.(name))
        cfg.(name) = defaults.(name);
    end
end

cfg.avg_time = min(max(cfg.avg_time, 0.001), 0.9 * control.pos_scan.hold_time);
cfg.settle_time = min(max(cfg.settle_time, 0), control.pos_scan.hold_time - cfg.avg_time);
end

function text = local_mode_string(cfg)
if cfg.use_last_sample
    text = 'last sample';
else
    text = 'tail-window mean';
end
end

function out = ternary(cond, a, b)
if cond
    out = a;
else
    out = b;
end
end