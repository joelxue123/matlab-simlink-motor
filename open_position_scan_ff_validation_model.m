function open_position_scan_ff_validation_model(tableSource, cfg)
% Build and open the offline-FF validation model using a saved scan table.
%
% Usage:
%   open_position_scan_ff_validation_model
%   open_position_scan_ff_validation_model('validated_scan_ff_table.mat')
%   open_position_scan_ff_validation_model(ff_table)
%   open_position_scan_ff_validation_model(..., struct('ff_enable_time', 0.05))

if nargin < 1
    tableSource = [];
end
if nargin < 2
    cfg = struct();
end

ff_table = local_resolve_table(tableSource);
scan_meta = local_resolve_scan_metadata(tableSource);

motor_control_params;
cfg = local_fill_defaults(cfg, control, ff_table, scan_meta);

control.vib.mode = 'offline';
control.vib.enable_learning = 0;
control.vib.enable_ff = 1;
control.vib.table_points = cfg.table_points;
control.vib.ff_table = zeros(360, 1);
control.vib.ff_table(1:numel(ff_table)) = ff_table(:);
control.vib.ff_table_file = cfg.ff_table_file;
control.vib.output_limit = cfg.output_limit;
control.vib.phase_advance_deg = cfg.phase_advance_deg;
control.vib.ff_enable_time = cfg.ff_enable_time;
control.vib.learn_start_time = cfg.learn_start_time;
control.vib.test_stop_time = cfg.test_stop_time;
control = apply_cogging_load_config(control, cfg);

assignin('base', 'motor', motor);
assignin('base', 'inverter', inverter);
assignin('base', 'control', control);
assignin('base', 'simcfg', simcfg);

build_vibration_comp_test;
set_param('vibration_comp_test', 'InitFcn', '');

open_system('vibration_comp_test');
open_system('vibration_comp_test/Vibration Scope');

fprintf('\nOffline FF validation model is ready.\n');
fprintf('Model             : vibration_comp_test\n');
fprintf('Mode              : %s\n', control.vib.mode);
fprintf('enable_ff         : %d\n', control.vib.enable_ff);
fprintf('table_points      : %d\n', control.vib.table_points);
fprintf('ff_enable_time    : %.4f s\n', control.vib.ff_enable_time);
fprintf('test_stop_time    : %.4f s\n', control.vib.test_stop_time);
fprintf('nonzero ff points : %d\n', nnz(abs(control.vib.ff_table) > 0));
fprintf('\nRun in MATLAB command window:\n');
fprintf('  sim_out = sim(''vibration_comp_test'', ''ReturnWorkspaceOutputs'', ''on'');\n');
fprintf('  max(abs(sim_out.get(''log_vib_iqff'').Data))\n');
end

function ff_table = local_resolve_table(tableSource)
if isempty(tableSource)
    if evalin('base', "exist('validated_scan_ff_table','var')")
        ff_table = evalin('base', 'validated_scan_ff_table');
        return;
    end
    if evalin('base', "exist('ff_table','var')")
        ff_table = evalin('base', 'ff_table');
        return;
    end
    if exist('validated_scan_ff_table.mat', 'file')
        s = load('validated_scan_ff_table.mat');
        ff_table = s.ff_table;
        return;
    end
    error(['No FF table found. Provide a table, or create one first with ' ...
        'validate_position_scan_ff_table.']);
end

if isnumeric(tableSource)
    ff_table = tableSource(:);
    return;
end

if ischar(tableSource) || isstring(tableSource)
    s = load(char(tableSource));
    if isfield(s, 'ff_table')
        ff_table = s.ff_table(:);
        return;
    end
    error('File %s does not contain ff_table.', char(tableSource));
end

error('Unsupported tableSource input.');
end

function meta = local_resolve_scan_metadata(tableSource)
meta = struct();
if evalin('base', "exist('validated_scan_result','var')")
    candidate = evalin('base', 'validated_scan_result');
    if isstruct(candidate) && isfield(candidate, 'config')
        meta = candidate.config;
        return;
    end
end
if evalin('base', "exist('position_scan_iq_result','var')")
    candidate = evalin('base', 'position_scan_iq_result');
    if isstruct(candidate) && isfield(candidate, 'disturbance')
        meta = candidate.disturbance;
        return;
    end
end
if ischar(tableSource) || isstring(tableSource)
    loaded = load(char(tableSource));
    if isfield(loaded, 'cfg') && isstruct(loaded.cfg)
        meta = loaded.cfg;
    elseif isfield(loaded, 'result') && isstruct(loaded.result) && isfield(loaded.result, 'config')
        meta = loaded.result.config;
    elseif isfield(loaded, 'scan_result') && isstruct(loaded.scan_result) && isfield(loaded.scan_result, 'disturbance')
        meta = loaded.scan_result.disturbance;
    end
end
end

function cfg = local_fill_defaults(cfg, control, ff_table, scan_meta)
defaults = struct();
defaults.table_points = min(360, nnz(abs(ff_table) > 0));
defaults.output_limit = 0.25 * control.iq_ref_limit;
defaults.phase_advance_deg = 0;
defaults.ff_enable_time = 0.55;
defaults.learn_start_time = 0.15;
defaults.test_stop_time = 1.0;
defaults.ff_table_file = 'validated_scan_ff_table.mat';
load_defaults = cogging_load_config(scan_meta);
defaults.load_base_torque = load_defaults.load_base_torque;
defaults.amp1 = load_defaults.amp1;
defaults.harmonic1 = load_defaults.harmonic1;
defaults.phase1_deg = load_defaults.phase1_deg;
defaults.amp2 = load_defaults.amp2;
defaults.harmonic2 = load_defaults.harmonic2;
defaults.phase2_deg = load_defaults.phase2_deg;

names = fieldnames(defaults);
for idx = 1:numel(names)
    name = names{idx};
    if ~isfield(cfg, name) || isempty(cfg.(name))
        cfg.(name) = defaults.(name);
    end
end

cfg.table_points = max(8, min(360, round(cfg.table_points)));
end

function value = local_meta_field(meta, names, fallback)
value = fallback;
for idx = 1:numel(names)
    name = names{idx};
    if isstruct(meta) && isfield(meta, name)
        value = meta.(name);
        return;
    end
end
end