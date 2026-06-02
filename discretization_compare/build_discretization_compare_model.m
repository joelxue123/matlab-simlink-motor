function mdl_path = build_discretization_compare_model(varargin)
%BUILD_DISCRETIZATION_COMPARE_MODEL Build a Simulink model comparing discretizations.
%   mdl_path = build_discretization_compare_model()
%   mdl_path = build_discretization_compare_model('a', 100, 'b', 1, ...
%       'Ts', 1e-3, 'StopTime', 0.12, 'StepTime', 0.01)
%
% Continuous plant:
%   x_dot = -a * x + b * u
%   y     = x
%
% Discrete branches:
%   1. Forward Euler
%   2. Backward Euler
%   3. Exact ZOH discretization

cfg = localParseInputs(varargin{:});

mdl = 'discretization_compare_model';
script_dir = fileparts(mfilename('fullpath'));
mdl_path = fullfile(script_dir, [mdl '.slx']);

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
if exist(mdl_path, 'file')
    delete(mdl_path);
end

localAssignWorkspace(cfg);

new_system(mdl);
open_system(mdl);

set_param(mdl, ...
    'Solver', 'ode4', ...
    'FixedStep', 'disc_cfg.Ts / 20', ...
    'StopTime', 'disc_cfg.StopTime', ...
    'SaveFormat', 'StructureWithTime');

% Input source
add_block('simulink/Sources/Step', [mdl '/Step'], ...
    'Position', [40 110 90 140], ...
    'Time', 'disc_cfg.StepTime', ...
    'Before', 'disc_cfg.StepBefore', ...
    'After', 'disc_cfg.StepAfter', ...
    'SampleTime', '0');

% Continuous reference branch
add_block('simulink/Continuous/State-Space', [mdl '/Continuous Plant'], ...
    'Position', [160 70 320 130], ...
    'A', 'disc_cfg.A_cont', ...
    'B', 'disc_cfg.B_cont', ...
    'C', '1', ...
    'D', '0');
add_line(mdl, 'Step/1', 'Continuous Plant/1');

% Shared sampled input for discrete branches
add_block('simulink/Discrete/Zero-Order Hold', [mdl '/Input ZOH'], ...
    'Position', [150 180 220 210], ...
    'SampleTime', 'disc_cfg.Ts');
add_line(mdl, 'Step/1', 'Input ZOH/1');

% Forward Euler
add_block('simulink/Discrete/Discrete State-Space', [mdl '/Forward Euler'], ...
    'Position', [290 160 470 220], ...
    'A', 'disc_cfg.A_fe', ...
    'B', 'disc_cfg.B_fe', ...
    'C', '1', ...
    'D', '0', ...
    'SampleTime', 'disc_cfg.Ts');
add_line(mdl, 'Input ZOH/1', 'Forward Euler/1');

% Backward Euler
add_block('simulink/Discrete/Discrete State-Space', [mdl '/Backward Euler'], ...
    'Position', [290 250 470 310], ...
    'A', 'disc_cfg.A_be', ...
    'B', 'disc_cfg.B_be', ...
    'C', '1', ...
    'D', '0', ...
    'SampleTime', 'disc_cfg.Ts');
add_line(mdl, 'Input ZOH/1', 'Backward Euler/1', 'autorouting', 'on');

% Exact ZOH discretization
add_block('simulink/Discrete/Discrete State-Space', [mdl '/Exact ZOH'], ...
    'Position', [290 340 470 400], ...
    'A', 'disc_cfg.A_zoh', ...
    'B', 'disc_cfg.B_zoh', ...
    'C', '1', ...
    'D', '0', ...
    'SampleTime', 'disc_cfg.Ts');
add_line(mdl, 'Input ZOH/1', 'Exact ZOH/1', 'autorouting', 'on');

% Workspace logging blocks
add_block('simulink/Sinks/To Workspace', [mdl '/y_cont'], ...
    'Position', [390 85 480 115], ...
    'VariableName', 'y_cont', ...
    'SaveFormat', 'StructureWithTime');
add_block('simulink/Sinks/To Workspace', [mdl '/y_fe'], ...
    'Position', [520 175 610 205], ...
    'VariableName', 'y_fe', ...
    'SaveFormat', 'StructureWithTime');
add_block('simulink/Sinks/To Workspace', [mdl '/y_be'], ...
    'Position', [520 265 610 295], ...
    'VariableName', 'y_be', ...
    'SaveFormat', 'StructureWithTime');
add_block('simulink/Sinks/To Workspace', [mdl '/y_zoh'], ...
    'Position', [520 355 610 385], ...
    'VariableName', 'y_zoh', ...
    'SaveFormat', 'StructureWithTime');

add_line(mdl, 'Continuous Plant/1', 'y_cont/1');
add_line(mdl, 'Forward Euler/1', 'y_fe/1');
add_line(mdl, 'Backward Euler/1', 'y_be/1');
add_line(mdl, 'Exact ZOH/1', 'y_zoh/1');

% Scope for visual comparison
add_block('simulink/Signal Routing/Mux', [mdl '/Mux'], ...
    'Position', [665 120 670 350], ...
    'Inputs', '4');
add_block('simulink/Sinks/Scope', [mdl '/Comparison Scope'], ...
    'Position', [735 130 935 320], ...
    'NumInputPorts', '1');

add_line(mdl, 'Continuous Plant/1', 'Mux/1');
add_line(mdl, 'Forward Euler/1', 'Mux/2');
add_line(mdl, 'Backward Euler/1', 'Mux/3');
add_line(mdl, 'Exact ZOH/1', 'Mux/4');
add_line(mdl, 'Mux/1', 'Comparison Scope/1');

save_system(mdl, mdl_path);
fprintf('Discretization comparison model saved: %s\n', mdl_path);
fprintf('a = %.6g, b = %.6g, Ts = %.6g s\n', cfg.a, cfg.b, cfg.Ts);
fprintf('Forward Euler : A = %.9g, B = %.9g\n', cfg.A_fe, cfg.B_fe);
fprintf('Backward Euler: A = %.9g, B = %.9g\n', cfg.A_be, cfg.B_be);
fprintf('Exact ZOH     : A = %.9g, B = %.9g\n', cfg.A_zoh, cfg.B_zoh);
end

function cfg = localParseInputs(varargin)
cfg = struct();
cfg.a = 100;
cfg.b = 1;
cfg.Ts = 1e-3;
cfg.StopTime = 0.12;
cfg.StepTime = 0.01;
cfg.StepBefore = 0;
cfg.StepAfter = 1;

if mod(numel(varargin), 2) ~= 0
    error('Name-value arguments must be pairs.');
end

for idx = 1:2:numel(varargin)
    name = varargin{idx};
    value = varargin{idx + 1};
    if ~isfield(cfg, name)
        error('Unknown option: %s', name);
    end
    cfg.(name) = value;
end

if ~isscalar(cfg.a) || cfg.a < 0
    error('a must be a nonnegative scalar.');
end
if ~isscalar(cfg.b)
    error('b must be a scalar.');
end
if ~isscalar(cfg.Ts) || cfg.Ts <= 0
    error('Ts must be a positive scalar.');
end

cfg.A_cont = -cfg.a;
cfg.B_cont = cfg.b;

cfg.A_fe = 1 - cfg.a * cfg.Ts;
cfg.B_fe = cfg.b * cfg.Ts;

cfg.A_be = 1 / (1 + cfg.a * cfg.Ts);
cfg.B_be = cfg.b * cfg.Ts / (1 + cfg.a * cfg.Ts);

cfg.A_zoh = exp(-cfg.a * cfg.Ts);
if cfg.a == 0
    cfg.B_zoh = cfg.b * cfg.Ts;
else
    cfg.B_zoh = cfg.b * (1 - exp(-cfg.a * cfg.Ts)) / cfg.a;
end
end

function localAssignWorkspace(cfg)
disc_cfg = cfg; %#ok<NASGU>
assignin('base', 'disc_cfg', disc_cfg);
end