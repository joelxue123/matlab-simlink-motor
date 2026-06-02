%% Generate C code for only the controller model

clear; clc;

script_dir = fileparts(mfilename('fullpath'));
root_dir = fileparts(script_dir);
matlab_dir = fullfile(root_dir, 'matlab');
addpath(matlab_dir);

cfg = config_default();
init_controller_fixed_point_types(cfg);

model = 'single_joint_controller_only';
model_file = fullfile(script_dir, [model '.slx']);

run(fullfile(script_dir, 'build_controller_only_model.m'));

load_system(model_file);

set_param(model, ...
    'SystemTargetFile', 'grt.tlc', ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'GenerateReport', 'on', ...
    'RTWVerbose', 'on');

fprintf('Generating controller-only C code for model: %s\n', model);
slbuild(model);

fprintf('\nController-only C code generation finished. Check:\n');
fprintf('  %s_grt_rtw/%s.c\n', model, model);
