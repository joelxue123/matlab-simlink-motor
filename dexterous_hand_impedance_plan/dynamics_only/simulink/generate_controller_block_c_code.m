%% Generate C code for only the Controller block inside the full model
% This creates Controller_grt_rtw/Controller.c.

clear; clc;

script_dir = fileparts(mfilename('fullpath'));
root_dir = fileparts(script_dir);
matlab_dir = fullfile(root_dir, 'matlab');
addpath(matlab_dir);

cfg = config_default();
init_controller_fixed_point_types(cfg);

model = 'single_joint_dynamics_control';
model_file = fullfile(script_dir, [model '.slx']);

run(fullfile(script_dir, 'build_single_joint_dynamics_model.m'));

load_system(model_file);

set_param(model, ...
    'SystemTargetFile', 'grt.tlc', ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'GenerateReport', 'on', ...
    'RTWVerbose', 'on');

fprintf('Generating C code for block: %s/Controller\n', model);
slbuild([model '/Controller']);

fprintf('\nController block C code generation finished. Check:\n');
fprintf('  Controller_grt_rtw/usr_pid.c\n');
fprintf('  Controller_grt_rtw/usr_pid.h\n');
