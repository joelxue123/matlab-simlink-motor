%% Generate C code for the single-joint dynamics Simulink model

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

fprintf('Generating C code for model: %s\n', model);
slbuild(model);

fprintf('\nC code generation finished. Check generated folders such as:\n');
fprintf('  %s_grt_rtw\n', model);
