%% Generate C code for the struct/bus I/O controller model

clear; clc;

script_dir = fileparts(mfilename('fullpath'));
model = 'single_joint_controller_struct';
model_file = fullfile(script_dir, [model '.slx']);

if ~exist(model_file, 'file')
    run(fullfile(script_dir, 'build_controller_struct_model.m'));
else
    run(fullfile(script_dir, 'build_controller_struct_model.m'));
end

load_system(model_file);

set_param(model, ...
    'SystemTargetFile', 'grt.tlc', ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'StrictBusMsg', 'ErrorLevel1', ...
    'BusObjectLabelMismatch', 'error', ...
    'GenerateReport', 'on', ...
    'RTWVerbose', 'on');

block = [model '/ControllerStruct'];
fprintf('Generating struct I/O controller C code for block: %s\n', block);
slbuild(block);

fprintf('\nStruct I/O controller C code generation finished. Check:\n');
fprintf('  ControllerStruct_grt_rtw/ControllerStruct.c\n');
fprintf('  ControllerStruct_grt_rtw/ControllerStruct.h\n');
