%% Generate embedded C code for motor_current_loop_model
%
% This script rebuilds the current-loop integration model and runs Embedded
% Coder code generation. It intentionally does not compile generated C with a
% host compiler.

clear; clc;

script_dir = fileparts(mfilename('fullpath'));
run(fullfile(script_dir, 'build_motor_current_loop_model.m'));

cfg = evalin('base', 'motor_current_loop_config');
model = cfg.modelName;

fprintf('\nGenerating C code for %s...\n', model);
slbuild(model);

code_dir = fullfile(script_dir, [model '_ert_rtw']);
source_files = [ ...
    dir(fullfile(code_dir, '*.c')); ...
    dir(fullfile(code_dir, '*.h'))];

fprintf('\nGenerated source files:\n');
for i = 1:numel(source_files)
    fprintf('  %s\n', fullfile(source_files(i).folder, source_files(i).name));
end

fprintf('\nCurrent-loop code generation finished.\n');
