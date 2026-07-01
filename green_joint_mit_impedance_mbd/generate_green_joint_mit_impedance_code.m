%% Generate embedded C code for green_joint_mit_impedance_model
%
% This script rebuilds the model and runs Embedded Coder code generation.
% It intentionally does not compile generated C with a host compiler.

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

run(fullfile(script_dir, 'build_green_joint_mit_impedance_model.m'));
cd(script_dir);

cfg = evalin('base', 'gj_mit_config');
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

fprintf('\nGreen-joint MIT impedance code generation finished.\n');
