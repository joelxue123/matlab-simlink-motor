%% Generate embedded C code for green_joint_current_loop_model
%
% This milestone checks that the formal PI interface, generated algorithm, and
% agreed Vd-priority voltage allocation are code-generation ready.

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

run(fullfile(script_dir, 'build_green_joint_current_loop_model.m'));
cd(script_dir);

contract = evalin('base', 'green_joint_current_loop_contract');
model = contract.model;

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

addpath(script_dir);
verify_green_joint_current_pi_codegen();

fprintf('\nGreen-joint current-loop code generation finished.\n');
