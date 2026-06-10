%% Generate embedded C code for the motor Clarke/Park struct example

clear; clc;

script_dir = fileparts(mfilename('fullpath'));
run(fullfile(script_dir, 'build_motor_clarke_park_model.m'));

cfg = evalin('base', 'motor_clarke_park_codegen_config');
model = cfg.modelName;
model_file = fullfile(script_dir, [model '.slx']);
code_dir = fullfile(script_dir, [model '_ert_rtw']);

load_system(model_file);

fprintf('Generating ERT C code for model: %s\n', model);
slbuild(model);

fprintf('\nGenerated code directory:\n  %s\n', code_dir);

c_files = dir(fullfile(code_dir, '*.c'));
h_files = dir(fullfile(code_dir, '*.h'));

fprintf('\nGenerated C files:\n');
for i = 1:numel(c_files)
    fprintf('  %s\n', fullfile(c_files(i).folder, c_files(i).name));
end

fprintf('\nGenerated header files:\n');
for i = 1:numel(h_files)
    fprintf('  %s\n', fullfile(h_files(i).folder, h_files(i).name));
end
