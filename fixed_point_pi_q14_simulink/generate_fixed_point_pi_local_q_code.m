%% Generate C code for the reusable local-Q fixed-point PI module

clear; clc;

script_dir = fileparts(mfilename('fullpath'));
run(fullfile(script_dir, 'build_fixed_point_pi_local_q_model.m'));

model = 'fixed_point_pi_local_q';
load_system(model);
slbuild(model);

source_file = fullfile(script_dir, [model '_ert_rtw'], [model '.c']);
header_file = fullfile(script_dir, [model '_ert_rtw'], [model '.h']);

fprintf('\nGenerated local-Q PI code:\n');
fprintf('  Source: %s\n', source_file);
fprintf('  Header: %s\n', header_file);

if exist(source_file, 'file')
    code_text = fileread(source_file);
    if contains(code_text, 'FixedPointPI_LocalQ')
        fprintf('  Reusable subsystem function found.\n');
    else
        fprintf('  Warning: reusable subsystem function name was not found.\n');
    end
end
