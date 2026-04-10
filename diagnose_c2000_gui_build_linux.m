function report = diagnose_c2000_gui_build_linux(example_folder)
% Diagnose common Linux R2023b GUI build failures for C2000 examples.

if nargin < 1 || strlength(string(example_folder)) == 0
    example_folder = pwd;
end

example_folder = char(string(example_folder));
release_name = version('-release');
arch_name = computer('arch');
registry_name = ['thirdpartytools_' arch_name '.xml'];

source_registry = fullfile(matlabroot, 'toolbox', 'c2b', 'tic2000', ...
    'registry', 'thirdpartytools', registry_name);
local_registry = fullfile(example_folder, release_name, 'toolbox', 'c2b', ...
    'tic2000', 'registry', 'thirdpartytools', registry_name);

report = struct();
report.current_folder = pwd;
report.example_folder = example_folder;
report.release_name = release_name;
report.arch_name = arch_name;
report.source_registry = source_registry;
report.local_registry = local_registry;
report.example_folder_exists = isfolder(example_folder);
report.source_registry_exists = isfile(source_registry);
report.local_registry_exists = isfile(local_registry);
report.current_folder_matches_example = strcmp(report.current_folder, example_folder);
report.controlsuite_hint = 'Unknown';

fprintf('=== C2000 GUI Build Diagnostic ===\n');
fprintf('MATLAB release: %s\n', report.release_name);
fprintf('Architecture: %s\n', report.arch_name);
fprintf('Current folder: %s\n', report.current_folder);
fprintf('Example folder: %s\n', report.example_folder);
fprintf('Source registry: %s\n', report.source_registry);
fprintf('Local registry: %s\n\n', report.local_registry);

print_check('Example folder exists', report.example_folder_exists);
print_check('Source registry exists under matlabroot', report.source_registry_exists);
print_check('Local relative registry exists in example folder', report.local_registry_exists);
print_check('Current MATLAB folder matches example folder', report.current_folder_matches_example);

if ~report.example_folder_exists
    report.controlsuite_hint = 'Example folder path is wrong.';
elseif ~report.source_registry_exists
    report.controlsuite_hint = ['C2000 registry source file is missing. ', ...
        'Run c2000setup or verify the C2000 support package installation.'];
elseif ~report.local_registry_exists
    report.controlsuite_hint = ['Relative registry mirror is missing in the example folder. ', ...
        'Run prepare_c2000_gui_build_linux(example_folder).'];
elseif ~report.current_folder_matches_example
    report.controlsuite_hint = ['MATLAB current folder is not the example folder. ', ...
        'Run cd(example_folder) before GUI build, or use prepare_c2000_gui_build_linux(example_folder).'];
else
    report.controlsuite_hint = ['Registry path checks passed. If Build still fails, ', ...
        'the remaining issue is likely incomplete c2000setup configuration for CCS or ControlSUITE.'];
end

fprintf('\nDiagnosis: %s\n', report.controlsuite_hint);

fprintf('\nSuggested next command:\n');
if ~report.local_registry_exists || ~report.current_folder_matches_example
    fprintf('prepare_c2000_gui_build_linux(''%s'')\n', report.example_folder);
else
    fprintf('c2000setup\n');
end
end

function print_check(label, condition)
if condition
    status = 'OK';
else
    status = 'MISSING';
end

fprintf('[%s] %s\n', status, label);
end