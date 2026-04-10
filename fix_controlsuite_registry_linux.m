function fix_controlsuite_registry_linux(target_folder)
% Work around Linux C2000 registry lookup when MATLAB builds from example folders.

if nargin < 1 || strlength(string(target_folder)) == 0
    target_folder = pwd;
end

target_folder = char(string(target_folder));

release_name = version('-release');
arch_name = computer('arch');
registry_name = ['thirdpartytools_' arch_name '.xml'];

source_registry = fullfile(matlabroot, 'toolbox', 'c2b', 'tic2000', ...
    'registry', 'thirdpartytools', registry_name);
local_registry = fullfile(target_folder, release_name, 'toolbox', 'c2b', 'tic2000', ...
    'registry', 'thirdpartytools', registry_name);

fprintf('MATLAB release: %s\n', release_name);
fprintf('Architecture: %s\n', arch_name);
fprintf('Current folder: %s\n', pwd);
fprintf('Target folder: %s\n', target_folder);
fprintf('Expected source registry: %s\n', source_registry);

if ~isunix || ismac
    error('This helper is intended for Linux MATLAB installations only.');
end

if ~isfile(source_registry)
    error([ ...
        'MATLAB could not find the C2000 third-party registry file at:\n%s\n\n' ...
        'Run c2000setup first, or verify that the C2000 support package is installed.' ...
        ], source_registry);
end

local_folder = fileparts(local_registry);
if ~isfolder(local_folder)
    mkdir(local_folder);
end

if isfile(local_registry)
    delete(local_registry);
end

link_command = sprintf('ln -s "%s" "%s"', source_registry, local_registry);
[status, output] = system(link_command);

if status ~= 0
    copyfile(source_registry, local_registry, 'f');
    fprintf('Created a copied registry file at:\n%s\n', local_registry);
    fprintf('Symbolic link creation failed with:\n%s\n', strtrim(output));
else
    fprintf('Created a symbolic link at:\n%s\n', local_registry);
end

fprintf(['\nIf your build was launched from an example under /tmp/Examples, ', ...
    'this local registry mirror satisfies the relative ControlSUITE lookup.\n']);
fprintf(['If the build still reports that ControlSUITE is not set up, ', ...
    'run c2000setup and point it to your ControlSUITE and CCS installations.\n']);
end