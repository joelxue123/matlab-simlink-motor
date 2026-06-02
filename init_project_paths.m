function project_root = init_project_paths(anchor_path)
% Ensure the average-inverter project root and common library folders are on path.

if nargin < 1 || isempty(anchor_path)
    anchor_path = fileparts(mfilename('fullpath'));
end

project_root = local_find_project_root(anchor_path);

path_list = {
    project_root
    fullfile(project_root, 'algorithms')
    fullfile(project_root, 'build_modules')
    fullfile(project_root, 'studies')
    };

for idx = 1:numel(path_list)
    folder = path_list{idx};
    if exist(folder, 'dir') ~= 7
        continue;
    end
    if ~local_on_path(folder)
        addpath(folder);
    end
end
end

function project_root = local_find_project_root(anchor_path)
if exist(anchor_path, 'dir') ~= 7
    anchor_path = fileparts(anchor_path);
end

current = anchor_path;
while true
    has_config = exist(fullfile(current, 'motor_control_params.m'), 'file') == 2;
    has_algorithms = exist(fullfile(current, 'algorithms'), 'dir') == 7;
    has_build_modules = exist(fullfile(current, 'build_modules'), 'dir') == 7;
    if has_config && has_algorithms && has_build_modules
        project_root = current;
        return;
    end

    parent = fileparts(current);
    if strcmp(parent, current)
        error('Could not locate average-inverter project root from %s.', anchor_path);
    end
    current = parent;
end
end

function tf = local_on_path(folder)
current_path = regexp(path, pathsep, 'split');
tf = any(strcmp(current_path, folder));
end