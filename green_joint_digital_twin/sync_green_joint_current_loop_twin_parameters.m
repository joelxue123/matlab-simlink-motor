%% Sync green-joint current-loop twin parameters into data dictionaries
%
% This updates parameter values without rebuilding .slx files. It is useful
% when MATLAB Desktop has the model open and rebuild protection is active.

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
repo_dir = fileparts(script_dir);
green_joint_mbd_dir = fullfile(repo_dir, 'green_joint_current_loop_mbd');

previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

run(fullfile(script_dir, 'setup_green_joint_current_loop_twin.m'));

dictionary_files = { ...
    fullfile(green_joint_mbd_dir, 'green_joint_current_loop_interface.sldd'), ...
    fullfile(script_dir, 'green_joint_average_motor_twin_interface.sldd')};

for i = 1:numel(dictionary_files)
    dictionary_file = dictionary_files{i};
    if exist(dictionary_file, 'file')
        sync_dictionary(dictionary_file);
        fprintf('Synced green-joint current-loop parameters:\n  %s\n', ...
            dictionary_file);
    end
end

function sync_dictionary(dictionary_file)
dd = Simulink.data.dictionary.open(dictionary_file);
cleanup_dd = onCleanup(@() close(dd));
section = getSection(dd, 'Design Data');

upsert_parameter(section, 'CurDKp', double(evalin('base', 'GJDT_CurDKp')), ...
    'T_GJFloat', 'ExportedGlobal');
upsert_parameter(section, 'CurDKi', double(evalin('base', 'GJDT_CurDKi')), ...
    'T_GJFloat', 'ExportedGlobal');
upsert_parameter(section, 'CurQKp', double(evalin('base', 'GJDT_CurQKp')), ...
    'T_GJFloat', 'ExportedGlobal');
upsert_parameter(section, 'CurQKi', double(evalin('base', 'GJDT_CurQKi')), ...
    'T_GJFloat', 'ExportedGlobal');

saveChanges(dd);
end

function upsert_parameter(section, name, value, data_type, storage_class)
parameter = Simulink.Parameter(value);
parameter.DataType = data_type;
parameter.CoderInfo.StorageClass = storage_class;

entry = find(section, 'Name', name);
if isempty(entry)
    addEntry(section, name, parameter);
else
    setValue(entry(1), parameter);
end
end
