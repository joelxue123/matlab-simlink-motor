%% Sync green-joint speed-loop twin parameters into speed PI dictionary
%
% This script is the bridge from the physical green-joint speed-loop design
% to the reusable SpeedPiStep model. It does not rebuild .slx files; it only
% updates speed_pi_interface.sldd so Model Reference users see the same
% Kp/Ki/Kaw values as the design calculation and firmware adapter.

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
repo_dir = fileparts(script_dir);
speed_mbd_dir = fullfile(repo_dir, 'motor_speed_pi_mbd');

previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

run(fullfile(script_dir, 'design_green_joint_speed_loop.m'));

module_config = evalin('base', 'GJDT_ModuleConfig');
assignin('base', 'GJDT_SpeedKp', single(module_config.speed_loop.speed_kp));
assignin('base', 'GJDT_SpeedKi', single(module_config.speed_loop.speed_ki));
assignin('base', 'GJDT_SpeedKaw', single(module_config.speed_loop.speed_kaw));
assignin('base', 'GJDT_SpeedIqLimit_A', ...
    single(module_config.speed_loop.iq_limit_default_a));

% design_green_joint_speed_loop.m is script-style and clears the caller
% workspace, so restore local path variables before touching the dictionary.
script_dir = fileparts(mfilename('fullpath'));
repo_dir = fileparts(script_dir);
speed_mbd_dir = fullfile(repo_dir, 'motor_speed_pi_mbd');
cd(script_dir);

dictionary_file = fullfile(speed_mbd_dir, 'speed_pi_interface.sldd');
if ~exist(dictionary_file, 'file')
    run(fullfile(speed_mbd_dir, 'build_speed_pi_model.m'));
    cd(script_dir);
end

sync_speed_dictionary(dictionary_file);

fprintf('Synced green-joint speed-loop parameters:\n  %s\n', ...
    dictionary_file);
fprintf('  motor type           = %s\n', string(evalin('base', 'GJDT_MotorType')));
fprintf('  gear ratio           = %.9g\n', evalin('base', 'GJDT_MotorGearRatio'));
fprintf('  speed-loop equiv J   = %.9g kg*m^2\n', ...
    evalin('base', 'GJDT_SpeedLoopEquivalentInertia_kg_m2'));
fprintf('  Kp_speed             = %.9g A/(rad/s)\n', ...
    double(evalin('base', 'GJDT_SpeedKp')));
fprintf('  Ki_speed             = %.9g A/rad\n', ...
    double(evalin('base', 'GJDT_SpeedKi')));
fprintf('  Kaw_speed            = %.9g 1/s\n', ...
    double(evalin('base', 'GJDT_SpeedKaw')));
fprintf('  IqLimitDefault       = %.9g A\n', ...
    double(evalin('base', 'GJDT_SpeedIqLimit_A')));

function sync_speed_dictionary(dictionary_file)
dd = Simulink.data.dictionary.open(dictionary_file);
cleanup_dd = onCleanup(@() close(dd));
section = getSection(dd, 'Design Data');

upsert_parameter(section, 'Kp_speed', ...
    double(evalin('base', 'GJDT_SpeedKp')), ...
    'T_SpeedPiGain', 'Auto');
upsert_parameter(section, 'Ki_speed', ...
    double(evalin('base', 'GJDT_SpeedKi')), ...
    'T_SpeedPiGain', 'Auto');
upsert_parameter(section, 'Kaw_speed', ...
    double(evalin('base', 'GJDT_SpeedKaw')), ...
    'T_SpeedPiGain', 'Auto');
upsert_parameter(section, 'IqLimitDefault', ...
    double(evalin('base', 'GJDT_SpeedIqLimit_A')), ...
    'T_SpeedPiCurrent', 'Auto');

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
