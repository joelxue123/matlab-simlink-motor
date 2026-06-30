%% Build a reusable Simulink library for motor-control MBD modules
%
% The library is a team-facing reuse entry. The source-of-truth modules remain
% in their milestone folders and are copied into this library when rebuilt.

clear; clc;

script_dir = fileparts(mfilename('fullpath'));
repo_dir = fileparts(script_dir);
previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

library_name = 'motor_control_lib';
library_file = fullfile(script_dir, [library_name '.slx']);
dictionary_name = 'motor_control_interface.sldd';

setup_script = fullfile(script_dir, 'setup_motor_control_modules.m');
if exist(setup_script, 'file')
    run(setup_script);
end

dictionary_file = fullfile(script_dir, dictionary_name);
if ~exist(dictionary_file, 'file')
    error(['Missing shared data dictionary: %s\n' ...
        'Build it first with:\n  run(''%s'')'], ...
        dictionary_file, ...
        fullfile(script_dir, 'build_motor_control_interface_dictionary.m'));
end

ensure_source_model(repo_dir, 'motor_speed_pi_mbd', ...
    'speed_pi_model', 'build_speed_pi_model.m');
ensure_source_model(repo_dir, 'motor_current_pi_mbd', ...
    'current_pi_model', 'build_current_pi_model.m');
ensure_source_model(repo_dir, 'motor_float_open_loop_mbd', ...
    'motor_float_open_loop_model', 'build_motor_float_open_loop_model.m');
ensure_source_model(repo_dir, 'motor_clarke_park_struct', ...
    'motor_clarke_park_model', 'build_motor_clarke_park_model.m');
ensure_source_model(repo_dir, 'pwm_deadtime_compensation_mbd', ...
    'pwm_deadtime_compensation_model', 'build_pwm_deadtime_compensation_model.m');

source_models = { ...
    fullfile(repo_dir, 'motor_speed_pi_mbd'), 'speed_pi_model'; ...
    fullfile(repo_dir, 'motor_current_pi_mbd'), 'current_pi_model'; ...
    fullfile(repo_dir, 'motor_float_open_loop_mbd'), 'motor_float_open_loop_model'; ...
    fullfile(repo_dir, 'motor_clarke_park_struct'), 'motor_clarke_park_model'; ...
    fullfile(repo_dir, 'pwm_deadtime_compensation_mbd'), 'pwm_deadtime_compensation_model'};

for i = 1:size(source_models, 1)
    load_model_from_folder(source_models{i, 1}, source_models{i, 2});
end

if bdIsLoaded(library_name)
    close_system(library_name, 0);
end

if exist(library_file, 'file')
    delete(library_file);
end

new_system(library_name, 'Library');
set_param(library_name, ...
    'Lock', 'off', ...
    'DataDictionary', dictionary_name);

add_module_block('speed_pi_model', 'SpeedPiStep', library_name, ...
    'SpeedPiStep', [90 70 300 150]);
add_module_block('current_pi_model', 'CurrentPiStep', library_name, ...
    'CurrentPiStep', [90 210 300 300]);
add_module_block('motor_float_open_loop_model', 'DqToAbcDutyStep', library_name, ...
    'DqToAbcDutyStep', [90 360 300 450]);
add_module_block('motor_clarke_park_model', 'MotorClarkeParkStep', library_name, ...
    'MotorClarkeParkStep', [90 510 300 600]);
add_module_block('motor_float_open_loop_model', 'OpenLoopCommand', library_name, ...
    'OpenLoopCommand', [90 660 300 750]);
add_module_block('pwm_deadtime_compensation_model', 'DeadtimeCompensationStep', library_name, ...
    'DeadtimeCompensationStep', [90 810 300 900]);

add_library_annotation(library_name, ...
    ['Motor Control MBD Reuse Library' newline ...
     'Use linked library blocks or model references instead of copying demo subsystems.' newline ...
     'Attach the required .sldd contracts in integration models.']);

save_system(library_name, library_file);

for i = 1:size(source_models, 1)
    if bdIsLoaded(source_models{i, 2})
        close_system(source_models{i, 2}, 0);
    end
end

fprintf('Built reusable motor-control library:\n  %s\n', library_file);

function ensure_source_model(repo_dir, folder_name, model_name, build_script_name)
model_file = fullfile(repo_dir, folder_name, [model_name '.slx']);
build_script = fullfile(repo_dir, folder_name, build_script_name);

if exist(model_file, 'file')
    return;
end

if ~exist(build_script, 'file')
    error('Missing source model and build script for %s.', model_name);
end

run(build_script);
end

function load_model_from_folder(model_dir, model_name)
previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(model_dir);
load_system([model_name '.slx']);
end

function add_module_block(source_model, source_block, library_name, dest_block, position)
add_block([source_model '/' source_block], ...
    [library_name '/' dest_block], ...
    'Position', position);
end

function add_library_annotation(library_name, text)
try
    Simulink.Annotation(library_name, text, ...
        'Position', [360 70 760 170]);
catch
    % Annotation support varies across releases; the library blocks are enough.
end
end
