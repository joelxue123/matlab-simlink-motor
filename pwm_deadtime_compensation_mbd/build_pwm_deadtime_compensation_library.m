%% Build reusable Simulink library for PWM dead-time compensation
%
% This library is the block-insertion form of the MBD core. Integration
% models should link to this block instead of rebuilding the algorithm.

clear; clc;

script_dir = fileparts(mfilename('fullpath'));
previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

run(fullfile(script_dir, 'build_pwm_deadtime_compensation_model.m'));

src_model = 'pwm_deadtime_compensation_model';
lib_model = 'pwm_deadtime_compensation_lib';
lib_file = fullfile(script_dir, [lib_model '.slx']);

if bdIsLoaded(lib_model)
    close_system(lib_model, 0);
end
if exist(lib_file, 'file')
    delete(lib_file);
end

load_system(src_model);
new_system(lib_model, 'Library');
set_param(lib_model, 'EnableLBRepository', 'on');

add_block([src_model '/DeadtimeCompensationStep'], ...
    [lib_model '/DeadtimeCompensationStep'], ...
    'Position', [120 90 360 180]);

note = Simulink.Annotation(lib_model, sprintf([ ...
    'Reusable MBD block\n' ...
    'Source: pwm_deadtime_compensation_model/DeadtimeCompensationStep\n' ...
    'Interface: pwm_deadtime_comp_input_t -> pwm_deadtime_comp_output_t\n' ...
    'Do not copy algorithm logic into integration harnesses.']));
note.Position = [70 225 440 310];

set_param(lib_model, 'Lock', 'on');
save_system(lib_model, lib_file);

fprintf('Built reusable PWM dead-time compensation library:\n  %s\n', lib_file);
