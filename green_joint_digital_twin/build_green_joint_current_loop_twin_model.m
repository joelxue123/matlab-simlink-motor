%% Build green-joint current-loop digital twin v0
%
% This model references the green_joint_current_loop_model MBD core and
% closes it with a discrete dq average-voltage plant:
%
%   did/dt = (vd - Rs * id) / Ld
%   diq/dt = (vq - Rs * iq) / Lq
%
% Scope v0:
%   Model Reference current PI + Vd-priority limit + dq average plant
%
% Out of scope v0:
%   Clarke/Park, inverse Park, SVPWM, PWM registers, switching bridge,
%   dead-time, ADC timing, speed loop.

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
repo_dir = fileparts(script_dir);
green_joint_mbd_dir = fullfile(repo_dir, 'green_joint_current_loop_mbd');

previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);
run(fullfile(script_dir, 'setup_green_joint_current_loop_twin.m'));

model = 'green_joint_current_loop_twin_model';
model_file = fullfile(script_dir, [model '.slx']);

if should_open_existing_model_from_desktop(model_file)
    load_system(model_file);
    open_system(model);
    fprintf(['MATLAB Desktop detected. Opened existing digital twin model ' ...
        'without rebuilding:\n  %s\n'], model_file);
    fprintf(['To rebuild this model safely, close it in Desktop and run:\n' ...
        '  matlab -batch "run(''matlab-practice/green_joint_digital_twin/' ...
        'build_green_joint_current_loop_twin_model.m'')"\n']);
    return;
end

assert_safe_rebuild_environment(script_dir);

source_model = 'green_joint_current_loop_model';
source_model_file = fullfile(green_joint_mbd_dir, [source_model '.slx']);
source_dictionary = fullfile(green_joint_mbd_dir, ...
    'green_joint_current_loop_interface.sldd');

if ~exist(source_model_file, 'file')
    error(['Missing source model: %s\nRun green_joint_current_loop_mbd/' ...
        'build_green_joint_current_loop_model.m first.'], source_model_file);
end

if ~exist(source_dictionary, 'file')
    error(['Missing source dictionary: %s\nRun green_joint_current_loop_mbd/' ...
        'generate_green_joint_current_loop_dictionary.m first.'], ...
        source_dictionary);
end

if bdIsLoaded(model)
    close_system(model, 0);
end

if exist(model_file, 'file')
    delete(model_file);
end

load_system(source_model_file);
cleanup_source = onCleanup(@() close_loaded_model(source_model));

new_system(model);
set_param(model, ...
    'DataDictionary', 'green_joint_current_loop_interface.sldd', ...
    'StopTime', 'GJDT_StopTime', ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', 'GJDT_TsPlant', ...
    'ParameterPrecisionLossMsg', 'none');

add_sources(model);
add_controller(model, source_model);
add_dq_average_plant(model);
add_logs(model);
connect_model(model);

save_system(model, model_file);

fprintf('Built green-joint current-loop digital twin model:\n  %s\n', ...
    model_file);

function add_sources(model)
add_block('simulink/Sources/Constant', [model '/id_ref'], ...
    'Position', [45 65 105 95], ...
    'Value', 'GJDT_IdRef_A', ...
    'SampleTime', 'GJDT_Ts', ...
    'OutDataTypeStr', 'T_GJCurrent');

add_block('simulink/Sources/Step', [model '/iq_ref_step'], ...
    'Position', [45 120 105 150], ...
    'Time', 'GJDT_IqStepTime_s', ...
    'Before', 'GJDT_IqBefore_A', ...
    'After', 'GJDT_IqAfter_A', ...
    'SampleTime', 'GJDT_Ts');
add_block('simulink/Signal Attributes/Data Type Conversion', ...
    [model '/iq_ref_to_current'], ...
    'Position', [130 120 205 150], ...
    'OutDataTypeStr', 'T_GJCurrent');

add_block('simulink/Sources/Constant', [model '/vbus'], ...
    'Position', [45 285 105 315], ...
    'Value', 'GJDT_Vbus_V', ...
    'SampleTime', 'GJDT_Ts', ...
    'OutDataTypeStr', 'T_GJVoltage');

add_block('simulink/Signal Routing/Bus Creator', ...
    [model '/loop_input_bus_creator'], ...
    'Position', [300 65 315 315], ...
    'Inputs', '5', ...
    'UseBusObject', 'on', ...
    'BusObject', 'green_joint_current_loop_input_t', ...
    'NonVirtualBus', 'on');
end

function add_controller(model, source_model)
add_block('simulink/Ports & Subsystems/Model', ...
    [model '/GreenJointCurrentLoopModelRef'], ...
    'Position', [390 130 610 230], ...
    'ModelName', source_model);

add_block('simulink/Signal Routing/Bus Selector', ...
    [model '/loop_output_selector'], ...
    'Position', [665 85 685 285], ...
    'OutputSignals', 'vd_cmd,vq_cmd,voltage_mag,vd_norm,vq_norm,voltage_mag_norm');

add_block('simulink/Signal Attributes/Data Type Conversion', ...
    [model '/vd_to_double'], ...
    'Position', [735 85 810 115], ...
    'OutDataTypeStr', 'double');
add_block('simulink/Signal Attributes/Data Type Conversion', ...
    [model '/vq_to_double'], ...
    'Position', [735 145 810 175], ...
    'OutDataTypeStr', 'double');
end

function add_dq_average_plant(model)
% D-axis plant.
add_block('simulink/Discrete/Unit Delay', [model '/id_state'], ...
    'Position', [1000 360 1065 390], ...
    'InitialCondition', '0', ...
    'SampleTime', 'GJDT_Ts');
add_block('simulink/Math Operations/Gain', [model '/Rs_id'], ...
    'Position', [1120 360 1190 390], ...
    'Gain', 'GJDT_Rs_Ohm');
add_block('simulink/Math Operations/Sum', [model '/vd_minus_Rs_id'], ...
    'Position', [1240 250 1270 295], ...
    'Inputs', '+-');
add_block('simulink/Math Operations/Gain', [model '/id_delta'], ...
    'Position', [1315 255 1400 285], ...
    'Gain', 'GJDT_TsPlant / GJDT_Ld_H');
add_block('simulink/Math Operations/Sum', [model '/id_next'], ...
    'Position', [1445 315 1475 360], ...
    'Inputs', '++');
add_block('simulink/Signal Attributes/Data Type Conversion', ...
    [model '/id_to_current'], ...
    'Position', [1115 435 1190 465], ...
    'OutDataTypeStr', 'T_GJCurrent');

% Q-axis plant.
add_block('simulink/Discrete/Unit Delay', [model '/iq_state'], ...
    'Position', [1000 535 1065 565], ...
    'InitialCondition', '0', ...
    'SampleTime', 'GJDT_Ts');
add_block('simulink/Math Operations/Gain', [model '/Rs_iq'], ...
    'Position', [1120 535 1190 565], ...
    'Gain', 'GJDT_Rs_Ohm');
add_block('simulink/Math Operations/Sum', [model '/vq_minus_Rs_iq'], ...
    'Position', [1240 475 1270 520], ...
    'Inputs', '+-');
add_block('simulink/Math Operations/Gain', [model '/iq_delta'], ...
    'Position', [1315 480 1400 510], ...
    'Gain', 'GJDT_TsPlant / GJDT_Lq_H');
add_block('simulink/Math Operations/Sum', [model '/iq_next'], ...
    'Position', [1445 520 1475 565], ...
    'Inputs', '++');
add_block('simulink/Signal Attributes/Data Type Conversion', ...
    [model '/iq_to_current'], ...
    'Position', [1115 610 1190 640], ...
    'OutDataTypeStr', 'T_GJCurrent');
end

function add_logs(model)
log_specs = { ...
    'iq_ref_log', 'gjdt_iq_ref', [1535 120 1605 150]; ...
    'id_log', 'gjdt_id', [1535 435 1605 465]; ...
    'iq_log', 'gjdt_iq', [1535 610 1605 640]; ...
    'vd_log', 'gjdt_vd', [880 85 950 115]; ...
    'vq_log', 'gjdt_vq', [880 145 950 175]; ...
    'voltage_mag_norm_log', 'gjdt_voltage_mag_norm', [880 255 950 285]};

for i = 1:size(log_specs, 1)
    add_block('simulink/Sinks/To Workspace', [model '/' log_specs{i, 1}], ...
        'Position', log_specs{i, 3}, ...
        'VariableName', log_specs{i, 2}, ...
        'SaveFormat', 'Structure With Time');
end
end

function connect_model(model)
name_line(add_line(model, 'id_ref/1', 'loop_input_bus_creator/1', ...
    'autorouting', 'on'), 'id_ref');
add_line(model, 'iq_ref_step/1', 'iq_ref_to_current/1', 'autorouting', 'on');
name_line(add_line(model, 'iq_ref_to_current/1', ...
    'loop_input_bus_creator/2', 'autorouting', 'on'), 'iq_ref');
name_line(add_line(model, 'id_to_current/1', 'loop_input_bus_creator/3', ...
    'autorouting', 'on'), 'id_fbk');
name_line(add_line(model, 'iq_to_current/1', 'loop_input_bus_creator/4', ...
    'autorouting', 'on'), 'iq_fbk');
name_line(add_line(model, 'vbus/1', 'loop_input_bus_creator/5', ...
    'autorouting', 'on'), 'vbus');

add_line(model, 'loop_input_bus_creator/1', ...
    'GreenJointCurrentLoopModelRef/1', 'autorouting', 'on');
add_line(model, 'GreenJointCurrentLoopModelRef/1', ...
    'loop_output_selector/1', 'autorouting', 'on');

add_line(model, 'loop_output_selector/1', 'vd_to_double/1', ...
    'autorouting', 'on');
add_line(model, 'loop_output_selector/2', 'vq_to_double/1', ...
    'autorouting', 'on');

add_line(model, 'vd_to_double/1', 'vd_minus_Rs_id/1', 'autorouting', 'on');
add_line(model, 'id_state/1', 'Rs_id/1', 'autorouting', 'on');
add_line(model, 'Rs_id/1', 'vd_minus_Rs_id/2', 'autorouting', 'on');
add_line(model, 'vd_minus_Rs_id/1', 'id_delta/1', 'autorouting', 'on');
add_line(model, 'id_delta/1', 'id_next/1', 'autorouting', 'on');
add_line(model, 'id_state/1', 'id_next/2', 'autorouting', 'on');
add_line(model, 'id_next/1', 'id_state/1', 'autorouting', 'on');
add_line(model, 'id_state/1', 'id_to_current/1', 'autorouting', 'on');

add_line(model, 'vq_to_double/1', 'vq_minus_Rs_iq/1', 'autorouting', 'on');
add_line(model, 'iq_state/1', 'Rs_iq/1', 'autorouting', 'on');
add_line(model, 'Rs_iq/1', 'vq_minus_Rs_iq/2', 'autorouting', 'on');
add_line(model, 'vq_minus_Rs_iq/1', 'iq_delta/1', 'autorouting', 'on');
add_line(model, 'iq_delta/1', 'iq_next/1', 'autorouting', 'on');
add_line(model, 'iq_state/1', 'iq_next/2', 'autorouting', 'on');
add_line(model, 'iq_next/1', 'iq_state/1', 'autorouting', 'on');
add_line(model, 'iq_state/1', 'iq_to_current/1', 'autorouting', 'on');

add_line(model, 'iq_ref_to_current/1', 'iq_ref_log/1', 'autorouting', 'on');
add_line(model, 'id_to_current/1', 'id_log/1', 'autorouting', 'on');
add_line(model, 'iq_to_current/1', 'iq_log/1', 'autorouting', 'on');
add_line(model, 'vd_to_double/1', 'vd_log/1', 'autorouting', 'on');
add_line(model, 'vq_to_double/1', 'vq_log/1', 'autorouting', 'on');
add_line(model, 'loop_output_selector/6', ...
    'voltage_mag_norm_log/1', 'autorouting', 'on');
end

function assert_safe_rebuild_environment(script_dir)
if strcmp(getenv('GJ_DT_ALLOW_UNSAFE_REBUILD'), '1')
    return;
end

if usejava('desktop')
    error(['This script rebuilds a generated .slx file. Run it from ' ...
        'matlab -batch, or set GJ_DT_ALLOW_UNSAFE_REBUILD=1 only after ' ...
        'closing the model in MATLAB Desktop.']);
end

model_file = fullfile(script_dir, 'green_joint_current_loop_twin_model.slx');
if ~exist(model_file, 'file')
    return;
end

[status, output] = system(sprintf('lsof -F pc -- %s 2>/dev/null', ...
    shell_quote(model_file)));
if status ~= 0 || strlength(strtrim(string(output))) == 0
    return;
end

current_pid = feature('getpid');
lines = splitlines(strtrim(string(output)));
pid = NaN;
command_name = "";
holders = strings(0, 1);
for i = 1:numel(lines)
    line = lines(i);
    if startsWith(line, "p")
        pid = str2double(extractAfter(line, 1));
    elseif startsWith(line, "c")
        command_name = extractAfter(line, 1);
    end

    if ~isnan(pid) && command_name ~= ""
        if pid ~= current_pid && contains(command_name, "MATLAB")
            holders(end + 1, 1) = sprintf('%s held by PID %d (%s)', ...
                model_file, pid, command_name); %#ok<AGROW>
        end
        pid = NaN;
        command_name = "";
    end
end

if ~isempty(holders)
    error(['Another MATLAB process appears to hold the twin model. ' ...
        'Close it before rebuilding.\n%s'], strjoin(holders, newline));
end
end

function result = should_open_existing_model_from_desktop(model_file)
result = false;
if strcmp(getenv('GJ_DT_ALLOW_UNSAFE_REBUILD'), '1')
    return;
end

if usejava('desktop') && exist(model_file, 'file')
    result = true;
end
end

function close_loaded_model(model)
if bdIsLoaded(model)
    close_system(model, 0);
end
end

function rmpath_if_present(path_name)
if contains(path, path_name)
    rmpath(path_name);
end
end

function quoted = shell_quote(text)
quoted = ['''' strrep(text, '''', '''"''"''') ''''];
end

function name_line(line_handle, signal_name)
set_param(line_handle, 'Name', signal_name);
end
