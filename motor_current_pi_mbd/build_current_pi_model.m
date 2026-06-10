%% Build a float-based current PI MBD module
%
% This module is intentionally built from Simulink primitive blocks rather
% than a MATLAB Function block or a black-box PID Controller block.
%
% Architecture:
%   current_pi_input_t
%     -> CurrentPiStep
%     -> current_pi_output_t

clc;

cfg = customer_interface_config();
script_dir = fileparts(mfilename('fullpath'));
cfg.dictionaryFile = fullfile(script_dir, cfg.dictionaryName);
assignin('base', 'current_pi_config', cfg);

previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

defaults = default_current_pi_params();
assignin('base', 'current_pi_defaults', defaults);
assignin('base', 'current_pi_simcfg', defaults.simcfg);
assignin('base', 'current_pi_test', defaults.test);

model = cfg.modelName;
model_file = fullfile(script_dir, [model '.slx']);

if bdIsLoaded(model)
    close_system(model, 0);
end

close_open_data_dictionaries();
create_current_pi_data_dictionary(cfg, defaults);

if exist(model_file, 'file')
    delete(model_file);
end

new_system(model);
set_param(model, 'DataDictionary', cfg.dictionaryName);
set_param(model, ...
    'StopTime', 'current_pi_simcfg.stop_time', ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', 'current_pi_simcfg.Ts_ctrl', ...
    'SystemTargetFile', 'ert.tlc', ...
    'ProdHWDeviceType', 'ARM Compatible->ARM Cortex', ...
    'TargetHWDeviceType', 'ARM Compatible->ARM Cortex', ...
    'GenerateReport', 'on', ...
    'RTWVerbose', 'on', ...
    'GenCodeOnly', 'on', ...
    'GenerateSampleERTMain', 'off', ...
    'SupportContinuousTime', 'off', ...
    'InlineParams', 'off');

set_param(model, ...
    'CodeInterfacePackaging', 'Reusable function', ...
    'ModelReferenceNumInstancesAllowed', 'Multi');

add_current_pi_test_sources(model, cfg);
add_current_pi_subsystem(model, cfg);
add_output_logs(model);

set_param(model, 'SimulationCommand', 'update');
save_system(model, model_file);

fprintf('Built current PI MBD model:\n  %s\n', model_file);

function cfg = customer_interface_config()
% Customer-editable interface and architecture contract.
cfg.modelName = 'current_pi_model';
cfg.sampleTime = 'current_pi_simcfg.Ts_ctrl';

cfg.dictionaryName = 'current_pi_interface.sldd';
cfg.typeHeaderFile = 'current_pi_types.h';
cfg.rebuildDictionary = true;

cfg.realTypeName = 'T_CurrentPiFloat';
cfg.currentTypeName = 'T_CurrentPiCurrent';
cfg.voltageTypeName = 'T_CurrentPiVoltage';
cfg.speedTypeName = 'T_CurrentPiSpeed';
cfg.gainTypeName = 'T_CurrentPiGain';

cfg.inputBusName = 'current_pi_input_t';
cfg.outputBusName = 'current_pi_output_t';

cfg.floatBaseType = 'single';
end

function defaults = default_current_pi_params()
% Baseline numbers are derived from average-inverter/motor_control_params.m.
defaults.simcfg.stop_time = 0.030;
defaults.simcfg.Ts_ctrl = 50e-6;

line_to_line_resistance = 0.4267;
line_to_line_inductance = 0.53e-3;
Rs = line_to_line_resistance / 2;
Ld = line_to_line_inductance / 2;
current_bandwidth_rad_s = 2 * pi * 800;

defaults.Kp_id = single(Ld * current_bandwidth_rad_s);
defaults.Ki_id = single(Rs * current_bandwidth_rad_s);
defaults.Kaw_id = single(400);
defaults.Kp_iq = defaults.Kp_id;
defaults.Ki_iq = defaults.Ki_id;
defaults.Kaw_iq = defaults.Kaw_id;
defaults.VLimitRatio = single(0.577);

defaults.test.id_ref = single(0);
defaults.test.iq_ref = single(10);
defaults.test.id_meas = single(0);
defaults.test.iq_meas = single(0);
defaults.test.omega_e = single(0);
defaults.test.vdc = single(68);
end

function close_open_data_dictionaries()
try
    Simulink.data.dictionary.closeAll('-discard');
catch err
    warning('Could not close open data dictionaries before rebuild: %s', ...
        err.message);
end
end

function create_current_pi_data_dictionary(cfg, defaults)
dd = open_or_create_data_dictionary(cfg);
design_data = getSection(dd, 'Design Data');

upsert_alias_type(design_data, cfg.realTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.currentTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.voltageTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.speedTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.gainTypeName, cfg.floatBaseType, cfg.typeHeaderFile);

upsert_bus_type(design_data, cfg.inputBusName, cfg.typeHeaderFile, { ...
    {'id_ref', cfg.currentTypeName}; ...
    {'iq_ref', cfg.currentTypeName}; ...
    {'id_meas', cfg.currentTypeName}; ...
    {'iq_meas', cfg.currentTypeName}; ...
    {'omega_e', cfg.speedTypeName}; ...
    {'vdc', cfg.voltageTypeName}});

upsert_bus_type(design_data, cfg.outputBusName, cfg.typeHeaderFile, { ...
    {'vd_ref', cfg.voltageTypeName}; ...
    {'vq_ref', cfg.voltageTypeName}});

upsert_parameter(design_data, 'Kp_id', defaults.Kp_id, cfg.gainTypeName);
upsert_parameter(design_data, 'Ki_id', defaults.Ki_id, cfg.gainTypeName);
upsert_parameter(design_data, 'Kaw_id', defaults.Kaw_id, cfg.gainTypeName);
upsert_parameter(design_data, 'Kp_iq', defaults.Kp_iq, cfg.gainTypeName);
upsert_parameter(design_data, 'Ki_iq', defaults.Ki_iq, cfg.gainTypeName);
upsert_parameter(design_data, 'Kaw_iq', defaults.Kaw_iq, cfg.gainTypeName);
upsert_parameter(design_data, 'VLimitRatio', defaults.VLimitRatio, cfg.gainTypeName);

saveChanges(dd);
close(dd);
end

function dd = open_or_create_data_dictionary(cfg)
if isfield(cfg, 'rebuildDictionary') && cfg.rebuildDictionary && exist(cfg.dictionaryFile, 'file')
    delete(cfg.dictionaryFile);
end

if exist(cfg.dictionaryFile, 'file')
    dd = Simulink.data.dictionary.open(cfg.dictionaryFile);
else
    dd = Simulink.data.dictionary.create(cfg.dictionaryFile);
end
end

function upsert_alias_type(section, type_name, base_type, header_file)
alias_type = Simulink.AliasType(base_type);
alias_type.DataScope = 'Exported';
alias_type.HeaderFile = header_file;

entry = find(section, 'Name', type_name);
if isempty(entry)
    addEntry(section, type_name, alias_type);
else
    setValue(entry(1), alias_type);
end
end

function upsert_bus_type(section, bus_name, header_file, element_specs)
bus_elements = repmat(Simulink.BusElement, numel(element_specs), 1);
for i = 1:numel(element_specs)
    bus_elements(i).Name = element_specs{i}{1};
    bus_elements(i).DataType = element_specs{i}{2};
    bus_elements(i).Dimensions = 1;
end

bus_type = Simulink.Bus;
bus_type.Elements = bus_elements;
bus_type.DataScope = 'Exported';
bus_type.HeaderFile = header_file;

entry = find(section, 'Name', bus_name);
if isempty(entry)
    addEntry(section, bus_name, bus_type);
else
    setValue(entry(1), bus_type);
end
end

function upsert_parameter(section, name, value, data_type)
parameter = Simulink.Parameter(double(value));
parameter.DataType = data_type;
parameter.CoderInfo.StorageClass = 'Auto';

entry = find(section, 'Name', name);
if isempty(entry)
    addEntry(section, name, parameter);
else
    setValue(entry(1), parameter);
end
end

function add_current_pi_test_sources(model, cfg)
source_specs = { ...
    'id_ref', 'current_pi_test.id_ref', cfg.currentTypeName, [45 45 115 70]; ...
    'iq_ref', 'current_pi_test.iq_ref', cfg.currentTypeName, [45 85 115 110]; ...
    'id_meas', 'current_pi_test.id_meas', cfg.currentTypeName, [45 125 115 150]; ...
    'iq_meas', 'current_pi_test.iq_meas', cfg.currentTypeName, [45 165 115 190]; ...
    'omega_e', 'current_pi_test.omega_e', cfg.speedTypeName, [45 205 115 230]; ...
    'vdc', 'current_pi_test.vdc', cfg.voltageTypeName, [45 245 115 270]};

for i = 1:size(source_specs, 1)
    add_typed_constant(model, source_specs{i, 1}, source_specs{i, 2}, ...
        source_specs{i, 3}, source_specs{i, 4}, cfg.sampleTime);
end

add_block('simulink/Signal Routing/Bus Creator', [model '/pi_input_bus_creator'], ...
    'Position', [170 45 185 270], ...
    'Inputs', '6', ...
    'UseBusObject', 'on', ...
    'BusObject', cfg.inputBusName, ...
    'NonVirtualBus', 'on');

names = {'id_ref', 'iq_ref', 'id_meas', 'iq_meas', 'omega_e', 'vdc'};
for i = 1:numel(names)
    name_line(add_line(model, [names{i} '/1'], ...
        sprintf('pi_input_bus_creator/%d', i), 'autorouting', 'on'), names{i});
end
end

function add_current_pi_subsystem(model, cfg)
subsystem = [model '/CurrentPiStep'];
add_clean_subsystem(subsystem, [260 110 455 205]);

set_param(subsystem, ...
    'TreatAsAtomicUnit', 'on', ...
    'RTWSystemCode', 'Reusable function', ...
    'RTWFcnNameOpts', 'User specified', ...
    'RTWFcnName', 'CurrentPiStep', ...
    'RTWFileNameOpts', 'User specified', ...
    'RTWFileName', 'CurrentPiStep');

add_block('simulink/Sources/In1', [subsystem '/pi_in'], ...
    'Position', [35 135 65 155], ...
    'OutDataTypeStr', ['Bus: ' cfg.inputBusName], ...
    'BusOutputAsStruct', 'on');

add_block('simulink/Signal Routing/Bus Selector', [subsystem '/pi_input_selector'], ...
    'Position', [100 65 115 260], ...
    'OutputSignals', 'id_ref,iq_ref,id_meas,iq_meas,vdc');

add_block('simulink/Math Operations/Gain', [subsystem '/v_limit_calc'], ...
    'Position', [165 230 225 260], ...
    'Gain', 'VLimitRatio', ...
    'ParamDataTypeStr', cfg.gainTypeName, ...
    'OutDataTypeStr', cfg.voltageTypeName);
add_block('simulink/Math Operations/Gain', [subsystem '/neg_v_limit'], ...
    'Position', [255 230 315 260], ...
    'Gain', '-1', ...
    'ParamDataTypeStr', cfg.realTypeName, ...
    'OutDataTypeStr', cfg.voltageTypeName);

add_block('simulink/Signal Routing/Bus Creator', [subsystem '/pi_output_bus_creator'], ...
    'Position', [1130 120 1145 230], ...
    'Inputs', '2', ...
    'UseBusObject', 'on', ...
    'BusObject', cfg.outputBusName, ...
    'NonVirtualBus', 'on');

add_block('simulink/Sinks/Out1', [subsystem '/pi_out'], ...
    'Position', [1210 165 1240 185], ...
    'OutDataTypeStr', ['Bus: ' cfg.outputBusName]);

add_line(subsystem, 'pi_in/1', 'pi_input_selector/1', 'autorouting', 'on');
add_line(subsystem, 'pi_input_selector/5', 'v_limit_calc/1', 'autorouting', 'on');
add_line(subsystem, 'v_limit_calc/1', 'neg_v_limit/1', 'autorouting', 'on');

add_pi_axis(subsystem, cfg, 'id', 40, ...
    'pi_input_selector/1', 'pi_input_selector/3', ...
    'v_limit_calc/1', 'neg_v_limit/1', 'pi_output_bus_creator/1', ...
    'vd_ref');
add_pi_axis(subsystem, cfg, 'iq', 315, ...
    'pi_input_selector/2', 'pi_input_selector/4', ...
    'v_limit_calc/1', 'neg_v_limit/1', 'pi_output_bus_creator/2', ...
    'vq_ref');

add_line(subsystem, 'pi_output_bus_creator/1', 'pi_out/1', 'autorouting', 'on');

add_line(model, 'pi_input_bus_creator/1', 'CurrentPiStep/1', 'autorouting', 'on');
end

function add_pi_axis(subsystem, cfg, axis, y, ref_port, meas_port, limit_port, neg_limit_port, output_port, output_name)
kp_name = ['Kp_' axis];
ki_name = ['Ki_' axis];
kaw_name = ['Kaw_' axis];

add_block('simulink/Math Operations/Sum', [subsystem '/' axis '_error'], ...
    'Position', [170 y 200 y+35], ...
    'Inputs', '+-', ...
    'AccumDataTypeStr', cfg.currentTypeName, ...
    'OutDataTypeStr', cfg.currentTypeName);
add_block('simulink/Math Operations/Gain', [subsystem '/' axis '_kp'], ...
    'Position', [240 y 300 y+30], ...
    'Gain', kp_name, ...
    'ParamDataTypeStr', cfg.gainTypeName, ...
    'OutDataTypeStr', cfg.voltageTypeName);
add_block('simulink/Math Operations/Gain', [subsystem '/' axis '_ki'], ...
    'Position', [240 y+55 300 y+85], ...
    'Gain', ki_name, ...
    'ParamDataTypeStr', cfg.gainTypeName, ...
    'OutDataTypeStr', cfg.voltageTypeName);

add_block('simulink/Discrete/Unit Delay', [subsystem '/' axis '_integrator_state'], ...
    'Position', [365 y+95 430 y+125], ...
    'InitialCondition', '0', ...
    'SampleTime', cfg.sampleTime);

add_block('simulink/Math Operations/Sum', [subsystem '/' axis '_pre_sat_sum'], ...
    'Position', [470 y+10 500 y+55], ...
    'Inputs', '++', ...
    'AccumDataTypeStr', cfg.voltageTypeName, ...
    'OutDataTypeStr', cfg.voltageTypeName);
add_minmax(subsystem, [axis '_min_upper'], [545 y 605 y+30], 'min');
add_minmax(subsystem, [axis '_max_lower'], [650 y 710 y+30], 'max');

add_block('simulink/Math Operations/Sum', [subsystem '/' axis '_back_calc_error'], ...
    'Position', [745 y+45 775 y+90], ...
    'Inputs', '+-', ...
    'AccumDataTypeStr', cfg.voltageTypeName, ...
    'OutDataTypeStr', cfg.voltageTypeName);
add_block('simulink/Math Operations/Gain', [subsystem '/' axis '_kaw'], ...
    'Position', [815 y+50 875 y+80], ...
    'Gain', kaw_name, ...
    'ParamDataTypeStr', cfg.gainTypeName, ...
    'OutDataTypeStr', cfg.voltageTypeName);

add_block('simulink/Math Operations/Sum', [subsystem '/' axis '_integrator_rate'], ...
    'Position', [915 y+55 945 y+100], ...
    'Inputs', '++', ...
    'AccumDataTypeStr', cfg.voltageTypeName, ...
    'OutDataTypeStr', cfg.voltageTypeName);
add_block('simulink/Math Operations/Gain', [subsystem '/' axis '_integrator_delta'], ...
    'Position', [980 y+60 1040 y+90], ...
    'Gain', 'current_pi_simcfg.Ts_ctrl', ...
    'ParamDataTypeStr', cfg.realTypeName, ...
    'OutDataTypeStr', cfg.voltageTypeName);
add_block('simulink/Math Operations/Sum', [subsystem '/' axis '_integrator_next'], ...
    'Position', [1080 y+75 1110 y+120], ...
    'Inputs', '++', ...
    'AccumDataTypeStr', cfg.voltageTypeName, ...
    'OutDataTypeStr', cfg.voltageTypeName);

add_line(subsystem, ref_port, [axis '_error/1'], 'autorouting', 'on');
add_line(subsystem, meas_port, [axis '_error/2'], 'autorouting', 'on');
add_line(subsystem, [axis '_error/1'], [axis '_kp/1'], 'autorouting', 'on');
add_line(subsystem, [axis '_error/1'], [axis '_ki/1'], 'autorouting', 'on');
add_line(subsystem, [axis '_kp/1'], [axis '_pre_sat_sum/1'], 'autorouting', 'on');
add_line(subsystem, [axis '_integrator_state/1'], [axis '_pre_sat_sum/2'], 'autorouting', 'on');

add_line(subsystem, [axis '_pre_sat_sum/1'], [axis '_min_upper/1'], 'autorouting', 'on');
add_line(subsystem, limit_port, [axis '_min_upper/2'], 'autorouting', 'on');
add_line(subsystem, [axis '_min_upper/1'], [axis '_max_lower/1'], 'autorouting', 'on');
add_line(subsystem, neg_limit_port, [axis '_max_lower/2'], 'autorouting', 'on');

add_line(subsystem, [axis '_max_lower/1'], [axis '_back_calc_error/1'], 'autorouting', 'on');
add_line(subsystem, [axis '_pre_sat_sum/1'], [axis '_back_calc_error/2'], 'autorouting', 'on');
add_line(subsystem, [axis '_back_calc_error/1'], [axis '_kaw/1'], 'autorouting', 'on');
add_line(subsystem, [axis '_ki/1'], [axis '_integrator_rate/1'], 'autorouting', 'on');
add_line(subsystem, [axis '_kaw/1'], [axis '_integrator_rate/2'], 'autorouting', 'on');
add_line(subsystem, [axis '_integrator_rate/1'], [axis '_integrator_delta/1'], 'autorouting', 'on');
add_line(subsystem, [axis '_integrator_state/1'], [axis '_integrator_next/1'], 'autorouting', 'on');
add_line(subsystem, [axis '_integrator_delta/1'], [axis '_integrator_next/2'], 'autorouting', 'on');
add_line(subsystem, [axis '_integrator_next/1'], [axis '_integrator_state/1'], 'autorouting', 'on');

name_line(add_line(subsystem, [axis '_max_lower/1'], output_port, 'autorouting', 'on'), output_name);
end

function add_output_logs(model)
add_block('simulink/Signal Routing/Bus Selector', [model '/pi_output_selector'], ...
    'Position', [510 105 525 215], ...
    'OutputSignals', 'vd_ref,vq_ref');
add_line(model, 'CurrentPiStep/1', 'pi_output_selector/1', 'autorouting', 'on');

add_scalar_logger(model, 'log_vd_ref', [575 100 665 130], 'pi_output_selector/1');
add_scalar_logger(model, 'log_vq_ref', [575 155 665 185], 'pi_output_selector/2');

add_block('simulink/Signal Routing/Mux', [model '/pi_scope_mux'], ...
    'Position', [700 115 705 185], ...
    'Inputs', '2');
add_block('simulink/Sinks/Scope', [model '/Current PI Scope'], ...
    'Position', [750 120 815 180], ...
    'NumInputPorts', '1');
add_line(model, 'pi_output_selector/1', 'pi_scope_mux/1', 'autorouting', 'on');
add_line(model, 'pi_output_selector/2', 'pi_scope_mux/2', 'autorouting', 'on');
add_line(model, 'pi_scope_mux/1', 'Current PI Scope/1', 'autorouting', 'on');
end

function add_clean_subsystem(subsystem, position)
add_block('simulink/Ports & Subsystems/Subsystem', subsystem, ...
    'Position', position);
lines = find_system(subsystem, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line');
if ~isempty(lines)
    delete_line(lines);
end
blocks = find_system(subsystem, 'SearchDepth', 1, 'Type', 'Block');
for i = 2:numel(blocks)
    delete_block(blocks{i});
end
end

function add_typed_constant(model, block_name, value, data_type, position, sample_time)
add_block('simulink/Sources/Constant', [model '/' block_name], ...
    'Position', position, ...
    'Value', value, ...
    'OutDataTypeStr', data_type, ...
    'SampleTime', sample_time);
end

function add_scalar_logger(model, variable_name, position, src_port)
add_block('simulink/Sinks/To Workspace', [model '/' variable_name], ...
    'Position', position, ...
    'VariableName', variable_name, ...
    'SaveFormat', 'Timeseries');
add_line(model, src_port, [variable_name '/1'], 'autorouting', 'on');
end

function add_minmax(model, block_name, position, operator)
add_block('simulink/Math Operations/MinMax', [model '/' block_name], ...
    'Position', position, ...
    'Inputs', '2', ...
    'Function', operator);
end

function name_line(line_handle, signal_name)
set_param(line_handle, 'Name', signal_name);
end
