%% Build a simple legacy current feedback filter MBD module
%
% This module models the dq feedback low-pass that used to be in green-joint:
%
%   raw_id/raw_iq/v_mag_norm -> CurrentFilterStep -> id_f/iq_f/alpha
%
% The firmware mainline currently feeds raw Park id/iq directly to current PI.
% Keep this model only as a reusable experiment unless digital-twin validation
% proves that dq feedback filtering should be restored.

clear;
clc;

cfg = current_filter_config();
script_dir = fileparts(mfilename('fullpath'));
cfg.dictionaryFile = fullfile(script_dir, cfg.dictionaryName);

previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

assignin('base', 'current_filter_test', default_current_filter_test());

if bdIsLoaded(cfg.modelName)
    close_system(cfg.modelName, 0);
end

close_open_data_dictionaries();
create_current_filter_dictionary(cfg);

model_file = fullfile(script_dir, [cfg.modelName '.slx']);
if exist(model_file, 'file')
    delete(model_file);
end

new_system(cfg.modelName);
set_param(cfg.modelName, ...
    'DataDictionary', cfg.dictionaryName, ...
    'StopTime', '0.020', ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', '5e-5', ...
    'SystemTargetFile', 'ert.tlc', ...
    'GenerateReport', 'on', ...
    'RTWVerbose', 'on', ...
    'GenCodeOnly', 'on', ...
    'GenerateSampleERTMain', 'off', ...
    'SupportContinuousTime', 'off', ...
    'InlineParams', 'off', ...
    'CodeInterfacePackaging', 'Reusable function');

add_test_harness(cfg.modelName, cfg);
add_current_filter_subsystem(cfg.modelName, cfg);
connect_top_level(cfg.modelName);

set_param(cfg.modelName, 'SimulationCommand', 'update');
save_system(cfg.modelName, model_file);

fprintf('Built current filter MBD model:\n  %s\n', model_file);

function cfg = current_filter_config()
cfg.modelName = 'current_filter_model';
cfg.dictionaryName = 'current_filter_interface.sldd';
cfg.typeHeaderFile = 'current_filter_types.h';
cfg.rebuildDictionary = true;

cfg.realTypeName = 'T_CurrentFilterReal';
cfg.currentTypeName = 'T_CurrentFilterCurrent';
cfg.alphaTypeName = 'T_CurrentFilterAlpha';

cfg.inputBusName = 'current_filter_input_t';
cfg.outputBusName = 'current_filter_output_t';

cfg.floatBaseType = 'single';
end

function test = default_current_filter_test()
test.id_raw = single(0);
test.iq_raw = single(10);
test.v_mag_norm = single(0.1);
end

function close_open_data_dictionaries()
try
    Simulink.data.dictionary.closeAll('-discard');
catch err
    warning('Could not close open dictionaries: %s', err.message);
end
end

function create_current_filter_dictionary(cfg)
if isfield(cfg, 'rebuildDictionary') && cfg.rebuildDictionary && exist(cfg.dictionaryFile, 'file')
    delete(cfg.dictionaryFile);
end

if exist(cfg.dictionaryFile, 'file')
    dd = Simulink.data.dictionary.open(cfg.dictionaryFile);
else
    dd = Simulink.data.dictionary.create(cfg.dictionaryFile);
end

design_data = getSection(dd, 'Design Data');

upsert_alias_type(design_data, cfg.realTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.currentTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.alphaTypeName, cfg.floatBaseType, cfg.typeHeaderFile);

upsert_bus_type(design_data, cfg.inputBusName, cfg.typeHeaderFile, { ...
    {'id_raw', cfg.currentTypeName}; ...
    {'iq_raw', cfg.currentTypeName}; ...
    {'v_mag_norm', cfg.realTypeName}});

upsert_bus_type(design_data, cfg.outputBusName, cfg.typeHeaderFile, { ...
    {'id_f', cfg.currentTypeName}; ...
    {'iq_f', cfg.currentTypeName}; ...
    {'alpha', cfg.alphaTypeName}});

saveChanges(dd);
close(dd);
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

function add_test_harness(model, cfg)
sources = { ...
    'id_raw', 'current_filter_test.id_raw', cfg.currentTypeName, [35 45 105 70]; ...
    'iq_raw', 'current_filter_test.iq_raw', cfg.currentTypeName, [35 90 105 115]; ...
    'v_mag_norm', 'current_filter_test.v_mag_norm', cfg.realTypeName, [35 135 105 160]};

for i = 1:size(sources, 1)
    add_block('simulink/Sources/Constant', [model '/' sources{i, 1}], ...
        'Position', sources{i, 4}, ...
        'Value', sources{i, 2}, ...
        'OutDataTypeStr', sources{i, 3}, ...
        'SampleTime', '5e-5');
end

add_block('simulink/Signal Routing/Bus Creator', [model '/filter_input_bus_creator'], ...
    'Position', [155 45 170 160], ...
    'Inputs', '3', ...
    'UseBusObject', 'on', ...
    'BusObject', cfg.inputBusName, ...
    'NonVirtualBus', 'on');

name_line(add_line(model, 'id_raw/1', 'filter_input_bus_creator/1', 'autorouting', 'on'), 'id_raw');
name_line(add_line(model, 'iq_raw/1', 'filter_input_bus_creator/2', 'autorouting', 'on'), 'iq_raw');
name_line(add_line(model, 'v_mag_norm/1', 'filter_input_bus_creator/3', 'autorouting', 'on'), 'v_mag_norm');

add_block('simulink/Signal Routing/Bus Selector', [model '/filter_output_selector'], ...
    'Position', [430 65 450 155], ...
    'OutputSignals', 'id_f,iq_f,alpha');

out_names = {'id_f', 'iq_f', 'alpha'};
for i = 1:numel(out_names)
    add_block('simulink/Sinks/Out1', [model '/' out_names{i}], ...
        'Position', [505 50 + (i - 1) * 40 535 70 + (i - 1) * 40], ...
        'Port', num2str(i));
end

add_scalar_logger(model, 'log_id_f', [575 45 665 75], 'filter_output_selector/1');
add_scalar_logger(model, 'log_iq_f', [575 85 665 115], 'filter_output_selector/2');
add_scalar_logger(model, 'log_alpha', [575 125 665 155], 'filter_output_selector/3');
end

function add_current_filter_subsystem(model, cfg)
subsystem = [model '/CurrentFilterStep'];
add_block('simulink/Ports & Subsystems/Subsystem', subsystem, ...
    'Position', [250 70 380 150]);
delete_line(subsystem, 'In1/1', 'Out1/1');
delete_block([subsystem '/In1']);
delete_block([subsystem '/Out1']);

set_param(subsystem, ...
    'TreatAsAtomicUnit', 'on', ...
    'RTWSystemCode', 'Reusable function', ...
    'RTWFcnNameOpts', 'User specified', ...
    'RTWFcnName', 'CurrentFilterStep', ...
    'RTWFileNameOpts', 'User specified', ...
    'RTWFileName', 'CurrentFilterStep');

add_block('simulink/Sources/In1', [subsystem '/filter_in'], ...
    'Position', [35 95 65 115], ...
    'OutDataTypeStr', ['Bus: ' cfg.inputBusName], ...
    'BusOutputAsStruct', 'on');
add_block('simulink/Signal Routing/Bus Selector', [subsystem '/input_selector'], ...
    'Position', [100 50 120 165], ...
    'OutputSignals', 'id_raw,iq_raw,v_mag_norm');

add_alpha_blocks(subsystem, cfg);
add_iir_axis(subsystem, cfg, 'id', 'input_selector/1', [315 35]);
add_iir_axis(subsystem, cfg, 'iq', 'input_selector/2', [315 175]);

add_block('simulink/Signal Routing/Bus Creator', [subsystem '/filter_output_bus_creator'], ...
    'Position', [585 85 600 185], ...
    'Inputs', '3', ...
    'UseBusObject', 'on', ...
    'BusObject', cfg.outputBusName, ...
    'NonVirtualBus', 'on');
add_block('simulink/Sinks/Out1', [subsystem '/filter_out'], ...
    'Position', [645 125 675 145], ...
    'OutDataTypeStr', ['Bus: ' cfg.outputBusName]);

name_line(add_line(subsystem, 'filter_in/1', 'input_selector/1', 'autorouting', 'on'), 'current_filter_input_t');
add_line(subsystem, 'input_selector/3', 'alpha_saturation/1', 'autorouting', 'on');
name_line(add_line(subsystem, 'id_next/1', 'filter_output_bus_creator/1', 'autorouting', 'on'), 'id_f');
name_line(add_line(subsystem, 'iq_next/1', 'filter_output_bus_creator/2', 'autorouting', 'on'), 'iq_f');
name_line(add_line(subsystem, 'alpha/1', 'filter_output_bus_creator/3', 'autorouting', 'on'), 'alpha');
add_line(subsystem, 'filter_output_bus_creator/1', 'filter_out/1', 'autorouting', 'on');

Simulink.BlockDiagram.arrangeSystem(subsystem);
end

function add_alpha_blocks(subsystem, cfg)
add_block('simulink/Discontinuities/Saturation', [subsystem '/alpha_saturation'], ...
    'Position', [165 130 215 160], ...
    'LowerLimit', 'single(0.5)', ...
    'UpperLimit', 'single(0.9)');
add_block('simulink/Math Operations/Bias', [subsystem '/alpha_minus_low_threshold'], ...
    'Position', [240 130 290 160], ...
    'Bias', 'single(-0.5)');
add_block('simulink/Math Operations/Gain', [subsystem '/alpha_slope'], ...
    'Position', [315 130 365 160], ...
    'Gain', '-1.125', ...
    'ParamDataTypeStr', cfg.realTypeName);
add_block('simulink/Math Operations/Bias', [subsystem '/alpha'], ...
    'Position', [390 130 440 160], ...
    'Bias', 'single(0.95)');

add_line(subsystem, 'alpha_saturation/1', 'alpha_minus_low_threshold/1', 'autorouting', 'on');
add_line(subsystem, 'alpha_minus_low_threshold/1', 'alpha_slope/1', 'autorouting', 'on');
add_line(subsystem, 'alpha_slope/1', 'alpha/1', 'autorouting', 'on');
end

function add_iir_axis(subsystem, cfg, axis_name, raw_signal, origin)
x0 = origin(1);
y0 = origin(2);

state_name = [axis_name '_state'];
delta_name = [axis_name '_delta'];
step_name = [axis_name '_step'];
next_name = [axis_name '_next'];

add_block('simulink/Discrete/Unit Delay', [subsystem '/' state_name], ...
    'Position', [x0 y0 x0 + 45 y0 + 25], ...
    'InitialCondition', '0');
add_block('simulink/Math Operations/Sum', [subsystem '/' delta_name], ...
    'Position', [x0 + 75 y0 x0 + 105 y0 + 25], ...
    'Inputs', '+-');
add_block('simulink/Math Operations/Product', [subsystem '/' step_name], ...
    'Position', [x0 + 140 y0 x0 + 170 y0 + 25], ...
    'OutDataTypeStr', 'Inherit: Inherit via internal rule');
add_block('simulink/Math Operations/Sum', [subsystem '/' next_name], ...
    'Position', [x0 + 205 y0 x0 + 235 y0 + 25], ...
    'Inputs', '++');

add_line(subsystem, raw_signal, [delta_name '/1'], 'autorouting', 'on');
add_line(subsystem, [state_name '/1'], [delta_name '/2'], 'autorouting', 'on');
add_line(subsystem, [delta_name '/1'], [step_name '/1'], 'autorouting', 'on');
add_line(subsystem, 'alpha/1', [step_name '/2'], 'autorouting', 'on');
add_line(subsystem, [state_name '/1'], [next_name '/1'], 'autorouting', 'on');
add_line(subsystem, [step_name '/1'], [next_name '/2'], 'autorouting', 'on');
add_line(subsystem, [next_name '/1'], [state_name '/1'], 'autorouting', 'on');
end

function connect_top_level(model)
add_line(model, 'filter_input_bus_creator/1', 'CurrentFilterStep/1', 'autorouting', 'on');
add_line(model, 'CurrentFilterStep/1', 'filter_output_selector/1', 'autorouting', 'on');
add_line(model, 'filter_output_selector/1', 'id_f/1', 'autorouting', 'on');
add_line(model, 'filter_output_selector/2', 'iq_f/1', 'autorouting', 'on');
add_line(model, 'filter_output_selector/3', 'alpha/1', 'autorouting', 'on');

Simulink.BlockDiagram.arrangeSystem(model);
end

function name_line(line_handle, signal_name)
set_param(line_handle, 'Name', signal_name);
end

function add_scalar_logger(model, variable_name, position, src_port)
add_block('simulink/Sinks/To Workspace', [model '/' variable_name], ...
    'Position', position, ...
    'VariableName', variable_name, ...
    'SaveFormat', 'Structure With Time');
add_line(model, src_port, [variable_name '/1'], 'autorouting', 'on');
end
