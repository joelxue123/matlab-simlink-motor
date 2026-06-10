%% Build a minimal reentrant Simulink model for embedded C code generation
%
% Algorithm:
%   z = x + y
%
% Design notes:
%   - Root model uses ERT.
%   - The algorithm is inside an atomic reusable subsystem.
%   - Customer-facing types are stored in add_interface.sldd.
%   - No states, globals, tunable parameters, or dynamic allocation are used.
%   - The reusable subsystem function is named AddStep.

clc;

cfg = customer_interface_config();
script_dir = fileparts(mfilename('fullpath'));
cfg.dictionaryFile = fullfile(script_dir, cfg.dictionaryName);
cfg.xDataType = cfg.inputTypeName;
cfg.yDataType = cfg.inputTypeName;
cfg.zDataType = cfg.outputTypeName;
cfg.accumulatorDataType = cfg.accumulatorTypeName;
assignin('base', 'reentrant_add_codegen_config', cfg);

create_add_data_dictionary(cfg);

model = cfg.modelName;
model_file = fullfile(script_dir, [model '.slx']);

if bdIsLoaded(model)
    close_system(model, 0);
end

if exist(model_file, 'file')
    delete(model_file);
end

new_system(model);
set_param(model, 'DataDictionary', cfg.dictionaryName);

set_param(model, ...
    'StopTime', '0.001', ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', cfg.sampleTime, ...
    'SystemTargetFile', 'ert.tlc', ...
    'ProdHWDeviceType', 'ARM Compatible->ARM Cortex', ...
    'TargetHWDeviceType', 'ARM Compatible->ARM Cortex', ...
    'GenerateReport', 'on', ...
    'RTWVerbose', 'on', ...
    'GenCodeOnly', 'on', ...
    'GenerateSampleERTMain', 'off', ...
    'SupportContinuousTime', 'off', ...
    'InlineParams', 'on');

set_param(model, ...
    'CodeInterfacePackaging', 'Reusable function', ...
    'ModelReferenceNumInstancesAllowed', 'Multi');

add_block('simulink/Sources/In1', [model '/x'], ...
    'Position', [45 80 75 100], ...
    'OutDataTypeStr', cfg.xDataType, ...
    'PortDimensions', '1');

add_block('simulink/Sources/In1', [model '/y'], ...
    'Position', [45 150 75 170], ...
    'OutDataTypeStr', cfg.yDataType, ...
    'PortDimensions', '1');

add_add_subsystem(model, cfg);

add_block('simulink/Sinks/Out1', [model '/z'], ...
    'Position', [405 115 435 135], ...
    'OutDataTypeStr', cfg.zDataType, ...
    'PortDimensions', '1');

add_line(model, 'x/1', [cfg.functionName '/1'], 'autorouting', 'on');
add_line(model, 'y/1', [cfg.functionName '/2'], 'autorouting', 'on');
add_line(model, [cfg.functionName '/1'], 'z/1', 'autorouting', 'on');

Simulink.BlockDiagram.arrangeSystem(model);
set_param(model, 'SimulationCommand', 'update');
save_system(model, model_file);

fprintf('Built Simulink model:\n  %s\n', model_file);

function cfg = customer_interface_config()
% Customer-editable interface contract.
%
% Modern workflow:
%   1) Define project types in a Simulink Data Dictionary.
%   2) Use AliasType names in model ports and blocks.
%   3) Let Code Mappings / Embedded Coder Dictionary handle storage/interface
%      policy when the model grows.
%
% Change the base type strings below; the model will keep using T_AddIn,
% T_AddOut, and T_AddAcc.
cfg.modelName = 'reentrant_add_model';
cfg.functionName = 'AddStep';
cfg.sampleTime = '0.001';
cfg.dictionaryName = 'add_interface.sldd';
cfg.typeHeaderFile = 'add_types.h';

cfg.inputTypeName = 'T_AddIn';
cfg.outputTypeName = 'T_AddOut';
cfg.accumulatorTypeName = 'T_AddAcc';

cfg.inputBaseType = 'int16';
cfg.outputBaseType = 'int16';
cfg.accumulatorBaseType = 'int16';

cfg.roundingMethod = 'Floor';
cfg.saturateOnIntegerOverflow = 'on';
end

function create_add_data_dictionary(cfg)
if exist(cfg.dictionaryFile, 'file')
    dd = Simulink.data.dictionary.open(cfg.dictionaryFile);
else
    dd = Simulink.data.dictionary.create(cfg.dictionaryFile);
end

design_data = getSection(dd, 'Design Data');

upsert_alias_type(design_data, cfg.inputTypeName, ...
    cfg.inputBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.outputTypeName, ...
    cfg.outputBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.accumulatorTypeName, ...
    cfg.accumulatorBaseType, cfg.typeHeaderFile);

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

function add_add_subsystem(model, cfg)
block_name = cfg.functionName;
subsystem = [model '/' block_name];

add_block('simulink/Ports & Subsystems/Subsystem', subsystem, ...
    'Position', [170 80 300 170]);
delete_line(subsystem, 'In1/1', 'Out1/1');
delete_block([subsystem '/In1']);
delete_block([subsystem '/Out1']);

set_param(subsystem, ...
    'TreatAsAtomicUnit', 'on', ...
    'RTWSystemCode', 'Reusable function', ...
    'RTWFcnNameOpts', 'User specified', ...
    'RTWFcnName', block_name, ...
    'RTWFileNameOpts', 'User specified', ...
    'RTWFileName', block_name);

add_block('simulink/Sources/In1', [subsystem '/x'], ...
    'Position', [45 60 75 80], ...
    'OutDataTypeStr', cfg.xDataType, ...
    'PortDimensions', '1');

add_block('simulink/Sources/In1', [subsystem '/y'], ...
    'Position', [45 125 75 145], ...
    'OutDataTypeStr', cfg.yDataType, ...
    'PortDimensions', '1');

add_block('simulink/Math Operations/Sum', [subsystem '/sum'], ...
    'Position', [145 82 175 123], ...
    'Inputs', '++', ...
    'OutDataTypeStr', cfg.zDataType, ...
    'AccumDataTypeStr', cfg.accumulatorDataType, ...
    'RndMeth', cfg.roundingMethod, ...
    'SaturateOnIntegerOverflow', cfg.saturateOnIntegerOverflow);

add_block('simulink/Sinks/Out1', [subsystem '/z'], ...
    'Position', [245 95 275 115], ...
    'OutDataTypeStr', cfg.zDataType, ...
    'PortDimensions', '1');

add_line(subsystem, 'x/1', 'sum/1', 'autorouting', 'on');
add_line(subsystem, 'y/1', 'sum/2', 'autorouting', 'on');
add_line(subsystem, 'sum/1', 'z/1', 'autorouting', 'on');

Simulink.BlockDiagram.arrangeSystem(subsystem);
end
