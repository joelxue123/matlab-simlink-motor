%% Build a reentrant Clarke/Park transform model with struct-style interfaces
%
% Input struct:
%   motor_t {
%     T_MotorCurrent ia;
%     T_MotorCurrent ib;
%     T_MotorCurrent ic;
%     T_MotorAngle   theta_e;
%   }
%
% Output struct:
%   motor_dq_t {
%     T_MotorCurrent i_alpha;
%     T_MotorCurrent i_beta;
%     T_MotorCurrent id;
%     T_MotorCurrent iq;
%   }

clc;

cfg = customer_interface_config();
script_dir = fileparts(mfilename('fullpath'));
cfg.dictionaryFile = fullfile(script_dir, cfg.dictionaryName);
assignin('base', 'motor_clarke_park_codegen_config', cfg);

model = cfg.modelName;
model_file = fullfile(script_dir, [model '.slx']);

if bdIsLoaded(model)
    close_system(model, 0);
end

close_open_data_dictionaries();
create_motor_data_dictionary(cfg);

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

add_block('simulink/Sources/In1', [model '/motor_in'], ...
    'Position', [45 105 75 125], ...
    'OutDataTypeStr', ['Bus: ' cfg.inputBusName], ...
    'BusOutputAsStruct', 'on', ...
    'PortDimensions', '1');

add_transform_subsystem(model, cfg);

add_block('simulink/Sinks/Out1', [model '/dq_out'], ...
    'Position', [405 105 435 125], ...
    'OutDataTypeStr', ['Bus: ' cfg.outputBusName], ...
    'PortDimensions', '1');

add_line(model, 'motor_in/1', [cfg.functionName '/1'], 'autorouting', 'on');
add_line(model, [cfg.functionName '/1'], 'dq_out/1', 'autorouting', 'on');

set_param(model, 'SimulationCommand', 'update');
save_system(model, model_file);

fprintf('Built Simulink model:\n  %s\n', model_file);

function close_open_data_dictionaries()
try
    open_paths = Simulink.data.dictionary.getOpenDictionaryPaths;
catch
    open_paths = {};
end

for i = 1:numel(open_paths)
    try
        dd = Simulink.data.dictionary.open(open_paths{i});
        discardChanges(dd);
        close(dd);
    catch err
        warning('Could not discard and close data dictionary "%s": %s', ...
            open_paths{i}, err.message);
    end
end

try
    Simulink.data.dictionary.closeAll;
catch err
    warning('Could not close all open data dictionaries before rebuild: %s', ...
        err.message);
end
end

function cfg = customer_interface_config()
% Customer-editable interface contract.
%
% Use NumericType for current to demonstrate a fixed-point data dictionary
% contract. The angle stays single for now because fixed-point sin/cos usually
% needs a lookup-table or normalized-angle design.
cfg.modelName = 'motor_clarke_park_model';
cfg.functionName = 'MotorClarkeParkStep';
cfg.sampleTime = '0.001';

cfg.dictionaryName = 'motor_interface.sldd';
cfg.typeHeaderFile = 'motor_types.h';

cfg.currentTypeName = 'T_MotorCurrent';
cfg.angleTypeName = 'T_MotorAngle';
cfg.inputBusName = 'motor_t';
cfg.outputBusName = 'motor_dq_t';

cfg.currentTypeKind = 'fixed';
cfg.currentSignedness = 'Signed';
cfg.currentWordLength = 16;
cfg.currentFractionLength = 12;

cfg.angleBaseType = 'single';
end

function create_motor_data_dictionary(cfg)
dd = open_or_create_data_dictionary(cfg);

design_data = getSection(dd, 'Design Data');

upsert_current_type(design_data, cfg);
upsert_alias_type(design_data, cfg.angleTypeName, ...
    cfg.angleBaseType, cfg.typeHeaderFile);

upsert_bus_type(design_data, cfg.inputBusName, cfg.typeHeaderFile, { ...
    {'ia', cfg.currentTypeName}; ...
    {'ib', cfg.currentTypeName}; ...
    {'ic', cfg.currentTypeName}; ...
    {'theta_e', cfg.angleTypeName}});

upsert_bus_type(design_data, cfg.outputBusName, cfg.typeHeaderFile, { ...
    {'i_alpha', cfg.currentTypeName}; ...
    {'i_beta', cfg.currentTypeName}; ...
    {'id', cfg.currentTypeName}; ...
    {'iq', cfg.currentTypeName}});

saveChanges(dd);
close(dd);
end

function dd = open_or_create_data_dictionary(cfg)
if exist(cfg.dictionaryFile, 'file')
    try
        dd = Simulink.data.dictionary.open(cfg.dictionaryFile);
        getSection(dd, 'Design Data');
        return;
    catch err
        if is_dictionary_already_open_error(err)
            close_open_data_dictionaries();
            dd = Simulink.data.dictionary.open(cfg.dictionaryFile);
            getSection(dd, 'Design Data');
            return;
        end

        warning('Rebuilding data dictionary "%s" because it could not be opened: %s', ...
            cfg.dictionaryFile, err.message);
        close_open_data_dictionaries();
        backup_file = backup_bad_dictionary(cfg.dictionaryFile);
        fprintf('Backed up unusable data dictionary:\n  %s\n', backup_file);
    end
end

try
    dd = Simulink.data.dictionary.create(cfg.dictionaryFile);
catch err
    if ~is_dictionary_already_open_error(err)
        rethrow(err);
    end

    close_open_data_dictionaries();
    dd = Simulink.data.dictionary.create(cfg.dictionaryFile);
end
end

function tf = is_dictionary_already_open_error(err)
msg = err.message;
tf = contains(msg, 'same file name', 'IgnoreCase', true) || ...
    contains(msg, '相同文件名');
end

function backup_file = backup_bad_dictionary(dictionary_file)
[folder, name, ext] = fileparts(dictionary_file);
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
backup_file = fullfile(folder, [name '_bad_' timestamp ext]);
index = 1;

while exist(backup_file, 'file')
    backup_file = fullfile(folder, sprintf('%s_bad_%s_%d%s', ...
        name, timestamp, index, ext));
    index = index + 1;
end

movefile(dictionary_file, backup_file);
end

function upsert_current_type(section, cfg)
switch cfg.currentTypeKind
    case 'fixed'
        upsert_numeric_type(section, cfg.currentTypeName, ...
            cfg.currentSignedness, cfg.currentWordLength, ...
            cfg.currentFractionLength, cfg.typeHeaderFile);
    case 'alias'
        upsert_alias_type(section, cfg.currentTypeName, ...
            cfg.currentBaseType, cfg.typeHeaderFile);
    otherwise
        error('Unsupported currentTypeKind: %s', cfg.currentTypeKind);
end
end

function upsert_numeric_type(section, type_name, signedness, word_length, ...
    fraction_length, header_file)
numeric_type = Simulink.NumericType;
numeric_type.DataTypeMode = 'Fixed-point: binary point scaling';
numeric_type.Signedness = signedness;
numeric_type.WordLength = word_length;
numeric_type.FractionLength = fraction_length;
numeric_type.IsAlias = true;
numeric_type.DataScope = 'Exported';
numeric_type.HeaderFile = header_file;

entry = find(section, 'Name', type_name);
if isempty(entry)
    addEntry(section, type_name, numeric_type);
else
    setValue(entry(1), numeric_type);
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

function add_transform_subsystem(model, cfg)
subsystem = [model '/' cfg.functionName];

add_block('simulink/Ports & Subsystems/Subsystem', subsystem, ...
    'Position', [175 70 315 160]);
delete_line(subsystem, 'In1/1', 'Out1/1');
delete_block([subsystem '/In1']);
delete_block([subsystem '/Out1']);

set_param(subsystem, ...
    'TreatAsAtomicUnit', 'on', ...
    'RTWSystemCode', 'Reusable function', ...
    'RTWFcnNameOpts', 'User specified', ...
    'RTWFcnName', cfg.functionName, ...
    'RTWFileNameOpts', 'User specified', ...
    'RTWFileName', cfg.functionName);

add_block('simulink/Sources/In1', [subsystem '/motor_in'], ...
    'Position', [40 95 70 115], ...
    'OutDataTypeStr', ['Bus: ' cfg.inputBusName], ...
    'BusOutputAsStruct', 'on');

add_block('simulink/Signal Routing/Bus Selector', ...
    [subsystem '/motor_bus_selector'], ...
    'Position', [105 45 120 170], ...
    'OutputSignals', 'ia,ib,ic,theta_e');

add_block('simulink/Math Operations/Gain', [subsystem '/ib_half'], ...
    'Position', [175 70 235 100], ...
    'Gain', '0.5', ...
    'ParamDataTypeStr', cfg.currentTypeName, ...
    'OutDataTypeStr', cfg.currentTypeName, ...
    'RndMeth', 'Floor', ...
    'SaturateOnIntegerOverflow', 'on');

add_block('simulink/Math Operations/Gain', [subsystem '/ic_half'], ...
    'Position', [175 125 235 155], ...
    'Gain', '0.5', ...
    'ParamDataTypeStr', cfg.currentTypeName, ...
    'OutDataTypeStr', cfg.currentTypeName, ...
    'RndMeth', 'Floor', ...
    'SaturateOnIntegerOverflow', 'on');

add_block('simulink/Math Operations/Sum', [subsystem '/alpha_sum'], ...
    'Position', [285 58 315 142], ...
    'Inputs', '+--', ...
    'AccumDataTypeStr', cfg.currentTypeName, ...
    'OutDataTypeStr', cfg.currentTypeName, ...
    'RndMeth', 'Floor', ...
    'SaturateOnIntegerOverflow', 'on');

add_block('simulink/Math Operations/Gain', [subsystem '/alpha_gain'], ...
    'Position', [350 80 430 115], ...
    'Gain', '0.6666666666666666', ...
    'ParamDataTypeStr', cfg.currentTypeName, ...
    'OutDataTypeStr', cfg.currentTypeName, ...
    'RndMeth', 'Floor', ...
    'SaturateOnIntegerOverflow', 'on');

add_block('simulink/Math Operations/Sum', [subsystem '/beta_sum'], ...
    'Position', [285 185 315 230], ...
    'Inputs', '+-', ...
    'AccumDataTypeStr', cfg.currentTypeName, ...
    'OutDataTypeStr', cfg.currentTypeName, ...
    'RndMeth', 'Floor', ...
    'SaturateOnIntegerOverflow', 'on');

add_block('simulink/Math Operations/Gain', [subsystem '/beta_gain'], ...
    'Position', [350 190 430 225], ...
    'Gain', '0.5773502691896258', ...
    'ParamDataTypeStr', cfg.currentTypeName, ...
    'OutDataTypeStr', cfg.currentTypeName, ...
    'RndMeth', 'Floor', ...
    'SaturateOnIntegerOverflow', 'on');

add_block('simulink/Math Operations/Trigonometric Function', ...
    [subsystem '/cos_theta'], ...
    'Position', [285 275 345 305], ...
    'Operator', 'cos');

add_block('simulink/Math Operations/Trigonometric Function', ...
    [subsystem '/sin_theta'], ...
    'Position', [285 330 345 360], ...
    'Operator', 'sin');

add_product_block(subsystem, 'alpha_cos', [485 70 535 105], cfg);
add_product_block(subsystem, 'beta_sin', [485 130 535 165], cfg);
add_product_block(subsystem, 'alpha_sin', [485 220 535 255], cfg);
add_product_block(subsystem, 'beta_cos', [485 280 535 315], cfg);

add_block('simulink/Math Operations/Sum', [subsystem '/id_sum'], ...
    'Position', [585 93 615 142], ...
    'Inputs', '++', ...
    'AccumDataTypeStr', cfg.currentTypeName, ...
    'OutDataTypeStr', cfg.currentTypeName, ...
    'RndMeth', 'Floor', ...
    'SaturateOnIntegerOverflow', 'on');

add_block('simulink/Math Operations/Sum', [subsystem '/iq_sum'], ...
    'Position', [585 243 615 292], ...
    'Inputs', '-+', ...
    'AccumDataTypeStr', cfg.currentTypeName, ...
    'OutDataTypeStr', cfg.currentTypeName, ...
    'RndMeth', 'Floor', ...
    'SaturateOnIntegerOverflow', 'on');

add_block('simulink/Signal Routing/Bus Creator', ...
    [subsystem '/dq_bus_creator'], ...
    'Position', [690 80 705 310], ...
    'Inputs', '4', ...
    'UseBusObject', 'on', ...
    'BusObject', cfg.outputBusName, ...
    'NonVirtualBus', 'on');

add_block('simulink/Sinks/Out1', [subsystem '/dq_out'], ...
    'Position', [780 185 810 205], ...
    'OutDataTypeStr', ['Bus: ' cfg.outputBusName]);

add_line(subsystem, 'motor_in/1', 'motor_bus_selector/1', 'autorouting', 'on');

add_line(subsystem, 'motor_bus_selector/1', 'alpha_sum/1', 'autorouting', 'on');
add_line(subsystem, 'motor_bus_selector/2', 'ib_half/1', 'autorouting', 'on');
add_line(subsystem, 'motor_bus_selector/3', 'ic_half/1', 'autorouting', 'on');
add_line(subsystem, 'ib_half/1', 'alpha_sum/2', 'autorouting', 'on');
add_line(subsystem, 'ic_half/1', 'alpha_sum/3', 'autorouting', 'on');
add_line(subsystem, 'alpha_sum/1', 'alpha_gain/1', 'autorouting', 'on');

add_line(subsystem, 'motor_bus_selector/2', 'beta_sum/1', 'autorouting', 'on');
add_line(subsystem, 'motor_bus_selector/3', 'beta_sum/2', 'autorouting', 'on');
add_line(subsystem, 'beta_sum/1', 'beta_gain/1', 'autorouting', 'on');

add_line(subsystem, 'motor_bus_selector/4', 'cos_theta/1', 'autorouting', 'on');
add_line(subsystem, 'motor_bus_selector/4', 'sin_theta/1', 'autorouting', 'on');

add_line(subsystem, 'alpha_gain/1', 'alpha_cos/1', 'autorouting', 'on');
add_line(subsystem, 'cos_theta/1', 'alpha_cos/2', 'autorouting', 'on');
add_line(subsystem, 'beta_gain/1', 'beta_sin/1', 'autorouting', 'on');
add_line(subsystem, 'sin_theta/1', 'beta_sin/2', 'autorouting', 'on');
add_line(subsystem, 'alpha_cos/1', 'id_sum/1', 'autorouting', 'on');
add_line(subsystem, 'beta_sin/1', 'id_sum/2', 'autorouting', 'on');

add_line(subsystem, 'alpha_gain/1', 'alpha_sin/1', 'autorouting', 'on');
add_line(subsystem, 'sin_theta/1', 'alpha_sin/2', 'autorouting', 'on');
add_line(subsystem, 'beta_gain/1', 'beta_cos/1', 'autorouting', 'on');
add_line(subsystem, 'cos_theta/1', 'beta_cos/2', 'autorouting', 'on');
add_line(subsystem, 'alpha_sin/1', 'iq_sum/1', 'autorouting', 'on');
add_line(subsystem, 'beta_cos/1', 'iq_sum/2', 'autorouting', 'on');

name_line(add_line(subsystem, 'alpha_gain/1', 'dq_bus_creator/1', ...
    'autorouting', 'on'), 'i_alpha');
name_line(add_line(subsystem, 'beta_gain/1', 'dq_bus_creator/2', ...
    'autorouting', 'on'), 'i_beta');
name_line(add_line(subsystem, 'id_sum/1', 'dq_bus_creator/3', ...
    'autorouting', 'on'), 'id');
name_line(add_line(subsystem, 'iq_sum/1', 'dq_bus_creator/4', ...
    'autorouting', 'on'), 'iq');
add_line(subsystem, 'dq_bus_creator/1', 'dq_out/1', 'autorouting', 'on');

end

function add_product_block(subsystem, block_name, position, cfg)
add_block('simulink/Math Operations/Product', ...
    [subsystem '/' block_name], ...
    'Position', position, ...
    'Inputs', '**', ...
    'OutDataTypeStr', cfg.currentTypeName, ...
    'RndMeth', 'Floor', ...
    'SaturateOnIntegerOverflow', 'on');
end

function name_line(line_handle, signal_name)
set_param(line_handle, 'Name', signal_name);
end
