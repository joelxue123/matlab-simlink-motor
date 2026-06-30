%% Create a simple FOC interface dictionary demo
%
% This script is intentionally small for learning:
% 1. Create or open a Simulink Data Dictionary
% 2. Define shared scalar types with Simulink.AliasType
% 3. Define FOC input/output bus objects
% 4. Save the dictionary
%
% Run:
%   run('matlab-practice/examples/create_foc_interface_dictionary_demo.m')

clear;
clc;

cfg.dictionaryName = 'FOC_Interface.sldd';
cfg.typeHeaderFile = 'foc_interface_types.h';

cfg.currentTypeName = 'T_FocCurrent';
cfg.voltageTypeName = 'T_FocVoltage';

cfg.inputBusName = 'foc_input_t';
cfg.outputBusName = 'foc_output_t';

cfg.baseType = 'single';

script_dir = fileparts(mfilename('fullpath'));
cfg.dictionaryFile = fullfile(script_dir, cfg.dictionaryName);

close_open_data_dictionaries();

dd = open_or_create_dictionary(cfg.dictionaryFile);
design_data = getSection(dd, 'Design Data');

upsert_alias_type(design_data, cfg.currentTypeName, cfg.baseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.voltageTypeName, cfg.baseType, cfg.typeHeaderFile);

upsert_bus_type(design_data, cfg.inputBusName, cfg.typeHeaderFile, { ...
    {'Id_ref', cfg.currentTypeName}; ...
    {'Iq_ref', cfg.currentTypeName}; ...
    {'Id_fbk', cfg.currentTypeName}; ...
    {'Iq_fbk', cfg.currentTypeName}});

upsert_bus_type(design_data, cfg.outputBusName, cfg.typeHeaderFile, { ...
    {'Vd_cmd', cfg.voltageTypeName}; ...
    {'Vq_cmd', cfg.voltageTypeName}});

saveChanges(dd);
close(dd);

fprintf('Created dictionary:\n  %s\n', cfg.dictionaryFile);
fprintf('Input bus : %s\n', cfg.inputBusName);
fprintf('Output bus: %s\n', cfg.outputBusName);

function close_open_data_dictionaries()
try
    Simulink.data.dictionary.closeAll('-discard');
catch err
    warning('Could not close open dictionaries: %s', err.message);
end
end

function dd = open_or_create_dictionary(dictionary_file)
if exist(dictionary_file, 'file')
    dd = Simulink.data.dictionary.open(dictionary_file);
else
    dd = Simulink.data.dictionary.create(dictionary_file);
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
