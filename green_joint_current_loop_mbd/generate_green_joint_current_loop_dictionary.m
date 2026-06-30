%% Generate green-joint current-loop interface dictionary from interface.json
%
% R2023b in this environment does not include a built-in YAML decoder, so the
% human-readable interface.yaml is mirrored by interface.json for generation.

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

assert_green_joint_safe_rebuild_environment();
contract = read_interface_contract(fullfile(script_dir, 'interface.json'));
contract.aliases = as_cell(contract.aliases);
contract.buses = as_cell(contract.buses);
contract.parameters = as_cell(contract.parameters);
for i = 1:numel(contract.buses)
    contract.buses{i}.elements = as_cell(contract.buses{i}.elements);
end
dictionary_file = fullfile(script_dir, contract.dictionary);

close_open_data_dictionaries();

if exist(dictionary_file, 'file')
    delete(dictionary_file);
end

dd = Simulink.data.dictionary.create(dictionary_file);
cleanup_dd = onCleanup(@() close(dd));
design_data = getSection(dd, 'Design Data');

validate_contract(contract);

for i = 1:numel(contract.aliases)
    upsert_alias_type(design_data, contract.aliases{i}, contract.type_header);
end

for i = 1:numel(contract.buses)
    upsert_bus_type(design_data, contract.buses{i}, contract.type_header);
end

for i = 1:numel(contract.parameters)
    upsert_parameter(design_data, contract.parameters{i});
end

saveChanges(dd);

assignin('base', 'green_joint_current_loop_contract', contract);

fprintf('Generated dictionary from interface.json:\n  %s\n', dictionary_file);

function contract = read_interface_contract(file_name)
text = fileread(file_name);
contract = jsondecode(text);
end

function values = as_cell(values)
if ~iscell(values)
    values = num2cell(values);
end
end

function validate_contract(contract)
required_top = {'module', 'dictionary', 'model', 'step_function', ...
    'type_header', 'sample_time_s', 'aliases', 'buses', 'parameters'};
for i = 1:numel(required_top)
    require_field(contract, required_top{i}, 'interface contract');
end

type_names = strings(numel(contract.aliases), 1);
for i = 1:numel(contract.aliases)
    alias = contract.aliases{i};
    require_field(alias, 'name', 'alias');
    require_field(alias, 'base_type', alias.name);
    type_names(i) = string(alias.name);
end

for i = 1:numel(contract.buses)
    bus = contract.buses{i};
    require_field(bus, 'name', 'bus');
    require_field(bus, 'elements', bus.name);
    for j = 1:numel(bus.elements)
        element = bus.elements{j};
        require_field(element, 'name', bus.name);
        require_field(element, 'type', element.name);
        if ~any(type_names == string(element.type))
            error('Bus element %s.%s references unknown type %s.', ...
                bus.name, element.name, element.type);
        end
    end
end

for i = 1:numel(contract.parameters)
    parameter = contract.parameters{i};
    require_field(parameter, 'name', 'parameter');
    require_field(parameter, 'type', parameter.name);
    require_field(parameter, 'value', parameter.name);
    if ~any(type_names == string(parameter.type))
        error('Parameter %s references unknown type %s.', ...
            parameter.name, parameter.type);
    end
end
end

function require_field(value, field_name, owner)
if ~isfield(value, field_name)
    error('Missing required field "%s" in %s.', field_name, owner);
end
end

function close_open_data_dictionaries()
try
    Simulink.data.dictionary.closeAll('-discard');
catch err
    warning('Could not close open dictionaries: %s', err.message);
end
end

function upsert_alias_type(section, spec, header_file)
alias_type = Simulink.AliasType(spec.base_type);
alias_type.DataScope = get_or_default(spec, 'data_scope', 'Exported');
alias_type.HeaderFile = header_file;
if isfield(spec, 'description')
    alias_type.Description = spec.description;
end

entry = find(section, 'Name', spec.name);
if isempty(entry)
    addEntry(section, spec.name, alias_type);
else
    setValue(entry(1), alias_type);
end
end

function upsert_bus_type(section, spec, header_file)
bus_elements = repmat(Simulink.BusElement, numel(spec.elements), 1);
for i = 1:numel(spec.elements)
    element = spec.elements{i};
    bus_elements(i).Name = element.name;
    bus_elements(i).DataType = element.type;
    bus_elements(i).Dimensions = 1;
    if isfield(element, 'unit')
        bus_elements(i).Unit = element.unit;
    end
    if isfield(element, 'description')
        bus_elements(i).Description = element.description;
    end
    if isfield(element, 'min')
        bus_elements(i).Min = element.min;
    end
    if isfield(element, 'max')
        bus_elements(i).Max = element.max;
    end
end

bus_type = Simulink.Bus;
bus_type.Elements = bus_elements;
bus_type.DataScope = get_or_default(spec, 'data_scope', 'Exported');
bus_type.HeaderFile = header_file;

entry = find(section, 'Name', spec.name);
if isempty(entry)
    addEntry(section, spec.name, bus_type);
else
    setValue(entry(1), bus_type);
end
end

function upsert_parameter(section, spec)
parameter = Simulink.Parameter(spec.value);
parameter.DataType = spec.type;
parameter.CoderInfo.StorageClass = get_or_default(spec, 'storage', 'Auto');
if isfield(spec, 'unit')
    parameter.Unit = spec.unit;
end
if isfield(spec, 'description')
    parameter.Description = spec.description;
end
if isfield(spec, 'min')
    parameter.Min = spec.min;
end
if isfield(spec, 'max')
    parameter.Max = spec.max;
end

entry = find(section, 'Name', spec.name);
if isempty(entry)
    addEntry(section, spec.name, parameter);
else
    setValue(entry(1), parameter);
end
end

function value = get_or_default(spec, field_name, default_value)
if isfield(spec, field_name)
    value = spec.(field_name);
else
    value = default_value;
end
end
