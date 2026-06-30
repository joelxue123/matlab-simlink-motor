%% Build a float-based speed PI MBD module
%
% Architecture:
%   speed_pi_input_t
%     -> SpeedPiStep
%     -> speed_pi_output_t
%
% The output is an iq_ref command for the inner current loop.

clc;

cfg = customer_interface_config();
script_dir = fileparts(mfilename('fullpath'));
cfg.dictionaryFile = fullfile(script_dir, cfg.dictionaryName);
assignin('base', 'speed_pi_config', cfg);

previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

defaults = default_speed_pi_params();
assignin('base', 'speed_pi_defaults', defaults);
assignin('base', 'speed_pi_simcfg', defaults.simcfg);
assignin('base', 'speed_pi_test', defaults.test);

model = cfg.modelName;
model_file = fullfile(script_dir, [model '.slx']);

if bdIsLoaded(model)
    close_system(model, 0);
end

close_open_data_dictionaries();
create_speed_pi_data_dictionary(cfg, defaults);

if exist(model_file, 'file')
    delete(model_file);
end

new_system(model);
set_param(model, 'DataDictionary', cfg.dictionaryName);
set_param(model, ...
    'StopTime', '0.001', ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', num2str(defaults.simcfg.Ts_speed, '%.12g'), ...
    'SystemTargetFile', 'ert.tlc', ...
    'ProdHWDeviceType', 'ARM Compatible->ARM Cortex', ...
    'TargetHWDeviceType', 'ARM Compatible->ARM Cortex', ...
    'GenerateReport', 'on', ...
    'RTWVerbose', 'on', ...
    'GenCodeOnly', 'on', ...
    'GenerateSampleERTMain', 'off', ...
    'SupportContinuousTime', 'off', ...
    'ParameterPrecisionLossMsg', 'none', ...
    'InlineParams', 'off');

set_param(model, ...
    'CodeInterfacePackaging', 'Reusable function', ...
    'ModelReferenceNumInstancesAllowed', 'Multi');

add_speed_pi_subsystem(model, cfg);
add_speed_pi_root_interface(model, cfg);

set_param(model, 'SimulationCommand', 'update');
save_system(model, model_file);

fprintf('Built speed PI MBD model:\n  %s\n', model_file);

function cfg = customer_interface_config()
% Customer-editable interface and architecture contract.
cfg.modelName = 'speed_pi_model';
cfg.sampleTime = '100e-6';

cfg.dictionaryName = 'speed_pi_interface.sldd';
cfg.typeHeaderFile = 'speed_pi_types.h';
cfg.rebuildDictionary = false;
cfg.preserveExistingParameterValues = true;

cfg.realTypeName = 'T_SpeedPiFloat';
cfg.speedTypeName = 'T_SpeedPiSpeed';
cfg.currentTypeName = 'T_SpeedPiCurrent';
cfg.gainTypeName = 'T_SpeedPiGain';

cfg.inputBusName = 'speed_pi_input_t';
cfg.outputBusName = 'speed_pi_output_t';

cfg.floatBaseType = 'single';
end

function defaults = default_speed_pi_params()
% Baseline numbers are derived from average-inverter/motor_control_params.m.
defaults.simcfg.stop_time = 0.250;
% green-joint calls the speed loop every 2 current-loop ISR ticks:
% 20 kHz current loop -> 100 us speed-loop period.
defaults.simcfg.Ts_speed = 100e-6;

motor.pole_pairs = 10;
motor.back_emf_vrms_per_krpm = 17.03;
motor.J = 2.5e-4;
motor.B = 1.0e-4;
motor.psi_f = motor.back_emf_vrms_per_krpm / ...
    (sqrt(3/2) * motor.pole_pairs * (1000 * 2 * pi / 60));
motor.torque_constant = 1.5 * motor.pole_pairs * motor.psi_f;

current_bandwidth_hz = 800;
tau_current_cl = 1 / (2 * pi * current_bandwidth_hz);
tau_speed_delay = 1.5 * defaults.simcfg.Ts_speed;
tau_sigma = tau_current_cl + tau_speed_delay;
speed_bandwidth_hz = min(1 / (2 * pi * 3 * tau_sigma), 40);
speed_bandwidth_rad_s = 2 * pi * speed_bandwidth_hz;
speed_damping = 1.0;

defaults.Kp_speed = single( ...
    2 * speed_damping * speed_bandwidth_rad_s * motor.J / ...
    motor.torque_constant);
defaults.Ki_speed = single( ...
    speed_bandwidth_rad_s^2 * motor.J / motor.torque_constant);
defaults.Kaw_speed = single(40);
defaults.IqLimitDefault = single(15);

defaults.test.wm_ref = single(400 * 2 * pi / 60);
defaults.test.wm_initial = single(0);
defaults.test.iq_limit = defaults.IqLimitDefault;
defaults.test.load_torque = single(0);
defaults.test.torque_constant = single(motor.torque_constant);
defaults.test.J = single(motor.J);
defaults.test.B = single(motor.B);
end

function close_open_data_dictionaries()
try
    Simulink.data.dictionary.closeAll('-discard');
catch err
    warning('Could not close open data dictionaries before rebuild: %s', ...
        err.message);
end
end

function create_speed_pi_data_dictionary(cfg, defaults)
dd = open_or_create_data_dictionary(cfg);
design_data = getSection(dd, 'Design Data');

upsert_alias_type(design_data, cfg.realTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.speedTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.currentTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.gainTypeName, cfg.floatBaseType, cfg.typeHeaderFile);

upsert_bus_type(design_data, cfg.inputBusName, cfg.typeHeaderFile, { ...
    {'wm_ref', cfg.speedTypeName}; ...
    {'wm_meas', cfg.speedTypeName}; ...
    {'iq_limit', cfg.currentTypeName}});

upsert_bus_type(design_data, cfg.outputBusName, cfg.typeHeaderFile, { ...
    {'iq_ref', cfg.currentTypeName}});

upsert_parameter(design_data, 'Kp_speed', defaults.Kp_speed, cfg.gainTypeName, cfg);
upsert_parameter(design_data, 'Ki_speed', defaults.Ki_speed, cfg.gainTypeName, cfg);
upsert_parameter(design_data, 'Kaw_speed', defaults.Kaw_speed, cfg.gainTypeName, cfg);
upsert_parameter(design_data, 'IqLimitDefault', defaults.IqLimitDefault, cfg.currentTypeName, cfg);

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

function upsert_parameter(section, name, value, data_type, cfg)
parameter = Simulink.Parameter(double(value));
parameter.DataType = data_type;
parameter.CoderInfo.StorageClass = 'Auto';

entry = find(section, 'Name', name);
if isempty(entry)
    addEntry(section, name, parameter);
else
    if isfield(cfg, 'preserveExistingParameterValues') && ...
            cfg.preserveExistingParameterValues
        existing_parameter = getValue(entry(1));
        if isa(existing_parameter, 'Simulink.Parameter')
            parameter.Value = existing_parameter.Value;
        end
    end
    setValue(entry(1), parameter);
end
end

function add_speed_pi_test_sources(model, cfg)
add_typed_constant(model, 'wm_ref', [45 45 115 70], ...
    'speed_pi_test.wm_ref', cfg.speedTypeName, cfg.sampleTime);
add_typed_constant(model, 'iq_limit', [45 165 115 190], ...
    'speed_pi_test.iq_limit', cfg.currentTypeName, cfg.sampleTime);

add_block('simulink/Discrete/Unit Delay', [model '/wm_meas_state'], ...
    'Position', [705 235 775 265], ...
    'InitialCondition', 'speed_pi_test.wm_initial', ...
    'SampleTime', cfg.sampleTime);

add_block('simulink/Signal Routing/Bus Creator', [model '/speed_input_bus_creator'], ...
    'Position', [180 45 195 190], ...
    'Inputs', '3', ...
    'UseBusObject', 'on', ...
    'BusObject', cfg.inputBusName, ...
    'NonVirtualBus', 'on');

name_line(add_line(model, 'wm_ref/1', 'speed_input_bus_creator/1', ...
    'autorouting', 'on'), 'wm_ref');
name_line(add_line(model, 'wm_meas_state/1', 'speed_input_bus_creator/2', ...
    'autorouting', 'on'), 'wm_meas');
name_line(add_line(model, 'iq_limit/1', 'speed_input_bus_creator/3', ...
    'autorouting', 'on'), 'iq_limit');
end

function add_speed_pi_subsystem(model, cfg)
subsystem = [model '/SpeedPiStep'];
add_clean_subsystem(subsystem, [265 90 460 185]);

set_param(subsystem, ...
    'TreatAsAtomicUnit', 'on', ...
    'RTWSystemCode', 'Reusable function', ...
    'RTWFcnNameOpts', 'User specified', ...
    'RTWFcnName', 'SpeedPiStep', ...
    'RTWFileNameOpts', 'User specified', ...
    'RTWFileName', 'SpeedPiStep');

add_block('simulink/Sources/In1', [subsystem '/speed_in'], ...
    'Position', [35 115 65 135], ...
    'OutDataTypeStr', ['Bus: ' cfg.inputBusName], ...
    'BusOutputAsStruct', 'on');

add_block('simulink/Signal Routing/Bus Selector', [subsystem '/speed_input_selector'], ...
    'Position', [105 65 120 190], ...
    'OutputSignals', 'wm_ref,wm_meas,iq_limit');

add_block('simulink/Math Operations/Sum', [subsystem '/speed_error'], ...
    'Position', [170 75 200 110], ...
    'Inputs', '+-', ...
    'AccumDataTypeStr', cfg.speedTypeName, ...
    'OutDataTypeStr', cfg.speedTypeName);
add_block('simulink/Math Operations/Gain', [subsystem '/speed_kp'], ...
    'Position', [240 75 300 105], ...
    'Gain', 'Kp_speed', ...
    'ParamDataTypeStr', cfg.gainTypeName, ...
    'OutDataTypeStr', cfg.currentTypeName);
add_block('simulink/Math Operations/Gain', [subsystem '/speed_ki'], ...
    'Position', [240 130 300 160], ...
    'Gain', 'Ki_speed', ...
    'ParamDataTypeStr', cfg.gainTypeName, ...
    'OutDataTypeStr', cfg.currentTypeName);

add_block('simulink/Discrete/Unit Delay', [subsystem '/speed_integrator_state'], ...
    'Position', [365 170 430 200], ...
    'InitialCondition', '0', ...
    'SampleTime', cfg.sampleTime);

add_block('simulink/Math Operations/Sum', [subsystem '/speed_pre_sat_sum'], ...
    'Position', [470 90 500 135], ...
    'Inputs', '++', ...
    'AccumDataTypeStr', cfg.currentTypeName, ...
    'OutDataTypeStr', cfg.currentTypeName);
add_minmax(subsystem, 'iq_min_upper', [545 80 605 110], 'min');
add_block('simulink/Math Operations/Gain', [subsystem '/neg_iq_limit'], ...
    'Position', [545 160 605 190], ...
    'Gain', '-1', ...
    'ParamDataTypeStr', cfg.realTypeName, ...
    'OutDataTypeStr', cfg.currentTypeName);
add_minmax(subsystem, 'iq_max_lower', [650 85 710 115], 'max');

add_block('simulink/Math Operations/Sum', [subsystem '/speed_back_calc_error'], ...
    'Position', [745 130 775 175], ...
    'Inputs', '+-', ...
    'AccumDataTypeStr', cfg.currentTypeName, ...
    'OutDataTypeStr', cfg.currentTypeName);
add_block('simulink/Math Operations/Gain', [subsystem '/speed_kaw'], ...
    'Position', [815 135 875 165], ...
    'Gain', 'Kaw_speed', ...
    'ParamDataTypeStr', cfg.gainTypeName, ...
    'OutDataTypeStr', cfg.currentTypeName);

add_block('simulink/Math Operations/Sum', [subsystem '/speed_integrator_rate'], ...
    'Position', [915 135 945 180], ...
    'Inputs', '++', ...
    'AccumDataTypeStr', cfg.currentTypeName, ...
    'OutDataTypeStr', cfg.currentTypeName);
add_block('simulink/Math Operations/Gain', [subsystem '/speed_integrator_delta'], ...
    'Position', [980 140 1040 170], ...
    'Gain', cfg.sampleTime, ...
    'ParamDataTypeStr', cfg.realTypeName, ...
    'OutDataTypeStr', cfg.currentTypeName);
add_block('simulink/Math Operations/Sum', [subsystem '/speed_integrator_next'], ...
    'Position', [1080 155 1110 200], ...
    'Inputs', '++', ...
    'AccumDataTypeStr', cfg.currentTypeName, ...
    'OutDataTypeStr', cfg.currentTypeName);

add_block('simulink/Signal Routing/Bus Creator', [subsystem '/speed_output_bus_creator'], ...
    'Position', [780 55 795 100], ...
    'Inputs', '1', ...
    'UseBusObject', 'on', ...
    'BusObject', cfg.outputBusName, ...
    'NonVirtualBus', 'on');
add_block('simulink/Sinks/Out1', [subsystem '/speed_out'], ...
    'Position', [865 65 895 85], ...
    'OutDataTypeStr', ['Bus: ' cfg.outputBusName]);

add_line(subsystem, 'speed_in/1', 'speed_input_selector/1', 'autorouting', 'on');
add_line(subsystem, 'speed_input_selector/1', 'speed_error/1', 'autorouting', 'on');
add_line(subsystem, 'speed_input_selector/2', 'speed_error/2', 'autorouting', 'on');
add_line(subsystem, 'speed_error/1', 'speed_kp/1', 'autorouting', 'on');
add_line(subsystem, 'speed_error/1', 'speed_ki/1', 'autorouting', 'on');
add_line(subsystem, 'speed_kp/1', 'speed_pre_sat_sum/1', 'autorouting', 'on');
add_line(subsystem, 'speed_integrator_state/1', 'speed_pre_sat_sum/2', 'autorouting', 'on');

add_line(subsystem, 'speed_pre_sat_sum/1', 'iq_min_upper/1', 'autorouting', 'on');
add_line(subsystem, 'speed_input_selector/3', 'iq_min_upper/2', 'autorouting', 'on');
add_line(subsystem, 'speed_input_selector/3', 'neg_iq_limit/1', 'autorouting', 'on');
add_line(subsystem, 'iq_min_upper/1', 'iq_max_lower/1', 'autorouting', 'on');
add_line(subsystem, 'neg_iq_limit/1', 'iq_max_lower/2', 'autorouting', 'on');

name_line(add_line(subsystem, 'iq_max_lower/1', ...
    'speed_output_bus_creator/1', 'autorouting', 'on'), 'iq_ref');
add_line(subsystem, 'speed_output_bus_creator/1', 'speed_out/1', 'autorouting', 'on');

add_line(subsystem, 'iq_max_lower/1', 'speed_back_calc_error/1', 'autorouting', 'on');
add_line(subsystem, 'speed_pre_sat_sum/1', 'speed_back_calc_error/2', 'autorouting', 'on');
add_line(subsystem, 'speed_back_calc_error/1', 'speed_kaw/1', 'autorouting', 'on');
add_line(subsystem, 'speed_ki/1', 'speed_integrator_rate/1', 'autorouting', 'on');
add_line(subsystem, 'speed_kaw/1', 'speed_integrator_rate/2', 'autorouting', 'on');
add_line(subsystem, 'speed_integrator_rate/1', 'speed_integrator_delta/1', 'autorouting', 'on');
add_line(subsystem, 'speed_integrator_state/1', 'speed_integrator_next/1', 'autorouting', 'on');
add_line(subsystem, 'speed_integrator_delta/1', 'speed_integrator_next/2', 'autorouting', 'on');
add_line(subsystem, 'speed_integrator_next/1', 'speed_integrator_state/1', 'autorouting', 'on');
end

function add_speed_pi_root_interface(model, cfg)
add_block('simulink/Sources/In1', [model '/speed_in'], ...
    'Position', [55 120 85 140], ...
    'OutDataTypeStr', ['Bus: ' cfg.inputBusName], ...
    'BusOutputAsStruct', 'on');
add_block('simulink/Sinks/Out1', [model '/speed_out'], ...
    'Position', [520 120 550 140], ...
    'OutDataTypeStr', ['Bus: ' cfg.outputBusName]);

add_line(model, 'speed_in/1', 'SpeedPiStep/1', 'autorouting', 'on');
add_line(model, 'SpeedPiStep/1', 'speed_out/1', 'autorouting', 'on');
end

function add_speed_plant_harness(model, cfg)
add_block('simulink/Signal Routing/Bus Selector', [model '/speed_output_selector'], ...
    'Position', [510 95 525 170], ...
    'OutputSignals', 'iq_ref');
add_line(model, 'SpeedPiStep/1', 'speed_output_selector/1', 'autorouting', 'on');

add_block('simulink/Math Operations/Gain', [model '/iq_to_torque'], ...
    'Position', [570 100 650 130], ...
    'Gain', 'speed_pi_test.torque_constant', ...
    'ParamDataTypeStr', cfg.gainTypeName, ...
    'OutDataTypeStr', cfg.realTypeName);
add_typed_constant(model, 'load_torque', [570 150 650 175], ...
    'speed_pi_test.load_torque', cfg.realTypeName, cfg.sampleTime);
add_block('simulink/Math Operations/Gain', [model '/viscous_torque'], ...
    'Position', [570 235 650 265], ...
    'Gain', 'speed_pi_test.B', ...
    'ParamDataTypeStr', cfg.gainTypeName, ...
    'OutDataTypeStr', cfg.realTypeName);
add_block('simulink/Math Operations/Sum', [model '/net_torque'], ...
    'Position', [700 120 730 185], ...
    'Inputs', '+--', ...
    'AccumDataTypeStr', cfg.realTypeName, ...
    'OutDataTypeStr', cfg.realTypeName);
add_block('simulink/Math Operations/Gain', [model '/inv_inertia'], ...
    'Position', [770 135 850 165], ...
    'Gain', '1/speed_pi_test.J', ...
    'ParamDataTypeStr', cfg.gainTypeName, ...
    'OutDataTypeStr', cfg.speedTypeName);
add_block('simulink/Math Operations/Gain', [model '/speed_delta'], ...
    'Position', [885 135 965 165], ...
    'Gain', 'speed_pi_simcfg.Ts_speed', ...
    'ParamDataTypeStr', cfg.realTypeName, ...
    'OutDataTypeStr', cfg.speedTypeName);
add_block('simulink/Math Operations/Sum', [model '/speed_next'], ...
    'Position', [1000 175 1030 225], ...
    'Inputs', '++', ...
    'AccumDataTypeStr', cfg.speedTypeName, ...
    'OutDataTypeStr', cfg.speedTypeName);

add_line(model, 'speed_output_selector/1', 'iq_to_torque/1', 'autorouting', 'on');
add_line(model, 'iq_to_torque/1', 'net_torque/1', 'autorouting', 'on');
add_line(model, 'load_torque/1', 'net_torque/2', 'autorouting', 'on');
add_line(model, 'wm_meas_state/1', 'viscous_torque/1', 'autorouting', 'on');
add_line(model, 'viscous_torque/1', 'net_torque/3', 'autorouting', 'on');
add_line(model, 'net_torque/1', 'inv_inertia/1', 'autorouting', 'on');
add_line(model, 'inv_inertia/1', 'speed_delta/1', 'autorouting', 'on');
add_line(model, 'speed_delta/1', 'speed_next/1', 'autorouting', 'on');
add_line(model, 'wm_meas_state/1', 'speed_next/2', 'autorouting', 'on');
add_line(model, 'speed_next/1', 'wm_meas_state/1', 'autorouting', 'on');
end

function add_output_logs(model)
add_scalar_logger(model, 'log_wm_ref', [235 35 315 65], 'wm_ref/1');
add_scalar_logger(model, 'log_wm_meas', [805 235 895 265], 'wm_meas_state/1');
add_scalar_logger(model, 'log_iq_ref', [570 60 650 90], 'speed_output_selector/1');

add_block('simulink/Signal Routing/Mux', [model '/speed_scope_mux'], ...
    'Position', [930 60 935 130], ...
    'Inputs', '3');
add_block('simulink/Sinks/Scope', [model '/Speed PI Scope'], ...
    'Position', [975 70 1045 130], ...
    'NumInputPorts', '1');
add_line(model, 'wm_ref/1', 'speed_scope_mux/1', 'autorouting', 'on');
add_line(model, 'wm_meas_state/1', 'speed_scope_mux/2', 'autorouting', 'on');
add_line(model, 'speed_output_selector/1', 'speed_scope_mux/3', 'autorouting', 'on');
add_line(model, 'speed_scope_mux/1', 'Speed PI Scope/1', 'autorouting', 'on');
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

function add_typed_constant(model, block_name, position, value, data_type, sample_time)
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
