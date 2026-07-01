%% Build green-joint MIT impedance MBD model
%
% Scope:
%   output-side position/velocity/torque command
%     -> physical impedance
%     -> motor-side iq_ref command
%
% This module is stateless. Current loop, PWM, estimator, and state machine
% remain outside this generated core.

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

cfg = mit_interface_config();
defaults = default_mit_params();
assignin('base', 'gj_mit_config', cfg);
assignin('base', 'gj_mit_defaults', defaults);
assignin('base', 'gj_mit_simcfg', defaults.simcfg);
assignin('base', 'gj_mit_test', defaults.test);

close_open_data_dictionaries();
create_mit_data_dictionary(cfg, defaults);

model = cfg.modelName;
model_file = fullfile(script_dir, [model '.slx']);

if bdIsLoaded(model)
    close_system(model, 0);
end

if exist(model_file, 'file')
    delete(model_file);
end

new_system(model);
set_param(model, ...
    'DataDictionary', cfg.dictionaryName, ...
    'StopTime', '0.001', ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', num2str(defaults.simcfg.Ts_mit, '%.12g'), ...
    'SystemTargetFile', 'ert.tlc', ...
    'ProdHWDeviceType', 'ARM Compatible->ARM Cortex', ...
    'TargetHWDeviceType', 'ARM Compatible->ARM Cortex', ...
    'GenerateReport', 'on', ...
    'RTWVerbose', 'on', ...
    'GenCodeOnly', 'on', ...
    'GenerateSampleERTMain', 'off', ...
    'SupportContinuousTime', 'off', ...
    'ParameterPrecisionLossMsg', 'none', ...
    'InlineParams', 'off', ...
    'CodeInterfacePackaging', 'Reusable function', ...
    'ModelReferenceNumInstancesAllowed', 'Multi');

add_algorithm_boundary(model, cfg);
if ~strcmp(getenv('GJ_MIT_SKIP_UPDATE'), '1')
    set_param(model, 'SimulationCommand', 'update');
end
save_system(model, model_file);

fprintf('Built green-joint MIT impedance MBD model:\n  %s\n', model_file);

function cfg = mit_interface_config()
cfg.modelName = 'green_joint_mit_impedance_model';
cfg.stepFunction = 'GreenJointMitImpedanceStep';
cfg.sampleTime = '50e-6';

cfg.dictionaryName = 'green_joint_mit_impedance_interface.sldd';
cfg.dictionaryFile = fullfile(fileparts(mfilename('fullpath')), cfg.dictionaryName);
cfg.typeHeaderFile = 'green_joint_mit_impedance_types.h';
cfg.rebuildDictionary = false;
cfg.preserveExistingParameterValues = true;

cfg.floatTypeName = 'T_GJMitFloat';
cfg.angleTypeName = 'T_GJMitAngle';
cfg.speedTypeName = 'T_GJMitSpeed';
cfg.torqueTypeName = 'T_GJMitTorque';
cfg.currentTypeName = 'T_GJMitCurrent';
cfg.gainTypeName = 'T_GJMitGain';

cfg.inputBusName = 'green_joint_mit_input_t';
cfg.outputBusName = 'green_joint_mit_output_t';
cfg.floatBaseType = 'single';
end

function defaults = default_mit_params()
defaults.simcfg.Ts_mit = 50e-6;
defaults.simcfg.stop_time = 0.020;

gear_ratio = 183.35;
kt_motor = 0.00517276217;
kt_output = kt_motor * gear_ratio;

j_output = 0.00132792306138;
b_output = 0.0109757550501;
bandwidth_hz = 15;
zeta = 1.0;
wn = 2 * pi * bandwidth_hz;

defaults.GJMitPi = single(pi);
defaults.GJMitTwoPi = single(2 * pi);
defaults.KpDefaultNmPerRad = single(j_output * wn^2);
defaults.KdDefaultNmSPerRad = single(2 * zeta * j_output * wn - b_output);
defaults.TorqueToIqGainDefault = single(1 / kt_output);
defaults.IqLimitDefaultA = single(4.0);

defaults.test.pos_target_rad = single(0.2);
defaults.test.pos_feedback_rad = single(0.0);
defaults.test.vel_target_rad_s = single(0.0);
defaults.test.vel_feedback_rad_s = single(0.0);
defaults.test.ff_torque_nm = single(0.0);
defaults.test.kp_nm_per_rad = defaults.KpDefaultNmPerRad;
defaults.test.kd_nm_s_per_rad = defaults.KdDefaultNmSPerRad;
defaults.test.torque_to_iq_gain_a_per_nm = defaults.TorqueToIqGainDefault;
defaults.test.iq_limit_a = defaults.IqLimitDefaultA;
defaults.test.expected_iq_a = single( ...
    defaults.test.pos_target_rad * defaults.test.kp_nm_per_rad * ...
    defaults.test.torque_to_iq_gain_a_per_nm);
end

function close_open_data_dictionaries()
try
    Simulink.data.dictionary.closeAll('-discard');
catch err
    warning('Could not close open data dictionaries before rebuild: %s', ...
        err.message);
end
end

function create_mit_data_dictionary(cfg, defaults)
dd = open_or_create_data_dictionary(cfg);
design_data = getSection(dd, 'Design Data');

upsert_alias_type(design_data, cfg.floatTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.angleTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.speedTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.torqueTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.currentTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.gainTypeName, cfg.floatBaseType, cfg.typeHeaderFile);

upsert_bus_type(design_data, cfg.inputBusName, cfg.typeHeaderFile, { ...
    {'pos_target_rad', cfg.angleTypeName}; ...
    {'pos_feedback_rad', cfg.angleTypeName}; ...
    {'vel_target_rad_s', cfg.speedTypeName}; ...
    {'vel_feedback_rad_s', cfg.speedTypeName}; ...
    {'ff_torque_nm', cfg.torqueTypeName}; ...
    {'kp_nm_per_rad', cfg.gainTypeName}; ...
    {'kd_nm_s_per_rad', cfg.gainTypeName}; ...
    {'torque_to_iq_gain_a_per_nm', cfg.gainTypeName}; ...
    {'iq_limit_a', cfg.currentTypeName}});

upsert_bus_type(design_data, cfg.outputBusName, cfg.typeHeaderFile, { ...
    {'iq_ref_a', cfg.currentTypeName}});

upsert_parameter(design_data, 'GJMitPi', defaults.GJMitPi, cfg.angleTypeName, cfg);
upsert_parameter(design_data, 'GJMitTwoPi', defaults.GJMitTwoPi, cfg.angleTypeName, cfg);
upsert_parameter(design_data, 'KpDefaultNmPerRad', defaults.KpDefaultNmPerRad, cfg.gainTypeName, cfg);
upsert_parameter(design_data, 'KdDefaultNmSPerRad', defaults.KdDefaultNmSPerRad, cfg.gainTypeName, cfg);
upsert_parameter(design_data, 'TorqueToIqGainDefault', defaults.TorqueToIqGainDefault, cfg.gainTypeName, cfg);
upsert_parameter(design_data, 'IqLimitDefaultA', defaults.IqLimitDefaultA, cfg.currentTypeName, cfg);

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

function add_algorithm_boundary(model, cfg)
subsystem = [model '/' cfg.stepFunction];
add_clean_subsystem(subsystem, [235 90 475 185]);

add_block('simulink/Sources/In1', [subsystem '/mit_in'], ...
    'Position', [35 205 65 225], ...
    'OutDataTypeStr', ['Bus: ' cfg.inputBusName], ...
    'BusOutputAsStruct', 'on');
add_block('simulink/Sinks/Out1', [subsystem '/mit_out'], ...
    'Position', [1185 205 1215 225], ...
    'OutDataTypeStr', ['Bus: ' cfg.outputBusName]);

set_param(subsystem, ...
    'TreatAsAtomicUnit', 'on', ...
    'RTWSystemCode', 'Reusable function', ...
    'RTWFcnNameOpts', 'User specified', ...
    'RTWFcnName', cfg.stepFunction, ...
    'RTWFileNameOpts', 'User specified', ...
    'RTWFileName', cfg.stepFunction);

add_block('simulink/Signal Routing/Bus Selector', [subsystem '/input_selector'], ...
    'Position', [105 75 125 335], ...
    'OutputSignals', ['pos_target_rad,pos_feedback_rad,vel_target_rad_s,' ...
        'vel_feedback_rad_s,ff_torque_nm,kp_nm_per_rad,kd_nm_s_per_rad,' ...
        'torque_to_iq_gain_a_per_nm,iq_limit_a']);
add_line(subsystem, 'mit_in/1', 'input_selector/1', 'autorouting', 'on');

add_mit_algorithm(subsystem, cfg);

add_block('simulink/Signal Routing/Bus Creator', [subsystem '/mit_output_bus_creator'], ...
    'Position', [1095 255 1110 300], ...
    'Inputs', '1', ...
    'UseBusObject', 'on', ...
    'BusObject', cfg.outputBusName, ...
    'NonVirtualBus', 'on');

name_line(add_line(subsystem, 'iq_ref_a/1', 'mit_output_bus_creator/1', 'autorouting', 'on'), 'iq_ref_a');
add_line(subsystem, 'mit_output_bus_creator/1', 'mit_out/1', 'autorouting', 'on');

add_block('simulink/Sources/In1', [model '/mit_in'], ...
    'Position', [55 120 85 140], ...
    'OutDataTypeStr', ['Bus: ' cfg.inputBusName], ...
    'BusOutputAsStruct', 'on');
add_block('simulink/Sinks/Out1', [model '/mit_out'], ...
    'Position', [570 120 600 140], ...
    'OutDataTypeStr', ['Bus: ' cfg.outputBusName]);
add_line(model, 'mit_in/1', [cfg.stepFunction '/1'], 'autorouting', 'on');
add_line(model, [cfg.stepFunction '/1'], 'mit_out/1', 'autorouting', 'on');
end

function add_mit_algorithm(subsystem, cfg)
add_position_wrap(subsystem, cfg);
add_speed_error(subsystem, cfg);
add_torque_command(subsystem, cfg);
add_iq_limit(subsystem, cfg);
end

function add_position_wrap(subsystem, cfg)
add_block('simulink/Math Operations/Sum', [subsystem '/position_error_raw'], ...
    'Position', [180 65 210 100], ...
    'Inputs', '+-', ...
    'AccumDataTypeStr', cfg.angleTypeName, ...
    'OutDataTypeStr', cfg.angleTypeName);
add_typed_constant(subsystem, 'two_pi', [180 125 240 150], ...
    'GJMitTwoPi', cfg.angleTypeName, '-1');
add_typed_constant(subsystem, 'neg_pi', [380 190 440 215], ...
    '-GJMitPi', cfg.angleTypeName, '-1');

add_block('simulink/Math Operations/Sum', [subsystem '/position_error_minus_2pi'], ...
    'Position', [280 50 310 95], ...
    'Inputs', '+-', ...
    'AccumDataTypeStr', cfg.angleTypeName, ...
    'OutDataTypeStr', cfg.angleTypeName);
add_block('simulink/Math Operations/Sum', [subsystem '/position_error_plus_2pi'], ...
    'Position', [280 115 310 160], ...
    'Inputs', '++', ...
    'AccumDataTypeStr', cfg.angleTypeName, ...
    'OutDataTypeStr', cfg.angleTypeName);

add_block('simulink/Signal Routing/Switch', [subsystem '/wrap_high_switch'], ...
    'Position', [370 60 420 110], ...
    'Criteria', 'u2 > Threshold', ...
    'Threshold', 'GJMitPi', ...
    'OutDataTypeStr', cfg.angleTypeName);
add_block('simulink/Logic and Bit Operations/Relational Operator', ...
    [subsystem '/is_below_neg_pi'], ...
    'Position', [470 155 520 185], ...
    'Operator', '<');
add_block('simulink/Signal Routing/Switch', [subsystem '/wrap_low_switch'], ...
    'Position', [570 75 620 125], ...
    'Criteria', 'u2 ~= 0', ...
    'OutDataTypeStr', cfg.angleTypeName);

add_block('simulink/Signal Routing/Goto', [subsystem '/position_error_rad'], ...
    'Position', [665 88 750 112], ...
    'GotoTag', 'position_error_rad');
add_block('simulink/Signal Routing/From', [subsystem '/position_error_rad_from'], ...
    'Position', [765 140 850 164], ...
    'GotoTag', 'position_error_rad');

add_line(subsystem, 'input_selector/1', 'position_error_raw/1', 'autorouting', 'on');
add_line(subsystem, 'input_selector/2', 'position_error_raw/2', 'autorouting', 'on');
add_line(subsystem, 'position_error_raw/1', 'position_error_minus_2pi/1', 'autorouting', 'on');
add_line(subsystem, 'two_pi/1', 'position_error_minus_2pi/2', 'autorouting', 'on');
add_line(subsystem, 'position_error_raw/1', 'position_error_plus_2pi/1', 'autorouting', 'on');
add_line(subsystem, 'two_pi/1', 'position_error_plus_2pi/2', 'autorouting', 'on');
add_line(subsystem, 'position_error_minus_2pi/1', 'wrap_high_switch/1', 'autorouting', 'on');
add_line(subsystem, 'position_error_raw/1', 'wrap_high_switch/2', 'autorouting', 'on');
add_line(subsystem, 'position_error_raw/1', 'wrap_high_switch/3', 'autorouting', 'on');
add_line(subsystem, 'position_error_raw/1', 'is_below_neg_pi/1', 'autorouting', 'on');
add_line(subsystem, 'neg_pi/1', 'is_below_neg_pi/2', 'autorouting', 'on');
add_line(subsystem, 'position_error_plus_2pi/1', 'wrap_low_switch/1', 'autorouting', 'on');
add_line(subsystem, 'is_below_neg_pi/1', 'wrap_low_switch/2', 'autorouting', 'on');
add_line(subsystem, 'wrap_high_switch/1', 'wrap_low_switch/3', 'autorouting', 'on');
add_line(subsystem, 'wrap_low_switch/1', 'position_error_rad/1', 'autorouting', 'on');
end

function add_speed_error(subsystem, cfg)
add_block('simulink/Math Operations/Sum', [subsystem '/speed_error_rad_s'], ...
    'Position', [180 245 210 280], ...
    'Inputs', '+-', ...
    'AccumDataTypeStr', cfg.speedTypeName, ...
    'OutDataTypeStr', cfg.speedTypeName);
add_line(subsystem, 'input_selector/3', 'speed_error_rad_s/1', 'autorouting', 'on');
add_line(subsystem, 'input_selector/4', 'speed_error_rad_s/2', 'autorouting', 'on');
end

function add_torque_command(subsystem, cfg)
add_block('simulink/Math Operations/Product', [subsystem '/stiffness_torque'], ...
    'Position', [870 125 910 165], ...
    'OutDataTypeStr', cfg.torqueTypeName);
add_block('simulink/Math Operations/Product', [subsystem '/damping_torque'], ...
    'Position', [300 250 340 290], ...
    'OutDataTypeStr', cfg.torqueTypeName);
add_block('simulink/Math Operations/Sum', [subsystem '/torque_cmd_nm'], ...
    'Position', [950 180 980 245], ...
    'Inputs', '+++', ...
    'AccumDataTypeStr', cfg.torqueTypeName, ...
    'OutDataTypeStr', cfg.torqueTypeName);
add_block('simulink/Math Operations/Product', [subsystem '/iq_unsat_a'], ...
    'Position', [1015 185 1055 225], ...
    'OutDataTypeStr', cfg.currentTypeName);

add_line(subsystem, 'position_error_rad_from/1', 'stiffness_torque/1', 'autorouting', 'on');
add_line(subsystem, 'input_selector/6', 'stiffness_torque/2', 'autorouting', 'on');
add_line(subsystem, 'speed_error_rad_s/1', 'damping_torque/1', 'autorouting', 'on');
add_line(subsystem, 'input_selector/7', 'damping_torque/2', 'autorouting', 'on');
add_line(subsystem, 'stiffness_torque/1', 'torque_cmd_nm/1', 'autorouting', 'on');
add_line(subsystem, 'damping_torque/1', 'torque_cmd_nm/2', 'autorouting', 'on');
add_line(subsystem, 'input_selector/5', 'torque_cmd_nm/3', 'autorouting', 'on');
add_line(subsystem, 'torque_cmd_nm/1', 'iq_unsat_a/1', 'autorouting', 'on');
add_line(subsystem, 'input_selector/8', 'iq_unsat_a/2', 'autorouting', 'on');
end

function add_iq_limit(subsystem, cfg)
add_block('simulink/Math Operations/Abs', [subsystem '/iq_limit_abs'], ...
    'Position', [870 335 910 365], ...
    'OutDataTypeStr', cfg.currentTypeName);
add_block('simulink/Math Operations/Gain', [subsystem '/neg_iq_limit'], ...
    'Position', [950 335 1010 365], ...
    'Gain', '-1', ...
    'ParamDataTypeStr', cfg.floatTypeName, ...
    'OutDataTypeStr', cfg.currentTypeName);
add_minmax(subsystem, 'iq_upper_limited', [1045 255 1105 290], 'min');
add_minmax(subsystem, 'iq_ref_a', [1145 255 1205 290], 'max');

add_block('simulink/Math Operations/Abs', [subsystem '/iq_unsat_abs'], ...
    'Position', [1045 395 1085 425], ...
    'OutDataTypeStr', cfg.currentTypeName);
add_block('simulink/Logic and Bit Operations/Relational Operator', ...
    [subsystem '/is_saturated'], ...
    'Position', [1130 395 1180 425], ...
    'Operator', '>');
add_block('simulink/Signal Attributes/Data Type Conversion', ...
    [subsystem '/saturated_flag'], ...
    'Position', [1220 395 1280 425], ...
    'OutDataTypeStr', cfg.floatTypeName);

add_line(subsystem, 'input_selector/9', 'iq_limit_abs/1', 'autorouting', 'on');
add_line(subsystem, 'iq_limit_abs/1', 'neg_iq_limit/1', 'autorouting', 'on');
add_line(subsystem, 'iq_unsat_a/1', 'iq_upper_limited/1', 'autorouting', 'on');
add_line(subsystem, 'iq_limit_abs/1', 'iq_upper_limited/2', 'autorouting', 'on');
add_line(subsystem, 'iq_upper_limited/1', 'iq_ref_a/1', 'autorouting', 'on');
add_line(subsystem, 'neg_iq_limit/1', 'iq_ref_a/2', 'autorouting', 'on');
add_line(subsystem, 'iq_unsat_a/1', 'iq_unsat_abs/1', 'autorouting', 'on');
add_line(subsystem, 'iq_unsat_abs/1', 'is_saturated/1', 'autorouting', 'on');
add_line(subsystem, 'iq_limit_abs/1', 'is_saturated/2', 'autorouting', 'on');
add_line(subsystem, 'is_saturated/1', 'saturated_flag/1', 'autorouting', 'on');
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

function add_minmax(model, block_name, position, function_name)
add_block('simulink/Math Operations/MinMax', [model '/' block_name], ...
    'Position', position, ...
    'Inputs', '2', ...
    'Function', function_name);
end

function name_line(line_handle, line_name)
try
    set_param(line_handle, 'Name', line_name);
catch
end
end
