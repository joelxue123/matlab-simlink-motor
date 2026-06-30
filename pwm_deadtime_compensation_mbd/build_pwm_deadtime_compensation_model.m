%% Build PWM dead-time compensation MBD module
%
% User-level algorithm:
%   duty + synthesized phase-current sign and smooth small-current gain
%   -> compensated duty

clear; clc;

cfg = customer_interface_config();
script_dir = fileparts(mfilename('fullpath'));
cfg.dictionaryFile = fullfile(script_dir, cfg.dictionaryName);
assignin('base', 'pwm_deadtime_compensation_config', cfg);

previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

defaults = default_deadtime_compensation_params();
assignin('base', 'deadtime_comp_defaults', defaults);
assignin('base', 'deadtime_comp_test', defaults.test);

model = cfg.modelName;
model_file = fullfile(script_dir, [model '.slx']);

if bdIsLoaded(model)
    close_system(model, 0);
end

close_open_data_dictionaries();
create_deadtime_compensation_dictionary(cfg, defaults);

if exist(model_file, 'file')
    delete(model_file);
end

new_system(model);
set_param(model, 'DataDictionary', cfg.dictionaryName);
set_param(model, ...
    'StopTime', 'deadtime_comp_defaults.simcfg.stop_time', ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', 'deadtime_comp_defaults.simcfg.Ts_step', ...
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

add_test_sources(model, cfg);
add_compensation_subsystem(model, cfg);
add_output_sinks(model, cfg);

set_param(model, 'SimulationCommand', 'update');
save_system(model, model_file);

fprintf('Built PWM dead-time compensation MBD model:\n  %s\n', model_file);

function cfg = customer_interface_config()
cfg.modelName = 'pwm_deadtime_compensation_model';
cfg.functionName = 'DeadtimeCompensationStep';
cfg.sampleTime = 'deadtime_comp_defaults.simcfg.Ts_step';

cfg.dictionaryName = 'pwm_deadtime_compensation_interface.sldd';
cfg.typeHeaderFile = 'pwm_deadtime_compensation_types.h';
cfg.rebuildDictionary = true;

cfg.floatBaseType = 'single';
cfg.boolBaseType = 'boolean';
cfg.dutyTypeName = 'T_DeadtimeCompDuty';
cfg.currentTypeName = 'T_DeadtimeCompCurrent';
cfg.trigTypeName = 'T_DeadtimeCompTrig';
cfg.validTypeName = 'T_DeadtimeCompBool';
cfg.inputBusName = 'pwm_deadtime_comp_input_t';
cfg.outputBusName = 'pwm_deadtime_comp_output_t';
end

function defaults = default_deadtime_compensation_params()
defaults.simcfg.stop_time = 50e-6;
defaults.simcfg.Ts_step = 50e-6;

defaults.DeadtimeCompEnable = true;
defaults.DeadtimeCompDuty = single(0.01000);
defaults.DeadtimeCompCurrentZero_A = single(0.02);
defaults.DeadtimeCompCurrentFull_A = single(0.10);
defaults.DeadtimeCompCurrentInvRange_1perA = single( ...
    1.0 / double(defaults.DeadtimeCompCurrentFull_A - defaults.DeadtimeCompCurrentZero_A));
defaults.DeadtimeCompPolarity = single(-1.0);

defaults.test.da = single(0.05);
defaults.test.db = single(0.95);
defaults.test.dc = single(0.50);
defaults.test.id = single(0.0);
defaults.test.iq = single(0.2);
defaults.test.sin_theta_e = single(0.0);
defaults.test.cos_theta_e = single(1.0);
end

function close_open_data_dictionaries()
try
    Simulink.data.dictionary.closeAll('-discard');
catch err
    warning('Could not close open data dictionaries before rebuild: %s', ...
        err.message);
end
end

function create_deadtime_compensation_dictionary(cfg, defaults)
dd = open_or_create_data_dictionary(cfg);
design_data = getSection(dd, 'Design Data');

upsert_alias_type(design_data, cfg.dutyTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.currentTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.trigTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.validTypeName, cfg.boolBaseType, cfg.typeHeaderFile);

upsert_bus_type(design_data, cfg.inputBusName, cfg.typeHeaderFile, { ...
    {'da', cfg.dutyTypeName}; ...
    {'db', cfg.dutyTypeName}; ...
    {'dc', cfg.dutyTypeName}; ...
    {'id', cfg.currentTypeName}; ...
    {'iq', cfg.currentTypeName}; ...
    {'sin_theta_e', cfg.trigTypeName}; ...
    {'cos_theta_e', cfg.trigTypeName}});

upsert_bus_type(design_data, cfg.outputBusName, cfg.typeHeaderFile, { ...
    {'da', cfg.dutyTypeName}; ...
    {'db', cfg.dutyTypeName}; ...
    {'dc', cfg.dutyTypeName}; ...
    {'comp_a', cfg.dutyTypeName}; ...
    {'comp_b', cfg.dutyTypeName}; ...
    {'comp_c', cfg.dutyTypeName}; ...
    {'active_a', cfg.validTypeName}; ...
    {'active_b', cfg.validTypeName}; ...
    {'active_c', cfg.validTypeName}});

upsert_parameter(design_data, 'DeadtimeCompEnable', defaults.DeadtimeCompEnable, cfg.validTypeName);
upsert_parameter(design_data, 'DeadtimeCompDuty', defaults.DeadtimeCompDuty, cfg.dutyTypeName);
upsert_parameter(design_data, 'DeadtimeCompCurrentZero_A', defaults.DeadtimeCompCurrentZero_A, cfg.currentTypeName);
upsert_parameter(design_data, 'DeadtimeCompCurrentFull_A', defaults.DeadtimeCompCurrentFull_A, cfg.currentTypeName);
upsert_parameter(design_data, 'DeadtimeCompCurrentInvRange_1perA', defaults.DeadtimeCompCurrentInvRange_1perA, cfg.dutyTypeName);
upsert_parameter(design_data, 'DeadtimeCompPolarity', defaults.DeadtimeCompPolarity, cfg.dutyTypeName);

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

function add_test_sources(model, cfg)
source_specs = { ...
    'da', 'deadtime_comp_test.da', cfg.dutyTypeName, [45 35 115 60]; ...
    'db', 'deadtime_comp_test.db', cfg.dutyTypeName, [45 75 115 100]; ...
    'dc', 'deadtime_comp_test.dc', cfg.dutyTypeName, [45 115 115 140]; ...
    'id', 'deadtime_comp_test.id', cfg.currentTypeName, [45 165 115 190]; ...
    'iq', 'deadtime_comp_test.iq', cfg.currentTypeName, [45 205 115 230]; ...
    'sin_theta_e', 'deadtime_comp_test.sin_theta_e', cfg.trigTypeName, [45 245 135 270]; ...
    'cos_theta_e', 'deadtime_comp_test.cos_theta_e', cfg.trigTypeName, [45 285 135 310]};

for i = 1:size(source_specs, 1)
    add_typed_constant(model, source_specs{i, 1}, source_specs{i, 2}, ...
        source_specs{i, 3}, source_specs{i, 4}, cfg.sampleTime);
end

add_block('simulink/Signal Routing/Bus Creator', [model '/comp_input_bus_creator'], ...
    'Position', [170 35 190 310], ...
    'Inputs', '7', ...
    'UseBusObject', 'on', ...
    'BusObject', cfg.inputBusName, ...
    'NonVirtualBus', 'on');

names = {'da', 'db', 'dc', 'id', 'iq', 'sin_theta_e', 'cos_theta_e'};
for i = 1:numel(names)
    name_line(add_line(model, [names{i} '/1'], ...
        sprintf('comp_input_bus_creator/%d', i), 'autorouting', 'on'), names{i});
end
end

function add_compensation_subsystem(model, cfg)
subsystem = [model '/' cfg.functionName];
add_clean_subsystem(subsystem, [275 95 510 185]);

set_param(subsystem, ...
    'TreatAsAtomicUnit', 'on', ...
    'RTWSystemCode', 'Reusable function', ...
    'RTWFcnNameOpts', 'User specified', ...
    'RTWFcnName', cfg.functionName, ...
    'RTWFileNameOpts', 'User specified', ...
    'RTWFileName', cfg.functionName);

add_block('simulink/Sources/In1', [subsystem '/comp_in'], ...
    'Position', [35 125 65 145], ...
    'OutDataTypeStr', ['Bus: ' cfg.inputBusName], ...
    'BusOutputAsStruct', 'on', ...
    'PortDimensions', '1');

add_block('simulink/Signal Routing/Bus Selector', [subsystem '/select_input'], ...
    'Position', [110 75 150 205], ...
    'OutputSignals', 'da,db,dc,id,iq,sin_theta_e,cos_theta_e');
add_line(subsystem, 'comp_in/1', 'select_input/1', 'autorouting', 'on');

add_comp_constant(subsystem, 'zero_duty', '0', [155 265 220 285], cfg.dutyTypeName);
add_comp_constant(subsystem, 'zero_current', '0', [155 545 240 565], cfg.currentTypeName);
add_comp_constant(subsystem, 'one_sign', '1', [155 300 220 320], cfg.dutyTypeName);
add_comp_constant(subsystem, 'minus_one_sign', '-1', [155 335 220 355], cfg.dutyTypeName);
add_comp_constant(subsystem, 'DeadtimeCompDuty', 'DeadtimeCompDuty', [155 370 270 390], cfg.dutyTypeName);
add_comp_constant(subsystem, 'DeadtimeCompPolarity', 'DeadtimeCompPolarity', [155 405 290 425], cfg.dutyTypeName);
add_comp_constant(subsystem, 'DeadtimeCompCurrentZero_A', 'DeadtimeCompCurrentZero_A', [155 440 330 460], cfg.currentTypeName);
add_comp_constant(subsystem, 'DeadtimeCompCurrentInvRange_1perA', 'DeadtimeCompCurrentInvRange_1perA', [155 475 370 495], cfg.dutyTypeName);
add_comp_constant(subsystem, 'DeadtimeCompEnable', 'DeadtimeCompEnable', [155 510 310 530], cfg.validTypeName);

[phase_current_sources, phase_current_names] = add_synthesized_phase_currents(subsystem, cfg);

phase_specs = { ...
    'a', 1, 55; ...
    'b', 2, 180; ...
    'c', 3, 305};

comp_sources = cell(3, 1);
duty_sources = cell(3, 1);
active_sources = cell(3, 1);
for i = 1:3
    [duty_sources{i}, comp_sources{i}, active_sources{i}] = add_phase_comp_path( ...
        subsystem, phase_specs{i, 1}, phase_specs{i, 2}, ...
        phase_current_sources{i}, phase_current_names{i}, phase_specs{i, 3}, cfg);
end

add_block('simulink/Signal Routing/Bus Creator', [subsystem '/comp_output_bus_creator'], ...
    'Position', [860 75 890 440], ...
    'Inputs', '9', ...
    'UseBusObject', 'on', ...
    'BusObject', cfg.outputBusName, ...
    'NonVirtualBus', 'on');

output_signals = { ...
    'da', duty_sources{1}; ...
    'db', duty_sources{2}; ...
    'dc', duty_sources{3}; ...
    'comp_a', comp_sources{1}; ...
    'comp_b', comp_sources{2}; ...
    'comp_c', comp_sources{3}; ...
    'active_a', active_sources{1}; ...
    'active_b', active_sources{2}; ...
    'active_c', active_sources{3}};

for i = 1:size(output_signals, 1)
    name_line(add_line(subsystem, output_signals{i, 2}, ...
        sprintf('comp_output_bus_creator/%d', i), 'autorouting', 'on'), ...
        output_signals{i, 1});
end

add_block('simulink/Sinks/Out1', [subsystem '/comp_out'], ...
    'Position', [960 245 990 265], ...
    'OutDataTypeStr', ['Bus: ' cfg.outputBusName], ...
    'PortDimensions', '1');
add_line(subsystem, 'comp_output_bus_creator/1', 'comp_out/1', 'autorouting', 'on');

Simulink.BlockDiagram.arrangeSystem(subsystem);

add_line(model, 'comp_input_bus_creator/1', [cfg.functionName '/1'], 'autorouting', 'on');
end

function [phase_current_sources, phase_current_names] = add_synthesized_phase_currents(subsystem, cfg)
% Inverse Park + inverse Clarke:
%   i_alpha = id*cos(theta) - iq*sin(theta)
%   i_beta  = id*sin(theta) + iq*cos(theta)
%   ia = i_alpha
%   ib = -0.5*i_alpha + sqrt(3)/2*i_beta
%   ic = -0.5*i_alpha - sqrt(3)/2*i_beta

add_block('simulink/Math Operations/Product', [subsystem '/id_cos'], ...
    'Position', [225 580 270 610], ...
    'Inputs', '**', ...
    'OutDataTypeStr', cfg.currentTypeName);
add_line(subsystem, 'select_input/4', 'id_cos/1', 'autorouting', 'on');
add_line(subsystem, 'select_input/7', 'id_cos/2', 'autorouting', 'on');

add_block('simulink/Math Operations/Product', [subsystem '/iq_sin'], ...
    'Position', [225 620 270 650], ...
    'Inputs', '**', ...
    'OutDataTypeStr', cfg.currentTypeName);
add_line(subsystem, 'select_input/5', 'iq_sin/1', 'autorouting', 'on');
add_line(subsystem, 'select_input/6', 'iq_sin/2', 'autorouting', 'on');

add_block('simulink/Math Operations/Sum', [subsystem '/i_alpha_synth'], ...
    'Position', [315 590 350 630], ...
    'Inputs', '+-', ...
    'OutDataTypeStr', cfg.currentTypeName);
add_line(subsystem, 'id_cos/1', 'i_alpha_synth/1', 'autorouting', 'on');
add_line(subsystem, 'iq_sin/1', 'i_alpha_synth/2', 'autorouting', 'on');

add_block('simulink/Math Operations/Product', [subsystem '/id_sin'], ...
    'Position', [225 675 270 705], ...
    'Inputs', '**', ...
    'OutDataTypeStr', cfg.currentTypeName);
add_line(subsystem, 'select_input/4', 'id_sin/1', 'autorouting', 'on');
add_line(subsystem, 'select_input/6', 'id_sin/2', 'autorouting', 'on');

add_block('simulink/Math Operations/Product', [subsystem '/iq_cos'], ...
    'Position', [225 715 270 745], ...
    'Inputs', '**', ...
    'OutDataTypeStr', cfg.currentTypeName);
add_line(subsystem, 'select_input/5', 'iq_cos/1', 'autorouting', 'on');
add_line(subsystem, 'select_input/7', 'iq_cos/2', 'autorouting', 'on');

add_block('simulink/Math Operations/Sum', [subsystem '/i_beta_synth'], ...
    'Position', [315 690 350 730], ...
    'Inputs', '++', ...
    'OutDataTypeStr', cfg.currentTypeName);
add_line(subsystem, 'id_sin/1', 'i_beta_synth/1', 'autorouting', 'on');
add_line(subsystem, 'iq_cos/1', 'i_beta_synth/2', 'autorouting', 'on');

add_gain(subsystem, 'minus_half_alpha_b', '-0.5', [395 625 455 655], cfg.currentTypeName);
add_line(subsystem, 'i_alpha_synth/1', 'minus_half_alpha_b/1', 'autorouting', 'on');
add_gain(subsystem, 'sqrt3_half_beta_b', '0.8660254037844386', [395 665 475 695], cfg.currentTypeName);
add_line(subsystem, 'i_beta_synth/1', 'sqrt3_half_beta_b/1', 'autorouting', 'on');
add_block('simulink/Math Operations/Sum', [subsystem '/ib_synth'], ...
    'Position', [520 640 560 680], ...
    'Inputs', '++', ...
    'OutDataTypeStr', cfg.currentTypeName);
add_line(subsystem, 'minus_half_alpha_b/1', 'ib_synth/1', 'autorouting', 'on');
add_line(subsystem, 'sqrt3_half_beta_b/1', 'ib_synth/2', 'autorouting', 'on');

add_gain(subsystem, 'minus_half_alpha_c', '-0.5', [395 730 455 760], cfg.currentTypeName);
add_line(subsystem, 'i_alpha_synth/1', 'minus_half_alpha_c/1', 'autorouting', 'on');
add_gain(subsystem, 'minus_sqrt3_half_beta_c', '-0.8660254037844386', [395 770 485 800], cfg.currentTypeName);
add_line(subsystem, 'i_beta_synth/1', 'minus_sqrt3_half_beta_c/1', 'autorouting', 'on');
add_block('simulink/Math Operations/Sum', [subsystem '/ic_synth'], ...
    'Position', [520 745 560 785], ...
    'Inputs', '++', ...
    'OutDataTypeStr', cfg.currentTypeName);
add_line(subsystem, 'minus_half_alpha_c/1', 'ic_synth/1', 'autorouting', 'on');
add_line(subsystem, 'minus_sqrt3_half_beta_c/1', 'ic_synth/2', 'autorouting', 'on');

phase_current_sources = {'i_alpha_synth/1', 'ib_synth/1', 'ic_synth/1'};
phase_current_names = {'ia_synth', 'ib_synth', 'ic_synth'};
end

function [duty_src, comp_src, active_src] = add_phase_comp_path(subsystem, phase, duty_port, current_src, current_name, y0, cfg)
prefix = ['phase_' phase '_'];

add_block('simulink/Math Operations/Abs', [subsystem '/' prefix 'abs_current'], ...
    'Position', [235 y0 265 y0+30]);
name_line(add_line(subsystem, current_src, [prefix 'abs_current/1'], 'autorouting', 'on'), current_name);

add_block('simulink/Math Operations/Sum', [subsystem '/' prefix 'current_minus_zero'], ...
    'Position', [305 y0 345 y0+30], ...
    'Inputs', '+-', ...
    'OutDataTypeStr', cfg.currentTypeName);
add_line(subsystem, [prefix 'abs_current/1'], [prefix 'current_minus_zero/1'], 'autorouting', 'on');
add_line(subsystem, 'DeadtimeCompCurrentZero_A/1', [prefix 'current_minus_zero/2'], 'autorouting', 'on');

add_block('simulink/Math Operations/Product', [subsystem '/' prefix 'gain_raw'], ...
    'Position', [380 y0 425 y0+30], ...
    'Inputs', '**', ...
    'OutDataTypeStr', cfg.dutyTypeName);
add_line(subsystem, [prefix 'current_minus_zero/1'], [prefix 'gain_raw/1'], 'autorouting', 'on');
add_line(subsystem, 'DeadtimeCompCurrentInvRange_1perA/1', [prefix 'gain_raw/2'], 'autorouting', 'on');

add_block('simulink/Discontinuities/Saturation', [subsystem '/' prefix 'gain_saturation'], ...
    'Position', [465 y0 505 y0+30], ...
    'LowerLimit', '0', ...
    'UpperLimit', '1', ...
    'OutDataTypeStr', cfg.dutyTypeName);
add_line(subsystem, [prefix 'gain_raw/1'], [prefix 'gain_saturation/1'], 'autorouting', 'on');

add_block('simulink/Logic and Bit Operations/Relational Operator', [subsystem '/' prefix 'gain_positive'], ...
    'Position', [545 y0 585 y0+30], ...
    'Operator', '>');
add_line(subsystem, [prefix 'gain_saturation/1'], [prefix 'gain_positive/1'], 'autorouting', 'on');
add_line(subsystem, 'zero_duty/1', [prefix 'gain_positive/2'], 'autorouting', 'on');

add_block('simulink/Logic and Bit Operations/Logical Operator', [subsystem '/' prefix 'active'], ...
    'Position', [625 y0 665 y0+35], ...
    'Operator', 'AND', ...
    'Inputs', '2', ...
    'OutDataTypeStr', cfg.validTypeName);
add_line(subsystem, [prefix 'gain_positive/1'], [prefix 'active/1'], 'autorouting', 'on');
add_line(subsystem, 'DeadtimeCompEnable/1', [prefix 'active/2'], 'autorouting', 'on');

add_block('simulink/Logic and Bit Operations/Relational Operator', [subsystem '/' prefix 'current_positive'], ...
    'Position', [305 y0+50 345 y0+80], ...
    'Operator', '>');
add_line(subsystem, current_src, [prefix 'current_positive/1'], 'autorouting', 'on');
add_line(subsystem, 'zero_current/1', [prefix 'current_positive/2'], 'autorouting', 'on');

add_block('simulink/Signal Routing/Switch', [subsystem '/' prefix 'sign_switch'], ...
    'Position', [465 y0+45 510 y0+95], ...
    'Criteria', 'u2 ~= 0', ...
    'OutDataTypeStr', cfg.dutyTypeName);
add_line(subsystem, 'one_sign/1', [prefix 'sign_switch/1'], 'autorouting', 'on');
add_line(subsystem, [prefix 'current_positive/1'], [prefix 'sign_switch/2'], 'autorouting', 'on');
add_line(subsystem, 'minus_one_sign/1', [prefix 'sign_switch/3'], 'autorouting', 'on');

add_block('simulink/Signal Routing/Switch', [subsystem '/' prefix 'active_sign_switch'], ...
    'Position', [705 y0+35 750 y0+85], ...
    'Criteria', 'u2 ~= 0', ...
    'OutDataTypeStr', cfg.dutyTypeName);
add_line(subsystem, [prefix 'sign_switch/1'], [prefix 'active_sign_switch/1'], 'autorouting', 'on');
add_line(subsystem, [prefix 'active/1'], [prefix 'active_sign_switch/2'], 'autorouting', 'on');
add_line(subsystem, 'zero_duty/1', [prefix 'active_sign_switch/3'], 'autorouting', 'on');

add_block('simulink/Math Operations/Product', [subsystem '/' prefix 'comp_calc'], ...
    'Position', [785 y0+38 830 y0+82], ...
    'Inputs', '****', ...
    'OutDataTypeStr', cfg.dutyTypeName);
add_line(subsystem, [prefix 'active_sign_switch/1'], [prefix 'comp_calc/1'], 'autorouting', 'on');
add_line(subsystem, 'DeadtimeCompDuty/1', [prefix 'comp_calc/2'], 'autorouting', 'on');
add_line(subsystem, 'DeadtimeCompPolarity/1', [prefix 'comp_calc/3'], 'autorouting', 'on');
add_line(subsystem, [prefix 'gain_saturation/1'], [prefix 'comp_calc/4'], 'autorouting', 'on');

add_block('simulink/Math Operations/Sum', [subsystem '/' prefix 'duty_plus_comp'], ...
    'Position', [870 y0+42 900 y0+78], ...
    'Inputs', '++', ...
    'OutDataTypeStr', cfg.dutyTypeName);
add_line(subsystem, sprintf('select_input/%d', duty_port), [prefix 'duty_plus_comp/1'], 'autorouting', 'on');
add_line(subsystem, [prefix 'comp_calc/1'], [prefix 'duty_plus_comp/2'], 'autorouting', 'on');

add_block('simulink/Discontinuities/Saturation', [subsystem '/' prefix 'duty_saturation'], ...
    'Position', [935 y0+42 975 y0+78], ...
    'LowerLimit', '0', ...
    'UpperLimit', '1', ...
    'OutDataTypeStr', cfg.dutyTypeName);
add_line(subsystem, [prefix 'duty_plus_comp/1'], [prefix 'duty_saturation/1'], 'autorouting', 'on');

duty_src = [prefix 'duty_saturation/1'];
comp_src = [prefix 'comp_calc/1'];
active_src = [prefix 'active/1'];
end

function add_output_sinks(model, cfg)
add_block('simulink/Sinks/Out1', [model '/comp_out'], ...
    'Position', [610 120 640 140], ...
    'OutDataTypeStr', ['Bus: ' cfg.outputBusName], ...
    'PortDimensions', '1');
add_line(model, [cfg.functionName '/1'], 'comp_out/1', 'autorouting', 'on');

add_block('simulink/Signal Routing/Bus Selector', [model '/comp_output_selector'], ...
    'Position', [585 215 635 425], ...
    'OutputSignals', 'da,db,dc,comp_a,comp_b,comp_c,active_a,active_b,active_c');
add_line(model, [cfg.functionName '/1'], 'comp_output_selector/1', 'autorouting', 'on');

fields = {'da', 'db', 'dc', 'comp_a', 'comp_b', 'comp_c', 'active_a', 'active_b', 'active_c'};
for i = 1:numel(fields)
    y = 205 + (i - 1) * 35;
    block_name = ['log_' fields{i}];
    add_block('simulink/Sinks/To Workspace', [model '/' block_name], ...
        'Position', [710 y 800 y+22], ...
        'VariableName', fields{i}, ...
        'SaveFormat', 'Structure With Time');
    add_line(model, sprintf('comp_output_selector/%d', i), [block_name '/1'], ...
        'autorouting', 'on');
end

Simulink.BlockDiagram.arrangeSystem(model);
end

function add_clean_subsystem(path, position)
add_block('simulink/Ports & Subsystems/Subsystem', path, ...
    'Position', position);

try
    delete_line(path, 'In1/1', 'Out1/1');
catch
end

if exist_block([path '/In1'])
    delete_block([path '/In1']);
end
if exist_block([path '/Out1'])
    delete_block([path '/Out1']);
end
end

function tf = exist_block(block_path)
try
    get_param(block_path, 'Handle');
    tf = true;
catch
    tf = false;
end
end

function add_typed_constant(model, name, value, data_type, position, sample_time)
add_block('simulink/Sources/Constant', [model '/' name], ...
    'Position', position, ...
    'Value', value, ...
    'OutDataTypeStr', data_type, ...
    'SampleTime', sample_time);
end

function add_comp_constant(model, name, value, position, data_type)
add_block('simulink/Sources/Constant', [model '/' name], ...
    'Position', position, ...
    'Value', value, ...
    'OutDataTypeStr', data_type, ...
    'SampleTime', '-1');
end

function add_gain(model, name, value, position, data_type)
add_block('simulink/Math Operations/Gain', [model '/' name], ...
    'Position', position, ...
    'Gain', value, ...
    'OutDataTypeStr', data_type);
end

function name_line(line_handle, signal_name)
set_param(line_handle, 'Name', signal_name);
end
