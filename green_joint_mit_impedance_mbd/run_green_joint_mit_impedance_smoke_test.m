%% Smoke test for green_joint_mit_impedance_model
%
% Builds the MIT impedance MBD core and verifies normal physical-domain
% output, current saturation, and position error wrap across +/-pi.

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

run(fullfile(script_dir, 'build_green_joint_mit_impedance_model.m'));

cfg = evalin('base', 'gj_mit_config');
model = cfg.modelName;
model_file = fullfile(script_dir, [model '.slx']);
load_system(model_file);

cases = define_smoke_cases();
for i = 1:numel(cases)
    run_case(cfg, model, cases(i));
end

fprintf('\nGreen-joint MIT impedance smoke test passed (%d cases).\n', numel(cases));

function cases = define_smoke_cases()
base = evalin('base', 'gj_mit_test');

cases(1).name = 'normal_15hz_0p2rad';
cases(1).input = base;
cases(1).expected_iq = double(base.expected_iq_a);

cases(2).name = 'positive_saturation';
cases(2).input = base;
cases(2).input.pos_target_rad = single(1.0);
cases(2).input.iq_limit_a = single(0.5);
cases(2).expected_iq = 0.5;

cases(3).name = 'wrap_negative_crossing';
cases(3).input = base;
cases(3).input.pos_target_rad = single(-3.0);
cases(3).input.pos_feedback_rad = single(3.0);
cases(3).input.vel_target_rad_s = single(0.0);
cases(3).input.vel_feedback_rad_s = single(0.0);
cases(3).input.ff_torque_nm = single(0.0);
position_error = -3.0 - 3.0 + 2.0 * pi;
cases(3).expected_iq = double(position_error) * ...
    double(base.kp_nm_per_rad) * double(base.torque_to_iq_gain_a_per_nm);
end

function run_case(cfg, referenced_model, test_case)
harness = ['green_joint_mit_smoke_harness_' test_case.name];
if bdIsLoaded(harness)
    close_system(harness, 0);
end

assignin('base', 'gj_mit_case', test_case.input);
new_system(harness);
cleanup_harness = onCleanup(@() close_harness_without_saving(harness));
set_param(harness, ...
    'DataDictionary', cfg.dictionaryName, ...
    'StopTime', 'gj_mit_simcfg.Ts_mit', ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', 'gj_mit_simcfg.Ts_mit', ...
    'ParameterPrecisionLossMsg', 'none');

add_case_harness_blocks(harness, cfg, referenced_model);

sim_out = sim(harness, ...
    'ReturnWorkspaceOutputs', 'on', ...
    'SaveOutput', 'off');

iq_ref = last_value(sim_out.get('log_iq_ref_a'));

fprintf('\nMIT smoke case: %s\n', test_case.name);
fprintf('  iq_ref_a           = %.9g A\n', iq_ref);

if abs(iq_ref - test_case.expected_iq) > 2e-5
    error('MIT smoke case %s failed: iq_ref mismatch.', test_case.name);
end

clear cleanup_harness;
close_harness_without_saving(harness);
end

function add_case_harness_blocks(harness, cfg, referenced_model)
source_specs = { ...
    'pos_target_rad', 'gj_mit_case.pos_target_rad', cfg.angleTypeName; ...
    'pos_feedback_rad', 'gj_mit_case.pos_feedback_rad', cfg.angleTypeName; ...
    'vel_target_rad_s', 'gj_mit_case.vel_target_rad_s', cfg.speedTypeName; ...
    'vel_feedback_rad_s', 'gj_mit_case.vel_feedback_rad_s', cfg.speedTypeName; ...
    'ff_torque_nm', 'gj_mit_case.ff_torque_nm', cfg.torqueTypeName; ...
    'kp_nm_per_rad', 'gj_mit_case.kp_nm_per_rad', cfg.gainTypeName; ...
    'kd_nm_s_per_rad', 'gj_mit_case.kd_nm_s_per_rad', cfg.gainTypeName; ...
    'torque_to_iq_gain_a_per_nm', 'gj_mit_case.torque_to_iq_gain_a_per_nm', cfg.gainTypeName; ...
    'iq_limit_a', 'gj_mit_case.iq_limit_a', cfg.currentTypeName};

for i = 1:size(source_specs, 1)
    y = 40 + (i - 1) * 45;
    add_typed_constant(harness, source_specs{i, 1}, [40 y 145 y + 25], ...
        source_specs{i, 2}, source_specs{i, 3}, cfg.sampleTime);
end

add_block('simulink/Signal Routing/Bus Creator', [harness '/mit_input_bus_creator'], ...
    'Position', [210 35 225 420], ...
    'Inputs', '9', ...
    'UseBusObject', 'on', ...
    'BusObject', cfg.inputBusName, ...
    'NonVirtualBus', 'on');
add_block('simulink/Ports & Subsystems/Model', [harness '/GreenJointMitModelRef'], ...
    'Position', [300 170 520 260], ...
    'ModelName', referenced_model);
add_block('simulink/Signal Routing/Bus Selector', [harness '/mit_output_selector'], ...
    'Position', [580 190 600 230], ...
    'OutputSignals', 'iq_ref_a');

for i = 1:size(source_specs, 1)
    name_line(add_line(harness, [source_specs{i, 1} '/1'], ...
        ['mit_input_bus_creator/' num2str(i)], 'autorouting', 'on'), ...
        source_specs{i, 1});
end

add_line(harness, 'mit_input_bus_creator/1', 'GreenJointMitModelRef/1', 'autorouting', 'on');
add_line(harness, 'GreenJointMitModelRef/1', 'mit_output_selector/1', 'autorouting', 'on');

add_scalar_logger(harness, 'log_iq_ref_a', [660 190 760 220], 'mit_output_selector/1');
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

function y = last_value(signal_data)
if isa(signal_data, 'timeseries')
    y = double(signal_data.Data(end));
elseif isstruct(signal_data)
    y = double(signal_data.signals.values(end));
else
    y = double(signal_data(end));
end
end

function close_harness_without_saving(harness)
if bdIsLoaded(harness)
    set_param(harness, 'Dirty', 'off');
    close_system(harness, 0);
end
end

function name_line(line_handle, line_name)
try
    set_param(line_handle, 'Name', line_name);
catch
end
end
