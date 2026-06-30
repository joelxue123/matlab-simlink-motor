%% Smoke test for green_joint_current_loop_model
%
% The current milestone verifies the formal MBD shell plus the first
% algorithm increment: Clarke/Park and adaptive dq current filtering.

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
run(fullfile(script_dir, 'build_green_joint_current_loop_model.m'));

contract = evalin('base', 'green_joint_current_loop_contract');
model = contract.model;
verify_contract_matches_firmware_variant(contract, script_dir, '1615');

load_system(fullfile(script_dir, [model '.slx']));
set_param(model, 'SimulationCommand', 'update');

assert_dictionary_entry(contract.dictionary, 'T_GJCurrent');
assert_dictionary_entry(contract.dictionary, 'T_GJVoltage');
assert_dictionary_entry(contract.dictionary, 'green_joint_current_loop_input_t');
assert_dictionary_entry(contract.dictionary, 'green_joint_current_loop_output_t');
assert_dictionary_entry(contract.dictionary, 'CurDKp');
assert_dictionary_entry(contract.dictionary, 'CurDKi');

fprintf('\nGreen-joint current-loop MBD smoke test result:\n');
fprintf('  model      = %s\n', model);
fprintf('  dictionary = %s\n', contract.dictionary);
fprintf('  input bus  = %s\n', contract.buses{1}.name);
fprintf('  output bus = %s\n', contract.buses{2}.name);
fprintf('  variant    = green_joint_1615 current-loop defaults\n');
fprintf('Green-joint current-loop MBD smoke test passed.\n');

function verify_contract_matches_firmware_variant(contract, script_dir, motor_type)
repo_dir = fileparts(script_dir);
workspace_dir = fileparts(repo_dir);
firmware_dir = fullfile(workspace_dir, 'green-joint');
config_file = fullfile(firmware_dir, 'Module', 'Config', ...
    ['green_joint_' motor_type '_config.json']);
if ~exist(config_file, 'file')
    error('Missing firmware variant contract: %s', config_file);
end

cfg = jsondecode(fileread(config_file));
assert_close(contract.sample_time_s, cfg.current_loop.sample_time_s, ...
    1e-12, 'sample_time_s');
assert_param(contract, 'CurDKp', cfg.current_loop.cur_d_kp, 1e-9);
assert_param(contract, 'CurDKi', cfg.current_loop.cur_d_ki, 1e-6);
assert_param(contract, 'CurQKp', cfg.current_loop.cur_q_kp, 1e-9);
assert_param(contract, 'CurQKi', cfg.current_loop.cur_q_ki, 1e-6);
assert_param(contract, 'PiCorrectionGain', ...
    cfg.current_loop.pi_correction_gain, 1e-6);
assert_param(contract, 'VoltageLimitRatio', ...
    cfg.current_loop.voltage_limit_ratio, 1e-9);
assert_param(contract, 'VoltageModulationRatio', ...
    cfg.current_loop.voltage_modulation_ratio, 1e-9);
end

function assert_param(contract, name, expected, tolerance)
value = get_parameter_value(contract, name);
assert_close(value, expected, tolerance, name);
end

function value = get_parameter_value(contract, name)
for i = 1:numel(contract.parameters)
    parameter = contract.parameters{i};
    if strcmp(parameter.name, name)
        value = parameter.value;
        return;
    end
end
error('Missing current-loop contract parameter "%s".', name);
end

function assert_close(actual, expected, tolerance, name)
if abs(double(actual) - double(expected)) > tolerance
    error('%s mismatch: actual %.12g expected %.12g tolerance %.12g', ...
        name, double(actual), double(expected), tolerance);
end
end

function assert_dictionary_entry(dictionary_name, entry_name)
dd = Simulink.data.dictionary.open(dictionary_name);
cleanup = onCleanup(@() close(dd));
section = getSection(dd, 'Design Data');
entry = find(section, 'Name', entry_name);
if isempty(entry)
    error('Dictionary entry "%s" was not generated.', entry_name);
end
end
