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
runtime_profiles = {'1615', '1620'};
verify_runtime_profiles_exist(script_dir, runtime_profiles);
contract_profile = get_contract_profile('1615', runtime_profiles);
verify_contract_matches_firmware_variant(contract, script_dir, contract_profile);

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
fprintf('  app fw     = jointboard_mh3p0 common firmware\n');
fprintf('  profiles   = %s runtime profiles; pmotor.motor_type selects profile, invalid Flash falls back to 1620\n', ...
    strjoin(runtime_profiles, '/'));
fprintf('  defaults   = green_joint_%s current-loop contract values; firmware applies runtime profile values\n', ...
    contract_profile);
fprintf('Green-joint current-loop MBD smoke test passed.\n');

function verify_runtime_profiles_exist(script_dir, runtime_profiles)
for i = 1:numel(runtime_profiles)
    load_firmware_variant_config(script_dir, runtime_profiles{i});
end
end

function verify_contract_matches_firmware_variant(contract, script_dir, motor_type)
cfg = load_firmware_variant_config(script_dir, motor_type);
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

function cfg = load_firmware_variant_config(script_dir, motor_type)
repo_dir = fileparts(script_dir);
workspace_dir = fileparts(repo_dir);
firmware_dir = fullfile(workspace_dir, 'green-joint');
config_file = fullfile(firmware_dir, 'Module', 'Config', ...
    ['green_joint_' char(motor_type) '_config.json']);
if ~exist(config_file, 'file')
    error('Missing firmware runtime profile contract: %s', config_file);
end

cfg = jsondecode(fileread(config_file));
required_fields = {'sample_time_s', 'cur_d_kp', 'cur_d_ki', 'cur_q_kp', ...
    'cur_q_ki', 'pi_correction_gain', 'voltage_limit_ratio', ...
    'voltage_modulation_ratio'};
for i = 1:numel(required_fields)
    if ~isfield(cfg.current_loop, required_fields{i})
        error('Missing current-loop field "%s" in runtime profile %s.', ...
            required_fields{i}, char(motor_type));
    end
end
end

function contract_profile = get_contract_profile(default_profile, runtime_profiles)
contract_profile = getenv('GJ_MBD_CONTRACT_PROFILE');
if isempty(contract_profile)
    contract_profile = default_profile;
end

if ~any(strcmp(contract_profile, runtime_profiles))
    error('GJ_MBD_CONTRACT_PROFILE must be one of: %s.', ...
        strjoin(runtime_profiles, ', '));
end
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
