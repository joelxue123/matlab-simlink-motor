%% Build a visible 1 kHz square-wave test harness for the V1 average motor twin
%
% Output model:
%   green_joint_average_motor_square_wave_harness.slx
%
% This harness is intentionally separate from the reusable controller model.
% It keeps test-state logic outside the generated controller core while making
% the 1 kHz current-square scenario visible in Simulink.

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

run(fullfile(script_dir, 'setup_green_joint_current_loop_twin.m'));

GJDT_StopTime = 0.010;
GJDT_CurDKp = single(1.0);
GJDT_CurDKi = single(20000.0);
GJDT_CurQKp = single(1.0);
GJDT_CurQKi = single(20000.0);
GJDT_SquareAmplitude_A = single(0.3);
GJDT_SquareHalfPeriodTicks = int32(10);

source_model = 'green_joint_average_motor_twin_model';
source_model_file = fullfile(script_dir, [source_model '.slx']);
if ~exist(source_model_file, 'file') || strcmp(getenv('GJ_DT_REBUILD_BEFORE_TEST'), '1')
    run(fullfile(script_dir, 'build_green_joint_average_motor_twin_model.m'));

    script_dir = fileparts(mfilename('fullpath'));
    previous_dir = pwd;
    cleanup_dir = onCleanup(@() cd(previous_dir)); %#ok<NASGU>
    cd(script_dir);
    run(fullfile(script_dir, 'setup_green_joint_current_loop_twin.m'));

    GJDT_StopTime = 0.010;
    GJDT_CurDKp = single(1.0);
    GJDT_CurDKi = single(20000.0);
    GJDT_CurQKp = single(1.0);
    GJDT_CurQKi = single(20000.0);
    GJDT_SquareAmplitude_A = single(0.3);
    GJDT_SquareHalfPeriodTicks = int32(10);
end

harness_model = 'green_joint_average_motor_square_wave_harness';
harness_file = fullfile(script_dir, [harness_model '.slx']);

if bdIsLoaded(harness_model)
    close_system(harness_model, 0);
end

if bdIsLoaded(source_model)
    close_system(source_model, 0);
end

if exist(harness_file, 'file')
    delete(harness_file);
end

load_system(source_model_file);
save_system(source_model, harness_file);
close_system(source_model, 0);

load_system(harness_file);
set_param(harness_model, ...
    'StopTime', 'GJDT_StopTime', ...
    'Description', ['Visible 1 kHz current-square harness. ', ...
    'TestSupervisor is simulation-only and is not generated controller code.']);

replace_iq_step_with_stateflow_supervisor(harness_model);
add_visible_test_scopes(harness_model);
sync_average_twin_current_loop_parameters(script_dir);

save_system(harness_model, harness_file);

fprintf('Built green-joint average motor square-wave harness:\n  %s\n', ...
    harness_file);
fprintf('\nRun from MATLAB Desktop:\n');
fprintf('  run(''%s'')\n', fullfile(script_dir, 'setup_green_joint_current_loop_twin.m'));
fprintf('  open_system(''%s'')\n', harness_file);
fprintf('  sim(''%s'')\n', harness_model);

function replace_iq_step_with_stateflow_supervisor(model)
block_path = [model '/iq_ref_step'];
if getSimulinkBlockHandle(block_path) ~= -1
    line_handles = get_param(block_path, 'LineHandles');
    if line_handles.Outport ~= -1
        delete_line(line_handles.Outport);
    end
    delete_block(block_path);
end

add_block('sflib/Chart', [model '/TestSupervisor'], ...
    'Position', [30 105 180 175]);
configure_square_wave_stateflow_chart([model '/TestSupervisor']);

add_line(model, 'TestSupervisor/1', 'iq_ref_to_current/1', 'autorouting', 'on');
end

function configure_square_wave_stateflow_chart(block_path)
rt = sfroot;
chart = rt.find('-isa', 'Stateflow.Chart', 'Path', block_path);
if isempty(chart)
    error('Could not find Stateflow chart: %s', block_path);
end

chart.Name = 'TestSupervisor';
chart.ActionLanguage = 'MATLAB';

delete(chart.find('-isa', 'Stateflow.State'));
delete(chart.find('-isa', 'Stateflow.Transition'));
delete(chart.find('-isa', 'Stateflow.Data'));

iq_ref = Stateflow.Data(chart);
iq_ref.Name = 'iq_ref';
iq_ref.Scope = 'Output';
iq_ref.Port = 1;

scenario_id = Stateflow.Data(chart);
scenario_id.Name = 'scenario_id';
scenario_id.Scope = 'Output';
scenario_id.Port = 2;

low_state = Stateflow.State(chart);
low_state.Name = 'CurrentSquareLow';
low_state.Position = [80 95 170 80];
low_state.LabelString = sprintf(['CurrentSquareLow\n', ...
    'during:\n', ...
    ' iq_ref = -single(0.3);\n', ...
    ' scenario_id = int32(1);']);

high_state = Stateflow.State(chart);
high_state.Name = 'CurrentSquareHigh';
high_state.Position = [330 95 175 80];
high_state.LabelString = sprintf(['CurrentSquareHigh\n', ...
    'during:\n', ...
    ' iq_ref = single(0.3);\n', ...
    ' scenario_id = int32(1);']);

default_transition = Stateflow.Transition(chart);
default_transition.Destination = low_state;

low_to_high = Stateflow.Transition(chart);
low_to_high.Source = low_state;
low_to_high.Destination = high_state;
low_to_high.LabelString = 'after(10,tick)';

high_to_low = Stateflow.Transition(chart);
high_to_low.Source = high_state;
high_to_low.Destination = low_state;
high_to_low.LabelString = 'after(10,tick)';
end

function add_visible_test_scopes(model)
if getSimulinkBlockHandle([model '/IqRef_Iq_Mux']) == -1
    add_block('simulink/Signal Routing/Mux', [model '/IqRef_Iq_Mux'], ...
        'Position', [845 605 850 665], ...
        'Inputs', '2');
end

if getSimulinkBlockHandle([model '/IqRef_Iq_Scope']) == -1
    add_block('simulink/Sinks/Scope', [model '/IqRef_Iq_Scope'], ...
        'Position', [895 610 945 660], ...
        'NumInputPorts', '1');
end

add_line_if_missing(model, 'iq_ref_to_current/1', 'IqRef_Iq_Mux/1');
add_line_if_missing(model, 'iq_to_gj_current/1', 'IqRef_Iq_Mux/2');
add_line_if_missing(model, 'IqRef_Iq_Mux/1', 'IqRef_Iq_Scope/1');

end

function add_line_if_missing(model, src, dst)
try
    add_line(model, src, dst, 'autorouting', 'on');
catch err
    if ~contains(err.message, 'already has a line') ...
            && ~contains(err.message, 'Invalid Simulink object name')
        rethrow(err);
    end
end
end

function sync_average_twin_current_loop_parameters(script_dir)
dictionary_file = fullfile(script_dir, ...
    'green_joint_average_motor_twin_interface.sldd');
if ~exist(dictionary_file, 'file')
    return;
end

dd = Simulink.data.dictionary.open(dictionary_file);
cleanup_dd = onCleanup(@() close(dd));
section = getSection(dd, 'Design Data');

upsert_parameter(section, 'CurDKp', double(evalin('base', 'GJDT_CurDKp')), ...
    'T_GJFloat', 'ExportedGlobal');
upsert_parameter(section, 'CurDKi', double(evalin('base', 'GJDT_CurDKi')), ...
    'T_GJFloat', 'ExportedGlobal');
upsert_parameter(section, 'CurQKp', double(evalin('base', 'GJDT_CurQKp')), ...
    'T_GJFloat', 'ExportedGlobal');
upsert_parameter(section, 'CurQKi', double(evalin('base', 'GJDT_CurQKi')), ...
    'T_GJFloat', 'ExportedGlobal');
upsert_parameter(section, 'VoltageLimitRatio', 0.577, ...
    'T_GJFloat', 'ExportedGlobal');
upsert_parameter(section, 'VoltageModulationRatio', 0.9, ...
    'T_GJFloat', 'ExportedGlobal');
upsert_parameter(section, 'VoltageEpsilon', 0.001, ...
    'T_GJVoltage', 'ExportedGlobal');

saveChanges(dd);
end

function upsert_parameter(section, name, value, data_type, storage_class)
parameter = Simulink.Parameter(value);
parameter.DataType = data_type;
parameter.CoderInfo.StorageClass = storage_class;

entry = find(section, 'Name', name);
if isempty(entry)
    addEntry(section, name, parameter);
else
    setValue(entry(1), parameter);
end
end
