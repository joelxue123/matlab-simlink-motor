%% Open the visible Stateflow 1 kHz current-square harness
%
% Use this script from MATLAB Desktop when you want to inspect or manually run
% the 1 kHz iq square-wave test model.
%
% The base model green_joint_average_motor_twin_model.slx is only the V1
% physical twin. The visible test-state logic is in:
%   green_joint_average_motor_square_wave_harness.slx/TestSupervisor

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
cd(script_dir);

run(fullfile(script_dir, 'setup_green_joint_current_loop_twin.m'));

model = 'green_joint_average_motor_square_wave_harness';
model_file = fullfile(script_dir, [model '.slx']);

if ~exist(model_file, 'file')
    run(fullfile(script_dir, 'build_green_joint_average_motor_square_wave_harness.m'));
    cd(script_dir);
    run(fullfile(script_dir, 'setup_green_joint_current_loop_twin.m'));
end

if ~bdIsLoaded(model)
    load_system(model_file);
end

open_system(model);

try
    open_system([model '/TestSupervisor']);
catch err
    warning('Could not open TestSupervisor block: %s', err.message);
end

rt = sfroot;
chart = rt.find('-isa', 'Stateflow.Chart', 'Path', [model '/TestSupervisor']);
if isempty(chart)
    warning('Stateflow TestSupervisor was not found in %s.', model);
else
    states = chart(1).find('-isa', 'Stateflow.State');
    fprintf('\nOpened visible 1 kHz current-square harness:\n');
    fprintf('  model: %s\n', model_file);
    fprintf('  Stateflow chart: %s/TestSupervisor\n', model);
    fprintf('  states:');
    for idx = 1:numel(states)
        fprintf(' %s', states(idx).Name);
    end
    fprintf('\n\n');
end

fprintf('Manual run commands:\n');
fprintf('  sim_result = sim(''%s'', ''ReturnWorkspaceOutputs'', ''on'');\n', model);
fprintf('  open_system(''%s/IqRef_Iq_Scope'');\n', model);
fprintf('\nBatch verification command:\n');
fprintf('  matlab -batch "run(''%s'')"\n', ...
    fullfile(script_dir, 'run_green_joint_average_motor_square_wave_harness_test.m'));
