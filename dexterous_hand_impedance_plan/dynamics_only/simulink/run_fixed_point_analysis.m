%% Run fixed-point analysis without modifying the model
% This script rebuilds the model, runs a baseline simulation, and opens the
% Fixed-Point Tool on the Controller subsystem for range analysis only.

clear; clc;

script_dir = fileparts(mfilename('fullpath'));

run(fullfile(script_dir, 'build_single_joint_dynamics_model.m'));

% The build script clears the workspace, so recompute local paths here.
script_dir = fileparts(mfilename('fullpath'));
model = 'single_joint_dynamics_control';
model_file = fullfile(script_dir, [model '.slx']);
controller_path = [model '/Controller'];

load_system(model_file);

set_param(model, ...
    'SimulationCommand', 'update', ...
    'SimulationMode', 'normal');

fprintf('Running baseline simulation for fixed-point analysis...\n');
sim_out = sim(model, 'CaptureErrors', 'on');
if ~isempty(sim_out.ErrorMessage)
    error('Baseline simulation failed: %s', sim_out.ErrorMessage);
end

fprintf('Opening Fixed-Point Tool for analysis only:\n  %s\n', controller_path);
fprintf('No proposed data types are accepted by this script.\n');
fxptdlg(controller_path);

fprintf('\nNext steps in Fixed-Point Tool:\n');
fprintf('  1. Use range collection results from the completed baseline run.\n');
fprintf('  2. Review SimMin/SimMax and ProposedDT manually.\n');
fprintf('  3. Keep authoritative data types in MATLAB scripts, not in GUI edits.\n');