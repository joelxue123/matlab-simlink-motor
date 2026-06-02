function result = run_study(cfg)
% Run the cogging torque-compensation study with local outputs.

if nargin < 1
    cfg = struct();
end

study_dir = fileparts(mfilename('fullpath'));
output_dir = fullfile(study_dir, 'outputs');
if exist(output_dir, 'dir') ~= 7
    mkdir(output_dir);
end

init_project_paths(study_dir);

old_dir = pwd;
cleanup = onCleanup(@() cd(old_dir));
cd(output_dir);

result = run_cogging_torque_comp_study(cfg);
result.output_dir = output_dir;
end