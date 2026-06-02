function result = run_study(cfg)
% Run the cogging position-scan study and keep outputs local to this study.

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

result = run_cogging_position_scan_study(cfg);
result.output_dir = output_dir;
end