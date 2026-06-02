function result = reproduce_validation(cfg)
% Reproduce the scan-to-validation chain with artifacts stored in this study.

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

result = reproduce_position_scan_validation(cfg);
result.output_dir = output_dir;
end