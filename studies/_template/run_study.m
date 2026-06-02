function result = run_study(cfg)
% Template entry for a new study.

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

result = struct();
result.config = cfg;
result.output_dir = output_dir;

error(['Template study is not connected yet. Replace the body of ', mfilename, ...
    ' with your actual experiment entry.']);
end