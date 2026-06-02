function result = run_effect_demo()
% Run the baseline vibration-compensation demo with local outputs.

study_dir = fileparts(mfilename('fullpath'));
output_dir = fullfile(study_dir, 'outputs');
if exist(output_dir, 'dir') ~= 7
    mkdir(output_dir);
end

init_project_paths(study_dir);

old_dir = pwd;
cleanup = onCleanup(@() cd(old_dir));
cd(output_dir);

result = show_vibration_comp_effect();
result.output_dir = output_dir;
end