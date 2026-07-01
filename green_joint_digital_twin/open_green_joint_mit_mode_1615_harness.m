%% Open visible green-joint 1615 MIT mode harness

script_dir = fileparts(mfilename('fullpath'));
model_file = fullfile(script_dir, 'green_joint_mit_mode_1615_harness.slx');

run(fullfile(script_dir, 'setup_green_joint_mit_mode_1615_harness.m'));

if ~exist(model_file, 'file')
    run(fullfile(script_dir, 'build_green_joint_mit_mode_1615_harness.m'));
end

load_system(model_file);
open_system('green_joint_mit_mode_1615_harness');
