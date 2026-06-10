%% Run smoke tests for reusable motor-control MBD modules

clear; clc;

script_dir = fileparts(mfilename('fullpath'));
modules_dir = fileparts(script_dir);
repo_dir = fileparts(modules_dir);

test_scripts = { ...
    fullfile(repo_dir, 'motor_current_pi_mbd', 'run_current_pi_smoke_test.m'); ...
    fullfile(repo_dir, 'motor_current_pi_mbd', 'run_current_pi_saturation_test.m'); ...
    fullfile(repo_dir, 'motor_speed_pi_mbd', 'run_speed_pi_smoke_test.m'); ...
    fullfile(repo_dir, 'motor_clarke_park_struct', 'run_motor_clarke_park_function_test.m'); ...
    fullfile(repo_dir, 'motor_float_open_loop_mbd', 'run_open_loop_smoke_test.m'); ...
    fullfile(repo_dir, 'motor_speed_current_loop_mbd', 'run_speed_current_loop_smoke_test.m')};

for i = 1:numel(test_scripts)
    fprintf('\n[%d/%d] Running:\n  %s\n', i, numel(test_scripts), test_scripts{i});
    if ~exist(test_scripts{i}, 'file')
        error('Missing test script: %s', test_scripts{i});
    end
    run(test_scripts{i});
end

fprintf('\nAll reusable motor-control module smoke tests passed.\n');
