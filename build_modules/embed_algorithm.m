function embed_algorithm(block_path, algorithm_name)
% Embed a standalone algorithm .m file into a Simulink MATLAB Function block.
%   block_path     - Full Simulink path to the MATLAB Function block
%   algorithm_name - Filename without .m (e.g., 'speed_pi_fcn')
%
% The algorithm file is resolved from the ../algorithms/ directory
% relative to this file's location.
    algo_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'algorithms');
    script = fileread(fullfile(algo_dir, [algorithm_name '.m']));
    rt = sfroot;
    chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', block_path);
    chart.Script = script;
end
