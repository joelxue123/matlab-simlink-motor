function prepare_c2000_gui_build_linux(example_folder)
% Prepare the current Linux MATLAB session for GUI builds of C2000 examples.

if nargin < 1 || strlength(string(example_folder)) == 0
    example_folder = pwd;
end

example_folder = char(string(example_folder));

if ~isfolder(example_folder)
    error('Example or model folder does not exist: %s', example_folder);
end

fix_controlsuite_registry_linux(example_folder);
cd(example_folder);

fprintf('\nGUI build preparation finished.\n');
fprintf('Active folder is now:\n%s\n', pwd);
fprintf(['Next step: keep this MATLAB session open, open the model in Simulink, ', ...
    'and click Build from the GUI.\n']);
end