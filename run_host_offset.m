%% run_host_offset.m — Launch host model for QEP offset computation
%  Fixes BaudRate=12e6 → 921600 (standard rate) before running.

exDir = '/home/user/Documents/MATLAB/Examples/R2023b/mcb/QuadratureEncoderOffsetExample';
cd(exDir);
fprintf('Working directory: %s\n', pwd);

% Define workspace variables required by the host model.
% Must use assignin('base',...) because Simulink variant conditions
% evaluate in the base workspace, not the caller's workspace.
dataType = 'single';
PWM_frequency = 20000;                  % Hz
T_pwm = 1/PWM_frequency;               % sec
Ts    = T_pwm;                          % sec — sample time
motor.polePairs  = 4;                   % pole pairs (adjust to your motor)
motor.calibSpeed = 60;                  % RPM during calibration

assignin('base', 'dataType', dataType);
assignin('base', 'PWM_frequency', PWM_frequency);
assignin('base', 'T_pwm', T_pwm);
assignin('base', 'Ts', Ts);
assignin('base', 'motor', motor);

fprintf('Base workspace variables set: dataType=''%s'', Ts=%g, motor.polePairs=%d\n', ...
    dataType, Ts, motor.polePairs);

% Open host model
mdl = 'mcb_pmsm_host_offsetComputation_f28379d';
open_system(mdl);
fprintf('Model %s opened.\n', mdl);

% --- Fix BaudRate: change 12e6 to 921600 (standard rate) ---
% USB CDC virtual COM port ignores baud rate anyway, so any valid rate works.
blk = [mdl '/Host Serial Setup'];
try
    oldBaud = get_param(blk, 'BaudRate');
    fprintf('Current BaudRate: %s\n', oldBaud);
    set_param(blk, 'BaudRate', '921600');
    fprintf('BaudRate changed to 921600 (standard rate for Linux).\n');
catch e
    % Try alternative block paths
    fprintf('Block not found at "%s", searching...\n', blk);
    allBlks = find_system(mdl, 'BlockType', 'SubSystem', 'ReferenceBlock', 'instrumentseriallib/Serial Configuration');
    if isempty(allBlks)
        allBlks = find_system(mdl, 'MaskType', 'Serial Configuration');
    end
    if ~isempty(allBlks)
        blk = allBlks{1};
        fprintf('Found Serial Configuration block: %s\n', blk);
        set_param(blk, 'BaudRate', '921600');
        fprintf('BaudRate changed to 921600.\n');
    else
        warning('Could not find Serial Configuration block. Proceeding anyway.');
    end
end

% Run the model
fprintf('\n=== Starting simulation ===\n');
fprintf('Make sure the F28379D target is connected, powered on,\n');
fprintf('and running the mcb_pmsm_qep_offset_f28379d firmware.\n');
fprintf('Press Ctrl+C in MATLAB to stop.\n\n');
set_param(mdl, 'SimulationCommand', 'start');
