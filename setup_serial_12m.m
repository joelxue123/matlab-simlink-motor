%% setup_serial_12m.m — Enable 12 Mbaud serial support for Linux MATLAB
%
% Run this script INSIDE MATLAB after launching with launch_matlab_12m.sh.
% It adds the patched SerialConfiguration to the MATLAB path and verifies
% the LD_PRELOAD shim is active.
%
% Two-layer fix:
%   Layer 1 (MATLAB):  Shadow +system/SerialConfiguration.m catches the
%                       baud rate whitelist error from Serial.p and maps
%                       non-standard rates to 4000000 for the MATLAB API.
%   Layer 2 (Native):  libbaudrate_shim.so (via LD_PRELOAD) intercepts
%                       serial::Serial::setBaudRate() in libmwserialsupport.so
%                       and uses Linux BOTHER/TCSETS2 ioctl to set the
%                       actual baud rate at the kernel level.
%
% For USB CDC virtual COM ports (like F28379D LaunchPad), the baud rate
% is ignored by the USB hardware — data flows at USB speed regardless.

fprintf('\n=== 12 Mbaud Serial Support Setup ===\n\n');

%% 1. Add the patched SerialConfiguration to the path
patchDir = fullfile(fileparts(mfilename('fullpath')), 'serial_baudrate_patch');
if ~isfolder(patchDir)
    error('Patch directory not found: %s', patchDir);
end

% Must be BEFORE the toolbox path
currentPath = path;
if ~contains(currentPath, patchDir)
    addpath(patchDir);
    fprintf('[OK] Added patch directory to MATLAB path:\n     %s\n', patchDir);
else
    fprintf('[OK] Patch directory already on MATLAB path.\n');
end

% Verify the shadow class takes priority
w = which('system.SerialConfiguration');
if contains(w, 'serial_baudrate_patch')
    fprintf('[OK] Shadow SerialConfiguration is active:\n     %s\n', w);
else
    warning(['Shadow SerialConfiguration NOT active. The toolbox version ' ...
        'will be used instead:\n     %s\n' ...
        'Make sure serial_baudrate_patch is BEFORE the toolbox on the path.'], w);
    % Force it to the top
    rmpath(patchDir);
    addpath(patchDir);
    fprintf('     Retrying... now using: %s\n', which('system.SerialConfiguration'));
end

%% 2. Check LD_PRELOAD shim
shimLoaded = false;
try
    ldPreload = getenv('LD_PRELOAD');
    if contains(ldPreload, 'libbaudrate_shim')
        shimLoaded = true;
        fprintf('[OK] LD_PRELOAD shim is active: %s\n', ldPreload);
    end
catch
end

if ~shimLoaded
    fprintf(['\n[WARNING] libbaudrate_shim.so is NOT loaded via LD_PRELOAD.\n' ...
        '  The MATLAB-level patch alone will map 12e6 -> 4000000.\n' ...
        '  This is fine for USB CDC (F28379D), but for real UARTs\n' ...
        '  you need the native shim. Launch MATLAB with:\n' ...
        '    ./launch_matlab_12m.sh\n\n']);
end

%% 3. Summary
fprintf('\n--- Summary ---\n');
fprintf('  MATLAB-level patch:  %s\n', ...
    iff(contains(which('system.SerialConfiguration'), 'serial_baudrate_patch'), ...
    'ACTIVE', 'INACTIVE'));
fprintf('  Native LD_PRELOAD:   %s\n', iff(shimLoaded, 'ACTIVE', 'NOT LOADED'));
fprintf('  Max hardware baud:   unrestricted (with LD_PRELOAD) / 4000000 (without)\n');
fprintf('\nYou can now run host models with BaudRate = 12e6.\n\n');

function result = iff(cond, trueVal, falseVal)
    if cond
        result = trueVal;
    else
        result = falseVal;
    end
end
