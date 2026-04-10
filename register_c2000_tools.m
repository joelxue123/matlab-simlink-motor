function register_c2000_tools()
% Register TI C2000 third-party tools for MATLAB R2023b on Linux.
% Bypasses c2000setup GUI to write thirdpartytools_glnxa64.xml directly.
%
% Prerequisites:
%   1. Run setup_controlsuite_compat.sh first to create symlink bridge
%   2. Then run this script in MATLAB

%% ---- User paths (edit if your installations differ) ----
cgt_c2000  = '/home/user/software/ti-cgt-c2000_22.6.0.LTS';
cgt_arm    = '/home/user/software/ti-cgt-arm_20.2.7.LTS';
ccs_root   = '/home/user/ti/ccs1220/ccs';
c2000ware  = '/home/user/ti/c2000/C2000Ware_4_03_00_00';

%% ---- Derived paths ----
dss_path        = fullfile(ccs_root, 'ccs_base', 'scripting');
f2806x_dev      = fullfile(c2000ware, 'device_support', 'f2806x');
f2837xd_dev     = fullfile(c2000ware, 'device_support', 'f2837xd');
clamath_path    = fullfile(c2000ware, 'libraries', 'math', 'CLAmath', 'c28');

% ControlSUITE compatibility root (symlink bridge created by shell script)
controlsuite_compat = fullfile(c2000ware, 'controlsuite_compat');

%% ---- Verify paths ----
paths_to_check = {
    cgt_c2000,          'C2000 CGT compiler'
    cgt_arm,            'ARM CGT compiler'
    ccs_root,           'CCS root'
    dss_path,           'Debug Server Scripting'
    c2000ware,          'C2000Ware'
    f2806x_dev,         'F2806x device support'
    f2837xd_dev,        'F2837xD device support'
    clamath_path,       'CLAmath'
};

all_ok = true;
for k = 1:size(paths_to_check, 1)
    if ~isfolder(paths_to_check{k, 1})
        fprintf('[MISSING] %s: %s\n', paths_to_check{k, 2}, paths_to_check{k, 1});
        all_ok = false;
    else
        fprintf('[OK]      %s\n', paths_to_check{k, 2});
    end
end

if ~isfolder(controlsuite_compat)
    fprintf('[MISSING] ControlSUITE compatibility symlinks: %s\n', controlsuite_compat);
    fprintf('         Run setup_controlsuite_compat.sh first.\n');
    all_ok = false;
else
    fprintf('[OK]      ControlSUITE compatibility symlinks\n');
end

if ~all_ok
    error('Some paths are missing. Fix them before registration.');
end

%% ---- Get target folder and registration file path ----
targetFolder = fullfile(matlabroot, 'toolbox', 'c2b', 'tic2000');
fileName = codertarget.target.getThirdPartyToolsRegistrationFileName(targetFolder);

% If the returned path is relative, make it absolute under matlabroot
if ~java.io.File(fileName).isAbsolute()
    fileName_abs = fullfile(matlabroot, '..', fileName);
    % Also write to pwd-relative location as fallback
    fileName_rel = fileName;
else
    fileName_abs = fileName;
    fileName_rel = '';
end

fprintf('\nRegistry file (resolved): %s\n', fileName_abs);

%% ---- Create ThirdPartyToolInfo and register ----
write_registry(fileName_abs, cgt_c2000, cgt_arm, dss_path, ...
    controlsuite_compat, c2000ware, f2806x_dev, f2837xd_dev, clamath_path);

% Also write relative-path copy if the function returned a relative path
if ~isempty(fileName_rel)
    rel_dir = fileparts(fileName_rel);
    if ~isfolder(rel_dir)
        mkdir(rel_dir);
    end
    write_registry(fileName_rel, cgt_c2000, cgt_arm, dss_path, ...
        controlsuite_compat, c2000ware, f2806x_dev, f2837xd_dev, clamath_path);
    fprintf('Also wrote relative copy: %s\n', fileName_rel);
end

fprintf('\nDone. You can now build C2000 models from the Simulink GUI.\n');
end

function write_registry(fileName, cgt_c2000, cgt_arm, dss_path, ...
    controlsuite_compat, c2000ware, f2806x_dev, f2837xd_dev, clamath_path)

    reg_dir = fileparts(fileName);
    if ~isfolder(reg_dir)
        mkdir(reg_dir);
    end

    try
        h = codertarget.thirdpartytools.ThirdPartyToolInfo(fileName, false);
    catch
        h = codertarget.thirdpartytools.ThirdPartyToolInfo();
        h.setDefinitionFileName(fileName);
    end
    h.setName('C2000 Tools');
    h.setTargetName('TI C2000');

    % 1. C2000 CGT compiler
    h.addTool('ToolName', 'Texas Instruments CCS with C2000 Code Generation Tools', ...
        'Category', 'toolchain', ...
        'TokenName', 'CCSINSTALLDIR', ...
        'RootFolder', cgt_c2000);

    % 2. Debug Server Scripting
    h.addTool('ToolName', 'Debug Server Scripting for Texas Instruments CCS', ...
        'Category', 'scripting', ...
        'TokenName', 'CCSSCRIPTINGDIR', ...
        'RootFolder', dss_path);

    % 3. controlSUITE (mapped to compatibility bridge over C2000Ware)
    h.addTool('ToolName', 'Texas Instruments controlSUITE', ...
        'Category', 'other', ...
        'TokenName', 'CONTROLSUITEINSTALLDIR', ...
        'RootFolder', controlsuite_compat);

    % 4. C2000Ware
    h.addTool('ToolName', 'Texas Instruments C2000Ware', ...
        'Category', 'other', ...
        'TokenName', 'C2000WAREINSTALLDIR', ...
        'RootFolder', c2000ware);

    % 5. ARM CGT compiler
    h.addTool('ToolName', 'Texas Instruments CCS with ARM Code Generation Tools', ...
        'Category', 'toolchain', ...
        'TokenName', 'CCSARMINSTALLDIR', ...
        'RootFolder', cgt_arm);

    % 6. F2806x device headers (ControlSUITE-era processor)
    h.addTool('ToolName', 'f2806x C/C++ Source/Header Files', ...
        'Category', 'other', ...
        'TokenName', 'DSP2806x_INSTALLDIR', ...
        'RootFolder', f2806x_dev);

    % 7. F2837xD device headers
    h.addTool('ToolName', 'f2837xd C/C++ Source/Header Files', ...
        'Category', 'other', ...
        'TokenName', 'F2837xD_INSTALLDIR', ...
        'RootFolder', f2837xd_dev);

    % 8. CLAmath
    h.addTool('ToolName', 'CLAmath Library Files', ...
        'Category', 'other', ...
        'TokenName', 'CLAMATHLIB_INSTALLDIR', ...
        'RootFolder', clamath_path);

    % Make file writable if it already exists
    if isfile(fileName)
        [status, attrib] = fileattrib(fileName);
        if status && ~attrib.UserWrite
            fileattrib(fileName, '+w');
        end
    end

    h.register();
    fprintf('Registered tools to: %s\n', fileName);
end
