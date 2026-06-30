function assert_green_joint_safe_rebuild_environment()
% Guard scripts that delete/recreate .sldd and .slx files.
%
% Rebuilding the generated model while MATLAB Desktop has the same model or
% data dictionary open can leave Simulink/CEF busy in an apparent "update"
% hang. Keep rebuilds in one non-desktop MATLAB process unless explicitly
% overridden for debugging.

if strcmp(getenv('GJ_MBD_ALLOW_UNSAFE_REBUILD'), '1')
    return;
end

script_dir = fileparts(mfilename('fullpath'));
protected_files = { ...
    fullfile(script_dir, 'green_joint_current_loop_model.slx'), ...
    fullfile(script_dir, 'green_joint_current_loop_interface.sldd'), ...
    fullfile(script_dir, 'green_joint_current_loop_model.slxc')};

if usejava('desktop')
    error(['This script rebuilds generated .slx/.sldd files and should not ' ...
        'be run from MATLAB Desktop. Run it from matlab -batch, or set ' ...
        'GJ_MBD_ALLOW_UNSAFE_REBUILD=1 only if you have closed the model ' ...
        'and understand the risk.']);
end

current_pid = feature('getpid');
holders = find_other_matlab_file_holders(protected_files, current_pid);
if isempty(holders)
    return;
end

error(['Another MATLAB process appears to hold generated model artifacts ' ...
    'that this script rebuilds. Close the model/data dictionary in MATLAB ' ...
    'Desktop, or close MATLAB Desktop, then rerun.\n%s'], ...
    strjoin(holders, newline));
end

function holders = find_other_matlab_file_holders(file_names, current_pid)
holders = strings(0, 1);
for i = 1:numel(file_names)
    file_name = file_names{i};
    if ~exist(file_name, 'file')
        continue;
    end

    [status, output] = system(sprintf('lsof -F pc -- %s 2>/dev/null', ...
        shell_quote(file_name)));
    if status ~= 0 || strlength(strtrim(string(output))) == 0
        continue;
    end

    lines = splitlines(strtrim(string(output)));
    pid = NaN;
    command_name = "";
    for j = 1:numel(lines)
        line = lines(j);
        if startsWith(line, "p")
            pid = str2double(extractAfter(line, 1));
        elseif startsWith(line, "c")
            command_name = extractAfter(line, 1);
        end

        if ~isnan(pid) && command_name ~= ""
            if pid ~= current_pid && contains(command_name, "MATLAB")
                holders(end + 1, 1) = sprintf('%s held by PID %d (%s)', ...
                    file_name, pid, command_name); %#ok<AGROW>
            end
            pid = NaN;
            command_name = "";
        end
    end
end
end

function quoted = shell_quote(text)
quoted = ['''' strrep(text, '''', '''"''"''') ''''];
end
