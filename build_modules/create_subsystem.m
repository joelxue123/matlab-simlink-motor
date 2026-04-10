function create_subsystem(mdl, name, pos, inports, outports)
% Create a subsystem with named inports and outports, clearing default contents.
    path = [mdl '/' name];
    add_block('simulink/Ports & Subsystems/Subsystem', path, 'Position', pos);

    % Remove default contents
    lines = find_system(path, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line');
    if ~isempty(lines), delete_line(lines); end
    blks = find_system(path, 'SearchDepth', 1, 'Type', 'Block');
    for k = 2:numel(blks), delete_block(blks{k}); end

    for k = 1:numel(inports)
        p = [path '/' inports{k}];
        add_block('simulink/Sources/In1', p, ...
            'Position', [30 35+45*(k-1) 60 49+45*(k-1)]);
        set_param(p, 'Port', num2str(k));
    end
    for k = 1:numel(outports)
        p = [path '/' outports{k}];
        add_block('simulink/Sinks/Out1', p, ...
            'Position', [260 35+45*(k-1) 290 49+45*(k-1)]);
        set_param(p, 'Port', num2str(k));
    end
end
