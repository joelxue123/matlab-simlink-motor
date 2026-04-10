function add_goto(mdl, tag, pos, src_port)
% Add a Goto block with global visibility and optionally connect a source port.
    blk = [mdl '/Goto_' tag];
    add_block('simulink/Signal Routing/Goto', blk, ...
        'Position', pos, 'GotoTag', tag, 'TagVisibility', 'global');
    if nargin >= 4 && ~isempty(src_port)
        add_line(mdl, src_port, ['Goto_' tag '/1']);
    end
end
