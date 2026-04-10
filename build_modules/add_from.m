function add_from(mdl, tag, pos)
% Add a From block referencing the given Goto tag.
    blk = [mdl '/From_' tag];
    add_block('simulink/Signal Routing/From', blk, ...
        'Position', pos, 'GotoTag', tag);
end
