function populate_current_ref(mdl)
% Populate the Current Ref subsystem: id_ref=0, iq saturation.
    path = [mdl '/Current Ref'];

    % id_ref = 0 (MTPA for SPMSM)
    add_block('simulink/Sources/Constant', [path '/Zero'], ...
        'Position', [120 30 170 50], ...
        'Value', '0');

    % Saturate iq_ref_cmd to current limit
    add_block('simulink/Discontinuities/Saturation', [path '/Sat_iq'], ...
        'Position', [120 72 190 98], ...
        'UpperLimit', 'control.iq_ref_limit', ...
        'LowerLimit', '-control.iq_ref_limit');

    % Connections
    add_line(path, 'Zero/1',       'id_ref/1');
    add_line(path, 'iq_ref_cmd/1', 'Sat_iq/1');
    add_line(path, 'Sat_iq/1',     'iq_ref/1');
end
