function populate_position_p(mdl)
% Populate the Position P subsystem internals.
    path = [mdl '/Position P'];
    % Move outport right
    set_param([path '/w_ref'], 'Position', [400 35 430 49]);

    % Error: pos_ref - theta_meas
    add_block('simulink/Math Operations/Sum', [path '/Sum_pos'], ...
        'Position', [100 30 130 60], 'Inputs', '+-');

    % P gain
    add_block('simulink/Math Operations/Gain', [path '/Kp_pos'], ...
        'Position', [170 32 230 58], 'Gain', 'control.pi_pos.Kp');

    % Saturation: limit speed output
    add_block('simulink/Discontinuities/Saturation', [path '/Sat_wref'], ...
        'Position', [270 32 340 58], ...
        'UpperLimit', 'control.pi_pos.output_limit', ...
        'LowerLimit', '-control.pi_pos.output_limit');

    % Connections
    add_line(path, 'pos_ref/1',     'Sum_pos/1');
    add_line(path, 'theta_meas/1',  'Sum_pos/2');
    add_line(path, 'Sum_pos/1',     'Kp_pos/1');
    add_line(path, 'Kp_pos/1',      'Sat_wref/1');
    add_line(path, 'Sat_wref/1',    'w_ref/1');
end
