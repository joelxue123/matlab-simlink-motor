function populate_position_sine_ref(mdl)
% Populate the position-reference sinusoidal generator subsystem.
    path = [mdl '/PosRefSine'];
    set_param([path '/pos_ref'], 'Position', [360 35 390 49]);

    fcn_blk = [path '/position_sine_ref_fcn'];
    add_block('simulink/User-Defined Functions/MATLAB Function', fcn_blk, ...
        'Position', [120 20 290 85]);
    embed_algorithm(fcn_blk, 'position_sine_ref_fcn');

    add_block('simulink/Sources/Constant', [path '/sine_params'], ...
        'Position', [25 110 245 135], ...
        'Value', '[control.pos_sine.amplitude_rad, control.pos_sine.freq_hz, control.pos_sine.start_time, control.pos_sine.offset_rad]');

    add_line(path, 't_now/1', 'position_sine_ref_fcn/1');
    add_line(path, 'sine_params/1', 'position_sine_ref_fcn/2');
    add_line(path, 'position_sine_ref_fcn/1', 'pos_ref/1');
end