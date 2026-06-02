function populate_position_chirp_ref(mdl)
% Populate the position-reference chirp generator subsystem.
    path = [mdl '/PosRefChirp'];
    set_param([path '/pos_ref'], 'Position', [360 35 390 49]);

    fcn_blk = [path '/position_chirp_ref_fcn'];
    add_block('simulink/User-Defined Functions/MATLAB Function', fcn_blk, ...
        'Position', [120 20 290 85]);
    embed_algorithm(fcn_blk, 'position_chirp_ref_fcn');

    add_block('simulink/Sources/Constant', [path '/chirp_params'], ...
        'Position', [25 110 245 135], ...
        'Value', '[control.pos_chirp.amplitude_rad, control.pos_chirp.f0_hz, control.pos_chirp.f1_hz, control.pos_chirp.start_time, control.pos_chirp.duration, control.pos_chirp.offset_rad]');

    add_line(path, 't_now/1', 'position_chirp_ref_fcn/1');
    add_line(path, 'chirp_params/1', 'position_chirp_ref_fcn/2');
    add_line(path, 'position_chirp_ref_fcn/1', 'pos_ref/1');
end