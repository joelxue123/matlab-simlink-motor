function populate_position_pidreg3(mdl)
% Populate the position-loop PIDREG3 subsystem internals.
    path = [mdl '/Position PID'];
    set_param([path '/w_ref'], 'Position', [400 35 430 49]);
    set_param([path '/ui_pos'], 'Position', [400 80 430 94]);

    fcn_blk = [path '/position_pidreg3_fcn'];
    add_block('simulink/User-Defined Functions/MATLAB Function', fcn_blk, ...
        'Position', [120 20 290 95]);
    embed_algorithm(fcn_blk, 'position_pidreg3_fcn');

    add_block('simulink/Sources/Constant', [path '/pid_params'], ...
        'Position', [30 120 240 145], ...
        'Value', '[control.pid_pos.Kp, control.pid_pos.Ki, control.pid_pos.Kc, control.pid_pos.output_limit]');

    add_line(path, 'pos_ref/1', 'position_pidreg3_fcn/1');
    add_line(path, 'theta_meas/1', 'position_pidreg3_fcn/2');
    add_line(path, 'pid_params/1', 'position_pidreg3_fcn/3');
    add_line(path, 'position_pidreg3_fcn/1', 'w_ref/1');
    add_line(path, 'position_pidreg3_fcn/2', 'ui_pos/1');
end