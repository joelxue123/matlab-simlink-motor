function populate_current_pi(mdl)
% Populate the Current PI subsystem: dual PI + cross-coupling decoupling.
    path = [mdl '/Current PI'];

    % Move outports right for room
    set_param([path '/vd_ref'], 'Position', [460 35 490 49]);
    set_param([path '/vq_ref'], 'Position', [460 80 490 94]);

    % MATLAB Function block: dual PI + cross-coupling decoupling
    fcn_blk = [path '/current_pi_fcn'];
    add_block('simulink/User-Defined Functions/MATLAB Function', fcn_blk, ...
        'Position', [120 20 320 240]);
    embed_algorithm(fcn_blk, 'current_pi_fcn');

    % PI parameter vector: [Kp, Ki, Ts, output_limit]
    add_block('simulink/Sources/Constant', [path '/pi_params'], ...
        'Position', [30 260 200 280], ...
        'Value', '[control.pi_id.Kp, control.pi_id.Ki, simcfg.Ts_ctrl, control.pi_id.output_limit]');

    % Motor parameter vector: [Ld, Lq, psi_f]
    add_block('simulink/Sources/Constant', [path '/motor_params'], ...
        'Position', [30 300 200 320], ...
        'Value', '[motor.Ld, motor.Lq, motor.psi_f]');

    % Connect subsystem inports to MATLAB Function
    add_line(path, 'id_ref/1',       'current_pi_fcn/1');
    add_line(path, 'iq_ref/1',       'current_pi_fcn/2');
    add_line(path, 'id_meas/1',      'current_pi_fcn/3');
    add_line(path, 'iq_meas/1',      'current_pi_fcn/4');
    add_line(path, 'omega_e/1',      'current_pi_fcn/5');
    add_line(path, 'pi_params/1',    'current_pi_fcn/6');
    add_line(path, 'motor_params/1', 'current_pi_fcn/7');

    % Connect MATLAB Function outputs to outports
    add_line(path, 'current_pi_fcn/1', 'vd_ref/1');
    add_line(path, 'current_pi_fcn/2', 'vq_ref/1');
end
