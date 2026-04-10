function populate_speed_pi(mdl)
% Populate the Speed PI subsystem: discrete PI with anti-windup.
    path = [mdl '/Speed PI'];
    % Move outport right to make room for internal blocks
    set_param([path '/iq_ref'], 'Position', [400 35 430 49]);

    % MATLAB Function block: discrete PI with anti-windup
    fcn_blk = [path '/speed_pi_fcn'];
    add_block('simulink/User-Defined Functions/MATLAB Function', fcn_blk, ...
        'Position', [120 20 280 100]);
    embed_algorithm(fcn_blk, 'speed_pi_fcn');

    % PI parameter vector: [Kp, Ki, Ts, output_limit]
    add_block('simulink/Sources/Constant', [path '/pi_params'], ...
        'Position', [30 120 200 140], ...
        'Value', '[control.pi_speed.Kp, control.pi_speed.Ki, simcfg.Ts_speed, control.pi_speed.output_limit]');

    % Connections
    add_line(path, 'w_ref/1',        'speed_pi_fcn/1');
    add_line(path, 'w_meas/1',       'speed_pi_fcn/2');
    add_line(path, 'pi_params/1',    'speed_pi_fcn/3');
    add_line(path, 'speed_pi_fcn/1', 'iq_ref/1');
end
