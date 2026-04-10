function populate_kalman_speed(mdl)
% Populate the Kalman Speed Estimator subsystem.
    path = [mdl '/Kalman Speed'];

    % Move outports right for room
    set_param([path '/w_kf'], 'Position', [400 35 430 49]);
    set_param([path '/theta_kf'], 'Position', [400 80 430 94]);

    % MATLAB Function block
    fcn_blk = [path '/kalman_speed_estimator'];
    add_block('simulink/User-Defined Functions/MATLAB Function', fcn_blk, ...
        'Position', [120 30 280 100]);
    embed_algorithm(fcn_blk, 'kalman_speed_estimator');

    % Kalman parameters: [Ts, q_theta, q_omega, r_theta]
    add_block('simulink/Sources/Constant', [path '/kf_params'], ...
        'Position', [30 120 200 140], ...
        'Value', '[simcfg.Ts_ctrl, control.kf.q_theta, control.kf.q_omega, control.kf.r_theta]');

    % Connections
    add_line(path, 'theta_meas/1',              'kalman_speed_estimator/1');
    add_line(path, 'kf_params/1',               'kalman_speed_estimator/2');
    add_line(path, 'kalman_speed_estimator/1',   'w_kf/1');
    add_line(path, 'kalman_speed_estimator/2',   'theta_kf/1');
end
