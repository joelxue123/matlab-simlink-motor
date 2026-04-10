function populate_abc2dq(mdl)
% Populate the abc-to-dq subsystem: Clarke + Park transform.
    path = [mdl '/abc to dq'];
    % Move outports right for room
    set_param([path '/id_meas'], 'Position', [400 35 430 49]);
    set_param([path '/iq_meas'], 'Position', [400 80 430 94]);

    % MATLAB Function block: Clarke + Park transform
    fcn_blk = [path '/abc2dq_fcn'];
    add_block('simulink/User-Defined Functions/MATLAB Function', fcn_blk, ...
        'Position', [120 30 280 190]);
    embed_algorithm(fcn_blk, 'abc2dq_fcn');

    % Connect inports -> MATLAB Function -> outports
    add_line(path, 'ia/1',         'abc2dq_fcn/1');
    add_line(path, 'ib/1',         'abc2dq_fcn/2');
    add_line(path, 'ic/1',         'abc2dq_fcn/3');
    add_line(path, 'theta_e/1',    'abc2dq_fcn/4');
    add_line(path, 'abc2dq_fcn/1', 'id_meas/1');
    add_line(path, 'abc2dq_fcn/2', 'iq_meas/1');
end
