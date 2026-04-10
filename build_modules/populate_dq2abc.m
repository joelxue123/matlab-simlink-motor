function populate_dq2abc(mdl)
% Populate the dq-to-abc subsystem: inverse Park + Clarke + SVPWM + duty.
    path = [mdl '/dq to abc'];

    % Add MATLAB Function block
    fcn_blk = [path '/dq2abc_fcn'];
    add_block('simulink/User-Defined Functions/MATLAB Function', fcn_blk, ...
        'Position', [100 30 220 120]);
    embed_algorithm(fcn_blk, 'dq2abc_fcn');

    % Connect inports -> MATLAB Function -> outports
    add_line(path, 'vd_ref/1',  'dq2abc_fcn/1');
    add_line(path, 'vq_ref/1',  'dq2abc_fcn/2');
    add_line(path, 'theta_e/1', 'dq2abc_fcn/3');
    add_line(path, 'Vdc/1',     'dq2abc_fcn/4');
    add_line(path, 'dq2abc_fcn/1', 'da/1');
    add_line(path, 'dq2abc_fcn/2', 'db/1');
    add_line(path, 'dq2abc_fcn/3', 'dc/1');
end
