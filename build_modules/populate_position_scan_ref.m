function populate_position_scan_ref(mdl)
% Populate the position-reference scan-table generator subsystem.
    path = [mdl '/PosRefScan'];
    set_param([path '/pos_ref'], 'Position', [360 35 390 49]);

    fcn_blk = [path '/position_scan_ref_fcn'];
    add_block('simulink/User-Defined Functions/MATLAB Function', fcn_blk, ...
        'Position', [120 20 290 85]);
    embed_algorithm(fcn_blk, 'position_scan_ref_fcn');

    add_block('simulink/Sources/Constant', [path '/scan_params'], ...
        'Position', [25 110 300 135], ...
        'Value', '[control.pos_scan.start_time, control.pos_scan.hold_time, control.pos_scan.points, control.pos_scan.theta_table(:).'']');

    add_line(path, 't_now/1', 'position_scan_ref_fcn/1');
    add_line(path, 'scan_params/1', 'position_scan_ref_fcn/2');
    add_line(path, 'position_scan_ref_fcn/1', 'pos_ref/1');
end