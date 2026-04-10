function populate_vibration_ff_lookup(mdl)
% Populate the offline fixed vibration feedforward lookup subsystem.
    path = [mdl '/Vibration FF Lookup'];

    set_param([path '/iq_ref_cmd'], 'Position', [430 35 460 49]);
    set_param([path '/iq_ff'], 'Position', [430 80 460 94]);
    set_param([path '/learn_active'], 'Position', [430 125 460 139]);

    fcn_blk = [path '/vibration_ff_lookup'];
    add_block('simulink/User-Defined Functions/MATLAB Function', fcn_blk, ...
        'Position', [140 25 320 145]);
    embed_algorithm(fcn_blk, 'vibration_ff_lookup');

    add_block('simulink/Sources/Constant', [path '/ff_table'], ...
        'Position', [20 180 110 200], ...
        'Value', 'control.vib.ff_table');
    add_block('simulink/Sources/Constant', [path '/ff_params'], ...
        'Position', [150 180 320 200], ...
        'Value', ['[control.vib.table_points, control.vib.phase_advance_deg, ' ...
                  'control.vib.output_limit, control.vib.enable_ff, ' ...
                  'control.vib.ff_enable_time]']);

    add_line(path, 'theta_meas/1', 'vibration_ff_lookup/1');
    add_line(path, 'iq_ref_base/1', 'vibration_ff_lookup/2');
    add_line(path, 't_now/1', 'vibration_ff_lookup/3');
    add_line(path, 'ff_table/1', 'vibration_ff_lookup/4');
    add_line(path, 'ff_params/1', 'vibration_ff_lookup/5');
    add_line(path, 'vibration_ff_lookup/1', 'iq_ref_cmd/1');
    add_line(path, 'vibration_ff_lookup/2', 'iq_ff/1');
    add_line(path, 'vibration_ff_lookup/3', 'learn_active/1');
end
