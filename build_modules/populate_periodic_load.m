function populate_periodic_load(mdl)
% Populate the Periodic Load subsystem.
    path = [mdl '/Periodic Load'];

    set_param([path '/T_load'], 'Position', [380 35 410 49]);

    fcn_blk = [path '/periodic_load_torque'];
    add_block('simulink/User-Defined Functions/MATLAB Function', fcn_blk, ...
        'Position', [120 20 280 80]);
    embed_algorithm(fcn_blk, 'periodic_load_torque');

    add_block('simulink/Sources/Constant', [path '/load_params'], ...
        'Position', [20 120 260 140], ...
        'Value', ['[control.vib.load_base_torque, control.vib.load_amp1, ' ...
                  'control.vib.load_harmonic1, control.vib.load_phase1_deg, ' ...
                  'control.vib.load_amp2, control.vib.load_harmonic2, ' ...
                  'control.vib.load_phase2_deg]']);

    add_line(path, 'theta_meas/1', 'periodic_load_torque/1');
    add_line(path, 'load_params/1', 'periodic_load_torque/2');
    add_line(path, 'periodic_load_torque/1', 'T_load/1');
end
