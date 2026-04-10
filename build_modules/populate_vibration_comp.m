function populate_vibration_comp(mdl)
% Populate the Vibration Compensation subsystem.
    path = [mdl '/Vibration Compensation'];

    set_param([path '/iq_ref_cmd'], 'Position', [430 35 460 49]);
    set_param([path '/iq_ff'], 'Position', [430 80 460 94]);
    set_param([path '/learn_active'], 'Position', [430 125 460 139]);

    fcn_blk = [path '/vibration_compensator'];
    add_block('simulink/User-Defined Functions/MATLAB Function', fcn_blk, ...
        'Position', [140 25 320 145]);
    embed_algorithm(fcn_blk, 'vibration_compensator');

    add_block('simulink/Sources/Constant', [path '/vib_params'], ...
        'Position', [30 180 260 200], ...
        'Value', ['[control.vib.table_points, control.vib.learning_rate, ' ...
                  'control.vib.phase_advance_deg, control.vib.output_limit, ' ...
                  'control.vib.mean_alpha, control.vib.min_speed_abs, ' ...
                  'control.vib.speed_err_threshold, control.vib.learn_start_time, ' ...
                  'control.vib.enable_learning, control.vib.enable_ff, ' ...
                  'control.vib.ff_enable_time]']);

    add_line(path, 'theta_meas/1', 'vibration_compensator/1');
    add_line(path, 'iq_ref_base/1', 'vibration_compensator/2');
    add_line(path, 'w_ref/1', 'vibration_compensator/3');
    add_line(path, 'w_meas/1', 'vibration_compensator/4');
    add_line(path, 't_now/1', 'vibration_compensator/5');
    add_line(path, 'vib_params/1', 'vibration_compensator/6');
    add_line(path, 'vibration_compensator/1', 'iq_ref_cmd/1');
    add_line(path, 'vibration_compensator/2', 'iq_ff/1');
    add_line(path, 'vibration_compensator/3', 'learn_active/1');
end
