function result = check_position_pidreg3_ui_current()
% Reconstruct the position-loop Ui signal for the current project settings.

sim_result = run_position_pidreg3_test(struct('plot_results', false));
motor_control_params;

t_pos = (0:simcfg.Ts_pos:simcfg.stop_time).';
pos_ref = interp1(sim_result.time, sim_result.pos_ref, t_pos, 'previous', 'extrap');
theta_meas = interp1(sim_result.time, sim_result.pos, t_pos, 'previous', 'extrap');

ui = 0;
out_pre_sat = 0;
ui_log = zeros(size(t_pos));
out_pre_sat_log = zeros(size(t_pos));
w_ref_log = zeros(size(t_pos));

out_max = control.pid_pos.output_limit;
out_min = -out_max;

for sample_index = 1:numel(t_pos)
    err = pos_ref(sample_index) - theta_meas(sample_index);
    up = control.pid_pos.Kp * err;

    if out_pre_sat > out_max
        if err < 0
            ui = ui + control.pid_pos.Ki * err;
        else
            ui = ui + control.pid_pos.Kc * (out_max - out_pre_sat);
        end
    elseif out_pre_sat < out_min
        if err > 0
            ui = ui + control.pid_pos.Ki * err;
        else
            ui = ui + control.pid_pos.Kc * (out_min - out_pre_sat);
        end
    else
        ui = ui + control.pid_pos.Ki * err;
    end

    ui = min(out_max, max(out_min, ui));
    out_pre_sat = up + ui;
    w_ref = min(out_max, max(out_min, out_pre_sat));

    ui_log(sample_index) = ui;
    out_pre_sat_log(sample_index) = out_pre_sat;
    w_ref_log(sample_index) = w_ref;
end

result = struct();
result.t_pos = t_pos;
result.ui = ui_log;
result.out_pre_sat = out_pre_sat_log;
result.w_ref = w_ref_log;
result.max_abs_ui = max(abs(ui_log));
result.final_ui = ui_log(end);
result.nonzero_samples = nnz(abs(ui_log) > 1e-12);

fprintf('\nCurrent project position-loop Ui check\n');
fprintf('  max |Ui|        = %.12f\n', result.max_abs_ui);
fprintf('  final Ui        = %.12f\n', result.final_ui);
fprintf('  nonzero samples = %d\n', result.nonzero_samples);
fprintf('  first 20 Ui     = ');
disp(ui_log(1:min(20, numel(ui_log)))');
end