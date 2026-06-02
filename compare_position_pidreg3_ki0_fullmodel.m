function result = compare_position_pidreg3_ki0_fullmodel()
% Compare Ki = 0 cases on the full average_inverter_foc model.

cases = [
    struct('name', 'Ki0_KcActive', 'overrides', struct('pid_pos_Ki', 0.0, 'pid_pos_Kc', 0.5, 'plot_results', false)), ...
    struct('name', 'Ki0_KcZero', 'overrides', struct('pid_pos_Ki', 0.0, 'pid_pos_Kc', 0.0, 'plot_results', false)) ...
    ];

result = struct();
for case_index = 1:numel(cases)
    case_def = cases(case_index);
    fprintf('\nRunning full-model case: %s\n', case_def.name);
    result.(case_def.name) = run_position_pidreg3_test(case_def.overrides);
end

fprintf('\nFull-model delta summary\n');
fprintf('  final position error delta = %.6f rad\n', ...
    result.Ki0_KcActive.final_position_error - result.Ki0_KcZero.final_position_error);
fprintf('  overshoot delta            = %.6f rad\n', ...
    result.Ki0_KcActive.overshoot_rad - result.Ki0_KcZero.overshoot_rad);
fprintf('  settling time delta        = %.6f s\n', ...
    result.Ki0_KcActive.settling_time_s - result.Ki0_KcZero.settling_time_s);
end