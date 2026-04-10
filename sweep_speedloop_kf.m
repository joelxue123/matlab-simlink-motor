function results = sweep_speedloop_kf()
% Sweep Kalman parameters and pick a practical optimum for the speed-loop test.
%
% The score favors low RMSE while penalizing lag and oversensitivity.

motor_control_params;

q_theta_list = [1e-8, 1e-7, 1e-6, 1e-5];
q_omega_list = [0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0];
r_theta = control.kf.r_theta;
test_noise_var = control.kf.test_noise_var;

results = struct('q_theta', {}, 'q_omega', {}, 'rmse', {}, 'mae', {}, ...
    'max_abs_err', {}, 'lag_time_s', {}, 'score', {});
index = 0;

fprintf('\n=== Sweeping KF parameters ===\n');
for i = 1:numel(q_theta_list)
    for j = 1:numel(q_omega_list)
        q_theta = q_theta_list(i);
        q_omega = q_omega_list(j);
        metrics = evaluate_speedloop_kf_test(q_theta, q_omega, r_theta, test_noise_var);

        % Composite score: prioritize RMSE, then penalize time lag and spikes.
        score = metrics.rmse + 8000 * abs(metrics.lag_time_s) + 0.03 * metrics.max_abs_err;

        index = index + 1;
        results(index).q_theta = q_theta;
        results(index).q_omega = q_omega;
        results(index).rmse = metrics.rmse;
        results(index).mae = metrics.mae;
        results(index).max_abs_err = metrics.max_abs_err;
        results(index).lag_time_s = metrics.lag_time_s;
        results(index).score = score;

        fprintf('q_theta=%-8.1e q_omega=%-6.3g RMSE=%7.4f lag=%9.2eus score=%7.4f\n', ...
            q_theta, q_omega, metrics.rmse, metrics.lag_time_s * 1e6, score);
    end
end

scores = [results.score];
[~, best_idx] = min(scores);
best = results(best_idx);

fprintf('\n=== Best KF tuning ===\n');
fprintf('q_theta    = %.3e\n', best.q_theta);
fprintf('q_omega    = %.3g\n', best.q_omega);
fprintf('RMSE       = %.6f rad/s\n', best.rmse);
fprintf('MAE        = %.6f rad/s\n', best.mae);
fprintf('Max |err|  = %.6f rad/s\n', best.max_abs_err);
fprintf('Lag time   = %.6e s\n', best.lag_time_s);
fprintf('Score      = %.6f\n', best.score);

assignin('base', 'kf_sweep_results', results);
assignin('base', 'kf_best_result', best);
end
