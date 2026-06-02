function result = run_discretization_compare(varargin)
%RUN_DISCRETIZATION_COMPARE Build, simulate, and plot three discretizations.
%   result = run_discretization_compare()
%   result = run_discretization_compare('Ts', 5e-4, 'a', 120)

mdl_path = build_discretization_compare_model(varargin{:});
[script_dir, mdl_name, ~] = fileparts(mdl_path);

sim_out = sim(mdl_name, 'ReturnWorkspaceOutputs', 'on');

y_cont = sim_out.get('y_cont');
y_fe = sim_out.get('y_fe');
y_be = sim_out.get('y_be');
y_zoh = sim_out.get('y_zoh');
disc_cfg = evalin('base', 'disc_cfg');

figure('Name', 'Three Discretization Comparison', 'Color', 'w');
plot(y_cont.time, y_cont.signals.values, 'k-', 'LineWidth', 1.5);
hold on;
stairs(y_fe.time, y_fe.signals.values, 'r--', 'LineWidth', 1.2);
stairs(y_be.time, y_be.signals.values, 'b-.', 'LineWidth', 1.2);
stairs(y_zoh.time, y_zoh.signals.values, 'g-', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('State x');
title(sprintf('x_dot = -a x + b u, a = %.3g, b = %.3g, Ts = %.3g s', ...
    disc_cfg.a, disc_cfg.b, disc_cfg.Ts));
legend({'Continuous', 'Forward Euler', 'Backward Euler', 'Exact ZOH'}, ...
    'Location', 'best');

result = struct();
result.model_path = mdl_path;
result.config = disc_cfg;
result.y_cont = y_cont;
result.y_fe = y_fe;
result.y_be = y_be;
result.y_zoh = y_zoh;
result.rmse_fe = localRmse(y_cont, y_fe);
result.rmse_be = localRmse(y_cont, y_be);
result.rmse_zoh = localRmse(y_cont, y_zoh);

fprintf('\n=== Three discretization comparison ===\n');
fprintf('Model: %s\n', mdl_name);
fprintf('Forward Euler RMSE : %.9g\n', result.rmse_fe);
fprintf('Backward Euler RMSE: %.9g\n', result.rmse_be);
fprintf('Exact ZOH RMSE     : %.9g\n', result.rmse_zoh);

open_system(mdl_name);
open_system([mdl_name '/Comparison Scope']);

if ~isempty(script_dir)
    cd(script_dir);
end
end

function value = localRmse(reference_sig, discrete_sig)
ref_time = reference_sig.time(:);
ref_value = reference_sig.signals.values(:);
disc_time = discrete_sig.time(:);
disc_value = discrete_sig.signals.values(:);

interp_ref = interp1(ref_time, ref_value, disc_time, 'linear', 'extrap');
value = sqrt(mean((interp_ref - disc_value) .^ 2));
end