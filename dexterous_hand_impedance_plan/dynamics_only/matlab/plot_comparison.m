function plot_comparison(results, controllers, test_name, results_dir)
%PLOT_COMPARISON Plot q, error, torque, load and current for one test.

fig = figure('Name', test_name, 'Color', 'w');
tiledlayout(5, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

colors = lines(numel(controllers));

nexttile;
hold on;
for i = 1:numel(controllers)
    out = results.(controllers{i}).out;
    if i == 1
        plot(out.t, out.q_ref, 'k--', 'LineWidth', 1.1);
    end
    plot(out.t, out.q, 'Color', colors(i, :), 'LineWidth', 1.2);
end
grid on;
ylabel('q / rad');
title(sprintf('%s: position', test_name), 'Interpreter', 'none');
legend(['q_ref', controllers], 'Location', 'best');

nexttile;
hold on;
for i = 1:numel(controllers)
    out = results.(controllers{i}).out;
    plot(out.t, out.e, 'Color', colors(i, :), 'LineWidth', 1.2);
end
grid on;
ylabel('e / rad');
title('tracking error');

nexttile;
hold on;
for i = 1:numel(controllers)
    out = results.(controllers{i}).out;
    plot(out.t, out.tau_applied, 'Color', colors(i, :), 'LineWidth', 1.2);
end
grid on;
ylabel('tau / N*m');
title('applied motor torque');

nexttile;
hold on;
for i = 1:numel(controllers)
    out = results.(controllers{i}).out;
    if i == 1
        plot(out.t, out.tau_load, 'k--', 'LineWidth', 1.1);
    end
    if strcmp(controllers{i}, 'dob_pd')
        plot(out.t, out.tau_load_hat, 'Color', colors(i, :), 'LineWidth', 1.2);
    end
end
grid on;
ylabel('load / N*m');
title('load torque and DOB estimate');

nexttile;
hold on;
for i = 1:numel(controllers)
    out = results.(controllers{i}).out;
    plot(out.t, out.current, 'Color', colors(i, :), 'LineWidth', 1.2);
end
grid on;
ylabel('I / A');
xlabel('time / s');
title('estimated motor current');

exportgraphics(fig, fullfile(results_dir, sprintf('%s_comparison.png', test_name)), 'Resolution', 160);
end
