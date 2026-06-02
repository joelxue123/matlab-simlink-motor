function plot_delay_scan(delay_table, results_dir)
%PLOT_DELAY_SCAN Plot delay sensitivity summary.

fig = figure('Name', 'delay_scan', 'Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

controllers = unique(delay_table.controller, 'stable');
colors = lines(numel(controllers));

nexttile;
hold on;
for i = 1:numel(controllers)
    idx = delay_table.controller == controllers(i);
    plot(delay_table.delay_ms(idx), delay_table.overshoot_pct(idx), '-o', ...
        'Color', colors(i, :), 'LineWidth', 1.2);
end
grid on;
ylabel('overshoot / %');
title('delay sensitivity');
legend(cellstr(controllers), 'Location', 'best');

nexttile;
hold on;
for i = 1:numel(controllers)
    idx = delay_table.controller == controllers(i);
    plot(delay_table.delay_ms(idx), delay_table.e_rms(idx), '-o', ...
        'Color', colors(i, :), 'LineWidth', 1.2);
end
grid on;
ylabel('e rms / rad');

nexttile;
hold on;
for i = 1:numel(controllers)
    idx = delay_table.controller == controllers(i);
    plot(delay_table.delay_ms(idx), delay_table.I_rms(idx), '-o', ...
        'Color', colors(i, :), 'LineWidth', 1.2);
end
grid on;
ylabel('I rms / A');
xlabel('command delay / ms');

exportgraphics(fig, fullfile(results_dir, 'delay_scan.png'), 'Resolution', 160);
end
