function plot_z_analysis(result, plotTitle)
%PLOT_Z_ANALYSIS Plot unit-circle poles/zeros and discrete Bode curves.

if nargin < 2
    plotTitle = 'Discrete z-domain analysis';
end

theta = linspace(0, 2 * pi, 400);
unitCircle = exp(1j * theta);

figure('Name', plotTitle, 'Color', 'w');

subplot(2, 2, 1);
plot(real(unitCircle), imag(unitCircle), 'k--', 'LineWidth', 1);
hold on;
plot(real(result.zeros), imag(result.zeros), 'bo', 'MarkerSize', 8, 'LineWidth', 1.5);
plot(real(result.poles), imag(result.poles), 'rx', 'MarkerSize', 9, 'LineWidth', 1.5);
grid on;
axis equal;
xlabel('Real');
ylabel('Imag');
title('Pole-zero map');
legend('Unit circle', 'Zeros', 'Poles', 'Location', 'best');

subplot(2, 2, 2);
plot(result.frequency_rad_sample, result.magnitude_db, 'LineWidth', 1.2);
grid on;
xlabel('\Omega (rad/sample)');
ylabel('Magnitude (dB)');
title('Magnitude response');
hold on;
yline(0, 'k--');

subplot(2, 2, 3);
plot(result.frequency_rad_sample, result.phase_deg, 'LineWidth', 1.2);
grid on;
xlabel('\Omega (rad/sample)');
ylabel('Phase (deg)');
title('Phase response');
hold on;
yline(-180, 'k--');

subplot(2, 2, 4);
stairs(0:numel(result.poleRadius)-1, sort(result.poleRadius, 'descend'), 'LineWidth', 1.2);
grid on;
xlabel('Pole index');
ylabel('|p|');
title('Pole radii');
hold on;
yline(1, 'r--');

sgtitle(plotTitle);
end