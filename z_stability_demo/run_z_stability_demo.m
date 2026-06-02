function run_z_stability_demo()
%RUN_Z_STABILITY_DEMO Demonstrate discrete-time stability analysis in z-domain.

Ts = 1 / 10000;

cases = {
    struct('name', '(z-1)/(z-0.95)', 'num', [1 -1], 'den', [1 -0.95])
    struct('name', '(z-1)/(z-1.00)', 'num', [1 -1], 'den', [1 -1.00])
    struct('name', '(z-1)/(z-1.05)', 'num', [1 -1], 'den', [1 -1.05])
    };

fprintf('Running z-domain stability demo with Ts = %.6g s\n', Ts);

for k = 1:numel(cases)
    item = cases{k};
    fprintf('\n============================================================\n');
    fprintf('Case %d: %s\n', k, item.name);
    result = analyze_z_stability(item.num, item.den, Ts);
    plot_z_analysis(result, ['Z stability: ' item.name]);
end

fprintf('\nInterpretation guide:\n');
fprintf('  1. Pole radius < 1  -> stable\n');
fprintf('  2. Pole radius = 1  -> marginal\n');
fprintf('  3. Pole radius > 1  -> unstable\n');
fprintf('  4. Frequency sweep uses z = exp(j*Omega), Omega in [0, pi]\n');
fprintf('  5. PM is read at |L(e^{jOmega})| = 1, GM at phase = -180 deg\n');
end