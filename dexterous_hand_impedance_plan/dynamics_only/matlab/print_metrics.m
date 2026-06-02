function print_metrics(name, metrics)
%PRINT_METRICS Compact command-window metric report.

fprintf('%s\n', name);
fprintf('  overshoot      : %.3f %%\n', metrics.overshoot_pct);
fprintf('  settling_time  : %.4f s\n', metrics.settling_time);
fprintf('  final_error    : %.6f rad\n', metrics.final_error);
fprintf('  e_rms          : %.6f rad\n', metrics.e_rms);
fprintf('  e_peak         : %.6f rad\n', metrics.e_peak);
fprintf('  tau_rms        : %.6f N*m\n', metrics.tau_rms);
fprintf('  tau_peak       : %.6f N*m\n', metrics.tau_peak);
fprintf('  I_rms          : %.6f A\n', metrics.I_rms);
fprintf('  heat_energy    : %.6f J\n\n', metrics.heat_energy);
end
