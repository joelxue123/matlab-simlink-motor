function metrics = compute_metrics(out, cfg)
%COMPUTE_METRICS Compute common control metrics for one simulation.

t = out.t;
q = out.q;
q_ref = out.q_ref;
e = out.e;

final_ref = q_ref(end);
step_amp = final_ref - cfg.ref.q_initial;
if abs(step_amp) < 1e-12
    step_amp = 1.0;
end

peak_q = max(q);
metrics.overshoot_pct = max(0.0, (peak_q - final_ref) / abs(step_amp) * 100.0);

band = 0.02*abs(step_amp);
idx_after_step = find(t >= cfg.ref.step_time, 1, 'first');
settling_time = NaN;
for k = idx_after_step:numel(t)
    if all(abs(q(k:end) - final_ref) <= band)
        settling_time = t(k) - cfg.ref.step_time;
        break;
    end
end

metrics.settling_time = settling_time;
metrics.final_error = e(end);
metrics.e_rms = sqrt(mean(e.^2));
metrics.e_peak = max(abs(e));
metrics.tau_rms = sqrt(mean(out.tau_applied.^2));
metrics.tau_peak = max(abs(out.tau_applied));
metrics.I_rms = sqrt(mean(out.current.^2));
metrics.I_peak = max(abs(out.current));
metrics.heat_mean = mean(out.heat_power);
metrics.heat_energy = trapz(t, out.heat_power);
metrics.load_rejection_error_peak = load_rejection_peak(out, cfg);
metrics.stable = all(isfinite(q)) && max(abs(q)) < 10*max(1, abs(final_ref));
end

function peak = load_rejection_peak(out, cfg)
    idx = find(out.t >= cfg.load.step_time, 1, 'first');
    if isempty(idx)
        peak = NaN;
        return;
    end
    peak = max(abs(out.e(idx:end)));
end
