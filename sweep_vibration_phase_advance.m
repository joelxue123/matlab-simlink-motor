function result = sweep_vibration_phase_advance(phase_list_deg)
% Sweep offline vibration compensation phase advance and score each point.
%
% The primary objective is to minimize the ripple of the speed PI output
% (logged as iq_ref_base), while also monitoring speed ripple.
%
% Usage:
%   result = sweep_vibration_phase_advance();
%   result = sweep_vibration_phase_advance(-10:5:40);

if nargin < 1
    phase_list_deg = -10:5:40;
end
phase_list_deg = phase_list_deg(:).';

motor_control_params;
assignin('base', 'control', control);
assignin('base', 'motor', motor);
assignin('base', 'inverter', inverter);
assignin('base', 'simcfg', simcfg);

% Learn the offline FF table once from the no-compensation baseline.
learn_vibration_ff_table();
control = evalin('base', 'control');
window_start = max(control.vib.learn_start_time + 0.25, 0.40);
window_end = control.vib.test_stop_time;

results = struct('phase_advance_deg', {}, 'iqbase_std', {}, 'iqbase_pp', {}, ...
    'speed_std', {}, 'speed_pp', {}, 'score', {});

fprintf('\n=== Sweep vibration phase advance ===\n');
for idx = 1:numel(phase_list_deg)
    phase_deg = phase_list_deg(idx);
    control = evalin('base', 'control');
    control.vib.mode = 'offline';
    control.vib.enable_ff = 1;
    control.vib.phase_advance_deg = phase_deg;
    assignin('base', 'control', control);

    build_vibration_comp_test;
    set_param('vibration_comp_test', 'InitFcn', '');
    sim_out = sim('vibration_comp_test', 'ReturnWorkspaceOutputs', 'on');

    iqbase_ts = sim_out.get('log_vib_iqbase');
    wm_ts = sim_out.get('log_vib_wm');

    [iqbase_std, iqbase_pp] = local_ripple_metrics(iqbase_ts, window_start, window_end);
    [speed_std, speed_pp] = local_ripple_metrics(wm_ts, window_start, window_end);

    % Prioritize the speed-PI output smoothness, then speed ripple.
    score = iqbase_std + 0.35 * speed_std + 0.10 * iqbase_pp;

    results(idx).phase_advance_deg = phase_deg;
    results(idx).iqbase_std = iqbase_std;
    results(idx).iqbase_pp = iqbase_pp;
    results(idx).speed_std = speed_std;
    results(idx).speed_pp = speed_pp;
    results(idx).score = score;

    fprintf('phase=%6.1f deg | iqbase std=%8.5f | speed std=%8.5f | score=%8.5f\n', ...
        phase_deg, iqbase_std, speed_std, score);
end

scores = [results.score];
[~, best_idx] = min(scores);
best = results(best_idx);

figure('Name', 'Phase Advance Sweep', 'Color', 'w');
subplot(2,1,1);
plot([results.phase_advance_deg], [results.iqbase_std], '-o', 'LineWidth', 1.2); hold on;
plot(best.phase_advance_deg, best.iqbase_std, 'rx', 'MarkerSize', 10, 'LineWidth', 2);
grid on;
ylabel('iq_ref_base ripple std (A)');
title('Speed PI Output Ripple vs Phase Advance');

subplot(2,1,2);
plot([results.phase_advance_deg], [results.speed_std], '-o', 'LineWidth', 1.2); hold on;
plot(best.phase_advance_deg, best.speed_std, 'rx', 'MarkerSize', 10, 'LineWidth', 2);
grid on;
xlabel('Phase Advance (deg)');
ylabel('Speed ripple std (rad/s)');
title('Speed Ripple vs Phase Advance');

fprintf('\n=== Best phase advance ===\n');
fprintf('phase_advance_deg : %.2f\n', best.phase_advance_deg);
fprintf('iqbase std        : %.6f A\n', best.iqbase_std);
fprintf('iqbase p-p        : %.6f A\n', best.iqbase_pp);
fprintf('speed std         : %.6f rad/s\n', best.speed_std);
fprintf('speed p-p         : %.6f rad/s\n', best.speed_pp);
fprintf('score             : %.6f\n', best.score);

result = struct();
result.results = results;
result.best = best;
result.window_start = window_start;
result.window_end = window_end;
assignin('base', 'vib_phase_sweep_result', result);
end

function [ripple_std, ripple_pp] = local_ripple_metrics(ts, t0, t1)
mask = ts.Time >= t0 & ts.Time <= t1;
y = ts.Data(mask);
if isempty(y)
    ripple_std = NaN;
    ripple_pp = NaN;
    return;
end
y = y(:) - mean(y(:));
ripple_std = std(y);
ripple_pp = max(y) - min(y);
end
