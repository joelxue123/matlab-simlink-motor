function metrics = evaluate_speedloop_kf_test(varargin)
% Run speed-loop KF test model and report basic estimation metrics.
%
% Usage:
%   metrics = evaluate_speedloop_kf_test();

q_theta = [];
q_omega = [];
r_theta = [];
test_noise_var = [];
if nargin >= 1, q_theta = varargin{1}; end
if nargin >= 2, q_omega = varargin{2}; end
if nargin >= 3, r_theta = varargin{3}; end
if nargin >= 4, test_noise_var = varargin{4}; end

motor_control_params;
if ~isempty(q_theta), control.kf.q_theta = q_theta; end
if ~isempty(q_omega), control.kf.q_omega = q_omega; end
if ~isempty(r_theta), control.kf.r_theta = r_theta; end
if ~isempty(test_noise_var), control.kf.test_noise_var = test_noise_var; end
assignin('base', 'control', control);
assignin('base', 'motor', motor);
assignin('base', 'inverter', inverter);
assignin('base', 'simcfg', simcfg);
build_speedloop_kf_test;
set_param('speedloop_kf_test', 'InitFcn', '');
sim_out = sim('speedloop_kf_test', 'ReturnWorkspaceOutputs', 'on');

% Prefer SimulationOutput variables; fallback to base workspace if needed.
wm_ts = [];
wkf_ts = [];
wer_ts = [];
if isa(sim_out, 'Simulink.SimulationOutput')
    try
        wm_ts = sim_out.get('log_wm');
        wkf_ts = sim_out.get('log_wkf');
        wer_ts = sim_out.get('log_werr');
    catch
    end
end

if isempty(wm_ts) || isempty(wkf_ts)
    if evalin('base', 'exist(''log_wm'', ''var'')')
        wm_ts = evalin('base', 'log_wm');
    end
    if evalin('base', 'exist(''log_wkf'', ''var'')')
        wkf_ts = evalin('base', 'log_wkf');
    end
    if evalin('base', 'exist(''log_werr'', ''var'')')
        wer_ts = evalin('base', 'log_werr');
    end
end

if isempty(wm_ts) || isempty(wkf_ts)
    error('Missing log_wm or log_wkf. Ensure speedloop_kf_test model ran successfully.');
end

% Align onto w_meas timeline for fair comparison.
t = wm_ts.Time;
wm = wm_ts.Data;
wkf = interp1(wkf_ts.Time, wkf_ts.Data, t, 'linear', 'extrap');
err = wkf - wm;

metrics = struct();
metrics.rmse = sqrt(mean(err.^2));
metrics.mae = mean(abs(err));
metrics.max_abs_err = max(abs(err));

% Estimate delay via cross-correlation (positive lag means w_kf lags w_meas).
wm0 = wm - mean(wm);
wkf0 = wkf - mean(wkf);
[c, lags] = xcorr(wm0, wkf0, 'coeff');
[~, idx] = max(c);
lag_samples = lags(idx);
metrics.lag_samples = lag_samples;
metrics.lag_time_s = lag_samples * simcfg.Ts_plant;

% Also report RMS of exported error signal for consistency check.
if ~isempty(wer_ts)
    metrics.err_rms_from_log = rms(wer_ts.Data);
else
    metrics.err_rms_from_log = NaN;
end

fprintf('\n=== Speed-loop KF evaluation ===\n');
fprintf('RMSE        : %.6f rad/s\n', metrics.rmse);
fprintf('MAE         : %.6f rad/s\n', metrics.mae);
fprintf('Max |error| : %.6f rad/s\n', metrics.max_abs_err);
fprintf('Lag samples : %d\n', metrics.lag_samples);
fprintf('Lag time    : %.6e s\n', metrics.lag_time_s);
fprintf('RMS(log err): %.6f rad/s\n', metrics.err_rms_from_log);

assignin('base', 'kf_metrics', metrics);
end
