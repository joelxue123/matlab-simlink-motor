function result = run_switching_deadtime_motor_smoke_test(varargin)
%RUN_SWITCHING_DEADTIME_MOTOR_SMOKE_TEST Build and run a switching inverter + PMSM harness.
%
% This is a short smoke test for the physical switching path:
%
%   center-aligned PWM + deadtime gates
%     -> Universal Bridge MOSFET/Diodes
%     -> SPS Permanent Magnet Synchronous Machine
%     -> phase current logging

switching_sampling_study_config;

ss_cfg.model_name = 'switching_deadtime_motor_model';
ss_cfg.Vdc = 12;
ss_cfg.f_pwm = 20e3;
ss_cfg.T_pwm = 1 / ss_cfg.f_pwm;
ss_cfg.dead_time_s = 500e-9;
ss_cfg.adc_settle_time_s = 1.0e-6;
ss_cfg.deadtime_comp_enable = 1;
ss_cfg.deadtime_comp_update_s = ss_cfg.T_pwm;
ss_cfg.deadtime_comp_gain = 1.0;
ss_cfg.deadtime_comp_polarity = -1.0;
ss_cfg.deadtime_comp_id_A = 0.0;
ss_cfg.deadtime_comp_iq_A = 0.2;
ss_cfg.deadtime_comp_current_zero_A = 0.02;
ss_cfg.deadtime_comp_current_full_A = 0.10;
ss_cfg.deadtime_comp_current_inv_range_1perA = ...
    1 / (ss_cfg.deadtime_comp_current_full_A - ss_cfg.deadtime_comp_current_zero_A);
ss_cfg.deadtime_comp_max_duty = 0.03;
ss_cfg.modulation_ratio = 0.9;
ss_cfg.theta_e_deg = 60;
ss_cfg.electrical_freq_hz = 80;
ss_cfg.sim_stop_time = 200e-6;
ss_cfg.samples_per_pwm_model = 2000;
ss_cfg.powergui_sample_time_s = 25e-9;
ss_cfg.model_fixed_step_s = min(ss_cfg.T_pwm / ss_cfg.samples_per_pwm_model, ss_cfg.powergui_sample_time_s);
ss_cfg.pmsm_speed_ref_rad_s = 0.0;
ss_cfg.pmsm_resistance_ohm = 4.0;
ss_cfg.pmsm_inductance_h = 100e-6;
ss_cfg.pmsm_flux_wb = 0.018;
ss_cfg.pmsm_pole_pairs = 4;
ss_cfg.pmsm_mechanical = [1.2e-6 1.0e-6 4 0];
ss_cfg.pmsm_initial_conditions = [0 0 0 0];

ss_cfg = local_parse_inputs(ss_cfg, varargin{:});

mdl = build_switching_sampling_study_model(ss_cfg);
set_param(mdl, 'StopTime', num2str(ss_cfg.sim_stop_time, '%.12g'));
save_system(mdl);

sim_out = sim(mdl, 'ReturnWorkspaceOutputs', 'on');

t = get_logged_time(sim_out, 'i_a');
ia = get_logged_values(sim_out, 'i_a');
ib = get_logged_values(sim_out, 'i_b');
ic = get_logged_values(sim_out, 'i_c');

result.model = mdl;
result.cfg = ss_cfg;
result.time = t;
result.ia = ia;
result.ib = ib;
result.ic = ic;
result.deadtime_comp_a = get_logged_values(sim_out, 'deadtime_comp_a');
result.deadtime_comp_b = get_logged_values(sim_out, 'deadtime_comp_b');
result.deadtime_comp_c = get_logged_values(sim_out, 'deadtime_comp_c');
result.deadtime_active_a = get_logged_values(sim_out, 'deadtime_active_a');
result.deadtime_active_b = get_logged_values(sim_out, 'deadtime_active_b');
result.deadtime_active_c = get_logged_values(sim_out, 'deadtime_active_c');
result.ia_pkpk_A = max(ia) - min(ia);
result.ib_pkpk_A = max(ib) - min(ib);
result.ic_pkpk_A = max(ic) - min(ic);
result.sum_current_rms_A = rms(ia + ib + ic);
result.max_abs_current_A = max(abs([ia(:); ib(:); ic(:)]));

assert(all(isfinite([ia(:); ib(:); ic(:)])), 'Switching PMSM current contains non-finite values.');
assert(result.max_abs_current_A > 0, 'Switching PMSM current stayed at zero.');

results_dir = fullfile(fileparts(mfilename('fullpath')), 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

report_file = fullfile(results_dir, 'switching_deadtime_motor_smoke_report.txt');
fid = fopen(report_file, 'w');
cleanup_file = onCleanup(@() fclose(fid));

fprintf(fid, 'Switching deadtime motor smoke test\n');
fprintf(fid, 'model = %s\n', result.model);
fprintf(fid, 'Vdc = %.6g V\n', ss_cfg.Vdc);
fprintf(fid, 'PWM = %.6g Hz\n', ss_cfg.f_pwm);
fprintf(fid, 'dead_time = %.6g s\n', ss_cfg.dead_time_s);
fprintf(fid, 'deadtime_comp_enable = %.6g\n', ss_cfg.deadtime_comp_enable);
fprintf(fid, 'deadtime_comp_update = %.6g s\n', ss_cfg.deadtime_comp_update_s);
fprintf(fid, 'deadtime_comp_duty = %.6g\n', ...
    min(ss_cfg.deadtime_comp_gain * ss_cfg.dead_time_s / ss_cfg.T_pwm, ss_cfg.deadtime_comp_max_duty));
fprintf(fid, 'deadtime_comp_current_source = dq_synthesized\n');
fprintf(fid, 'deadtime_comp_id_A = %.6g\n', ss_cfg.deadtime_comp_id_A);
fprintf(fid, 'deadtime_comp_iq_A = %.6g\n', ss_cfg.deadtime_comp_iq_A);
fprintf(fid, 'deadtime_comp_current_zero_A = %.6g\n', ss_cfg.deadtime_comp_current_zero_A);
fprintf(fid, 'deadtime_comp_current_full_A = %.6g\n', ss_cfg.deadtime_comp_current_full_A);
fprintf(fid, 'deadtime_comp_polarity = %.6g\n', ss_cfg.deadtime_comp_polarity);
fprintf(fid, 'R = %.6g ohm\n', ss_cfg.pmsm_resistance_ohm);
fprintf(fid, 'L = %.6g H\n', ss_cfg.pmsm_inductance_h);
fprintf(fid, 'sim_stop_time = %.6g s\n', ss_cfg.sim_stop_time);
fprintf(fid, 'ia_pkpk_A = %.9g\n', result.ia_pkpk_A);
fprintf(fid, 'ib_pkpk_A = %.9g\n', result.ib_pkpk_A);
fprintf(fid, 'ic_pkpk_A = %.9g\n', result.ic_pkpk_A);
fprintf(fid, 'deadtime_comp_range = a[%.9g %.9g], b[%.9g %.9g], c[%.9g %.9g]\n', ...
    min(result.deadtime_comp_a), max(result.deadtime_comp_a), ...
    min(result.deadtime_comp_b), max(result.deadtime_comp_b), ...
    min(result.deadtime_comp_c), max(result.deadtime_comp_c));
fprintf(fid, 'deadtime_active_count = [%.9g %.9g %.9g]\n', ...
    sum(result.deadtime_active_a ~= 0), ...
    sum(result.deadtime_active_b ~= 0), ...
    sum(result.deadtime_active_c ~= 0));
fprintf(fid, 'sum_current_rms_A = %.9g\n', result.sum_current_rms_A);
fprintf(fid, 'max_abs_current_A = %.9g\n', result.max_abs_current_A);
fprintf(fid, 'Result: PASS\n');

fprintf('\nSwitching deadtime motor smoke test passed.\n');
fprintf('model: %s\n', result.model);
fprintf('deadtime compensation: enable=%.0f, duty=%.5f, update=%.2f us\n', ...
    ss_cfg.deadtime_comp_enable, ...
    min(ss_cfg.deadtime_comp_gain * ss_cfg.dead_time_s / ss_cfg.T_pwm, ss_cfg.deadtime_comp_max_duty), ...
    ss_cfg.deadtime_comp_update_s * 1e6);
fprintf('deadtime polarity current source: dq synth, id/iq=[%.4f %.4f] A\n', ...
    ss_cfg.deadtime_comp_id_A, ss_cfg.deadtime_comp_iq_A);
fprintf('ia/ib/ic pk-pk = [%.4f %.4f %.4f] A\n', ...
    result.ia_pkpk_A, result.ib_pkpk_A, result.ic_pkpk_A);
fprintf('sum current RMS = %.6g A\n', result.sum_current_rms_A);
fprintf('Saved report:\n  %s\n', report_file);
end

function ss_cfg = local_parse_inputs(ss_cfg, varargin)
if mod(numel(varargin), 2) ~= 0
    error('Arguments must be name/value pairs.');
end

for idx = 1:2:numel(varargin)
    name = lower(varargin{idx});
    value = varargin{idx + 1};
    switch name
        case 'vdc'
            ss_cfg.Vdc = value;
        case 'deadtime'
            ss_cfg.dead_time_s = value;
        case 'modulationratio'
            ss_cfg.modulation_ratio = value;
        case 'electricalfreqhz'
            ss_cfg.electrical_freq_hz = value;
        case 'simstoptime'
            ss_cfg.sim_stop_time = value;
        case 'rohm'
            ss_cfg.pmsm_resistance_ohm = value;
        case 'lh'
            ss_cfg.pmsm_inductance_h = value;
        case 'deadtimecompenable'
            ss_cfg.deadtime_comp_enable = value;
        case 'deadtimecompgain'
            ss_cfg.deadtime_comp_gain = value;
        case 'deadtimecomppolarity'
            ss_cfg.deadtime_comp_polarity = value;
        case 'deadtimecompid'
            ss_cfg.deadtime_comp_id_A = value;
        case 'deadtimecompiq'
            ss_cfg.deadtime_comp_iq_A = value;
        case 'deadtimecompcurrentzero'
            ss_cfg.deadtime_comp_current_zero_A = value;
        case 'deadtimecompcurrentfull'
            ss_cfg.deadtime_comp_current_full_A = value;
        otherwise
            error('Unknown parameter: %s', varargin{idx});
    end
end

if ss_cfg.deadtime_comp_current_full_A <= ss_cfg.deadtime_comp_current_zero_A
    error('deadtimeCompCurrentFull must be greater than deadtimeCompCurrentZero.');
end

ss_cfg.deadtime_comp_current_inv_range_1perA = ...
    1 / (ss_cfg.deadtime_comp_current_full_A - ss_cfg.deadtime_comp_current_zero_A);
end

function t = get_logged_time(sim_out, name)
data = sim_out.get(name);
if isstruct(data) && isfield(data, 'time')
    t = data.time;
elseif isa(data, 'timeseries')
    t = data.Time;
else
    error('Unsupported logged time format for "%s": %s', name, class(data));
end
end

function values = get_logged_values(sim_out, name)
data = sim_out.get(name);
if isstruct(data) && isfield(data, 'signals') && isfield(data.signals, 'values')
    values = data.signals.values;
elseif isa(data, 'timeseries')
    values = data.Data;
else
    error('Unsupported logged value format for "%s": %s', name, class(data));
end

values = values(:);
end
