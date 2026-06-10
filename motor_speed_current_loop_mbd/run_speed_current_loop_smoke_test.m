%% Smoke test for motor_speed_current_loop_model
%
% The test integrates:
%   SpeedPiStep -> CurrentPiStep -> DqToAbcDutyStep -> Average Inverter -> PMSM
%
% It checks the multi-rate boundaries, finite signals, duty range, speed
% response, and speed-loop current limiting.

clear; clc;

script_dir = fileparts(mfilename('fullpath'));
run(fullfile(script_dir, 'build_motor_speed_current_loop_model.m'));

cfg = evalin('base', 'motor_speed_current_loop_config');
model = cfg.modelName;
model_file = fullfile(script_dir, [model '.slx']);
cleanup = onCleanup(@() cleanup_after_test(model));

load_system(model_file);

assert_rate_transition_sample_time(model, 'PhaseDuty_RateTransition_25us', ...
    evalin('base', 'simcfg.Ts_plant'));
assert_rate_transition_sample_time(model, 'Vdc_RateTransition_25us', ...
    evalin('base', 'simcfg.Ts_plant'));
assert_rate_transition_sample_time(model, 'ia_feedback_rt_50us', ...
    evalin('base', 'simcfg.Ts_ctrl'));
assert_rate_transition_sample_time(model, 'theta_e_feedback_rt_50us', ...
    evalin('base', 'simcfg.Ts_ctrl'));
assert_rate_transition_sample_time(model, 'wm_feedback_rt_100us', ...
    evalin('base', 'simcfg.Ts_speed'));
assert_rate_transition_sample_time(model, 'IqRef_RateTransition_50us', ...
    evalin('base', 'simcfg.Ts_ctrl'));

sim_out = sim(model, ...
    'ReturnWorkspaceOutputs', 'on', ...
    'SaveOutput', 'off');

wm_ref = sim_out.get('log_wm_ref').Data;
wm_meas = sim_out.get('log_wm_meas').Data;
iq_ref = sim_out.get('log_iq_ref').Data;
iq_ref_speed = sim_out.get('log_iq_ref_speed').Data;
iq_meas = sim_out.get('log_iq_meas').Data;
id_meas = sim_out.get('log_id_meas').Data;
vd_ref = sim_out.get('log_vd_ref').Data;
vq_ref = sim_out.get('log_vq_ref').Data;
da = sim_out.get('log_da').Data;
db = sim_out.get('log_db').Data;
dc = sim_out.get('log_dc').Data;

duty_values = [da(:); db(:); dc(:)];
all_values = [double(wm_ref(:)); double(wm_meas(:)); ...
    double(iq_ref(:)); double(iq_ref_speed(:)); double(iq_meas(:)); ...
    double(id_meas(:)); double(vd_ref(:)); double(vq_ref(:)); ...
    double(duty_values(:))];

params = evalin('base', 'speed_current_loop');
iq_limit = double(params.IqLimitDefault);
final_wm_ref = double(wm_ref(end));
final_wm_meas = double(wm_meas(end));
final_speed_error = final_wm_ref - final_wm_meas;
max_abs_id = max(abs(double(id_meas(:))));

fprintf('\nSpeed-current-loop smoke test result:\n');
fprintf('  wm_ref final   = %.6g rad/s\n', final_wm_ref);
fprintf('  wm_meas final  = %.6g rad/s\n', final_wm_meas);
fprintf('  speed error    = %.6g rad/s\n', final_speed_error);
fprintf('  iq_ref range   = [%.6g, %.6g] A\n', ...
    min(double(iq_ref(:))), max(double(iq_ref(:))));
fprintf('  iq_meas range  = [%.6g, %.6g] A\n', ...
    min(double(iq_meas(:))), max(double(iq_meas(:))));
fprintf('  id_meas range  = [%.6g, %.6g] A\n', ...
    min(double(id_meas(:))), max(double(id_meas(:))));
fprintf('  vd_ref range   = [%.6g, %.6g] V\n', ...
    min(double(vd_ref(:))), max(double(vd_ref(:))));
fprintf('  vq_ref range   = [%.6g, %.6g] V\n', ...
    min(double(vq_ref(:))), max(double(vq_ref(:))));
fprintf('  duty range     = [%.6g, %.6g]\n', ...
    min(double(duty_values)), max(double(duty_values)));
fprintf('  iq_limit       = %.6g A\n', iq_limit);

if any(~isfinite(all_values))
    error('Speed-current-loop smoke test failed: non-finite value detected.');
end

if any(duty_values < -1e-6) || any(duty_values > 1 + 1e-6)
    error('Speed-current-loop smoke test failed: duty command is outside [0, 1].');
end

if any(abs(double(iq_ref(:))) > iq_limit + 1e-4)
    error('Speed-current-loop smoke test failed: iq_ref exceeded iq_limit.');
end

if max(double(iq_ref(:))) < 0.5 * iq_limit
    error('Speed-current-loop smoke test failed: speed loop did not request acceleration current.');
end

if final_wm_meas < 0.8 * final_wm_ref
    error('Speed-current-loop smoke test failed: motor speed did not rise toward the command.');
end

if abs(final_speed_error) > 2.0
    error('Speed-current-loop smoke test failed: final speed error is too large.');
end

if max_abs_id > 2.0
    error('Speed-current-loop smoke test failed: d-axis current drift is too large.');
end

fprintf('Speed-current-loop smoke test passed.\n');

function assert_rate_transition_sample_time(model, block_name, expected_sample_time)
block_path = [model '/' block_name];
sample_time_text = get_param(block_path, 'OutPortSampleTime');
actual_sample_time = evaluate_sample_time(sample_time_text);

if ~isfinite(actual_sample_time) || abs(actual_sample_time - expected_sample_time) > eps(expected_sample_time)
    error('Sample time check failed for %s: expected %.12g, got %s.', ...
        block_path, expected_sample_time, sample_time_text);
end
end

function sample_time = evaluate_sample_time(sample_time_text)
sample_time = str2double(sample_time_text);

if isfinite(sample_time)
    return;
end

sample_time = evalin('base', sample_time_text);
end

function cleanup_after_test(model)
if bdIsLoaded(model)
    close_system(model, 0);
end

try
    Simulink.data.dictionary.closeAll('-discard');
catch
end
end
