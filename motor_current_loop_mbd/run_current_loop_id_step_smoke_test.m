%% Smoke test for motor_current_loop_model
%
% The test applies an id_ref step with iq_ref = 0. For a surface PMSM this
% should validate d-axis current regulation without producing meaningful
% average torque.

clear; clc;

script_dir = fileparts(mfilename('fullpath'));
run(fullfile(script_dir, 'build_motor_current_loop_model.m'));

cfg = evalin('base', 'motor_current_loop_config');
model = cfg.modelName;
model_file = fullfile(script_dir, [model '.slx']);

load_system(model_file);

assert_rate_transition_sample_time(model, 'PhaseDuty_RateTransition_25us', ...
    evalin('base', 'simcfg.Ts_plant'));
assert_rate_transition_sample_time(model, 'Vdc_RateTransition_25us', ...
    evalin('base', 'simcfg.Ts_plant'));
assert_rate_transition_sample_time(model, 'ia_feedback_rt_50us', ...
    evalin('base', 'simcfg.Ts_ctrl'));
assert_rate_transition_sample_time(model, 'theta_e_feedback_rt_50us', ...
    evalin('base', 'simcfg.Ts_ctrl'));

sim_out = sim(model, ...
    'ReturnWorkspaceOutputs', 'on', ...
    'SaveOutput', 'off');

id_ref = sim_out.get('log_id_ref').Data;
id_meas = sim_out.get('log_id_meas').Data;
iq_meas = sim_out.get('log_iq_meas').Data;
vd_ref = sim_out.get('log_vd_ref').Data;
vq_ref = sim_out.get('log_vq_ref').Data;
wm = sim_out.get('log_wm').Data;
da = sim_out.get('log_da').Data;
db = sim_out.get('log_db').Data;
dc = sim_out.get('log_dc').Data;

duty_values = [da(:); db(:); dc(:)];
all_values = [id_ref(:); id_meas(:); iq_meas(:); vd_ref(:); vq_ref(:); ...
    wm(:); duty_values(:)];

final_id_ref = double(id_ref(end));
final_id_meas = double(id_meas(end));
max_abs_iq = max(abs(double(iq_meas(:))));
max_abs_wm = max(abs(double(wm(:))));

fprintf('\nCurrent-loop id-step smoke test result:\n');
fprintf('  id_ref final  = %.6g A\n', final_id_ref);
fprintf('  id_meas final = %.6g A\n', final_id_meas);
fprintf('  iq_meas range = [%.6g, %.6g] A\n', ...
    min(double(iq_meas(:))), max(double(iq_meas(:))));
fprintf('  vd_ref range  = [%.6g, %.6g] V\n', ...
    min(double(vd_ref(:))), max(double(vd_ref(:))));
fprintf('  vq_ref range  = [%.6g, %.6g] V\n', ...
    min(double(vq_ref(:))), max(double(vq_ref(:))));
fprintf('  wm max abs    = %.6g rad/s\n', max_abs_wm);
fprintf('  duty range    = [%.6g, %.6g]\n', ...
    min(double(duty_values)), max(double(duty_values)));

if any(~isfinite(double(all_values)))
    error('Current-loop smoke test failed: non-finite value detected.');
end

if any(duty_values < 0) || any(duty_values > 1)
    error('Current-loop smoke test failed: duty command is outside [0, 1].');
end

if final_id_meas < 0.5 * final_id_ref
    error('Current-loop smoke test failed: id current did not respond to the step.');
end

if max_abs_iq > 0.5
    error('Current-loop smoke test failed: iq current is too large for an id-only test.');
end

if max_abs_wm > 0.5
    error('Current-loop smoke test failed: motor moved too much for an id-only test.');
end

fprintf('Current-loop id-step smoke test passed.\n');

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
