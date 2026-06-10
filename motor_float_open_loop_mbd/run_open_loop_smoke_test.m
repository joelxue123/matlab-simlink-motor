%% Smoke test for motor_float_open_loop_model
%
% This test verifies that the float MBD open-loop motor architecture builds and
% simulates. It is not a control-performance test yet.

clear; clc;

script_dir = fileparts(mfilename('fullpath'));
run(fullfile(script_dir, 'build_motor_float_open_loop_model.m'));

cfg = evalin('base', 'motor_float_open_loop_config');
model = cfg.modelName;
model_file = fullfile(script_dir, [model '.slx']);

load_system(model_file);

assert_rate_transition_sample_time(model, 'PhaseDuty_RateTransition_25us', ...
    evalin('base', 'simcfg.Ts_plant'));
assert_rate_transition_sample_time(model, 'Vdc_RateTransition_25us', ...
    evalin('base', 'simcfg.Ts_plant'));

sim_out = sim(model, ...
    'ReturnWorkspaceOutputs', 'on', ...
    'SaveOutput', 'off');

wm = last_sample(sim_out.get('log_wm'));
theta_e = last_sample(sim_out.get('log_theta_e'));
ia = last_sample(sim_out.get('log_ia'));
ib = last_sample(sim_out.get('log_ib'));
ic = last_sample(sim_out.get('log_ic'));
da = sim_out.get('log_da').Data;
db = sim_out.get('log_db').Data;
dc = sim_out.get('log_dc').Data;
duty_values = [da(:); db(:); dc(:)];

fprintf('\nOpen-loop smoke test result:\n');
fprintf('  wm      = %.6g rad/s\n', double(wm));
fprintf('  theta_e = %.6g rad\n', double(theta_e));
fprintf('  ia      = %.6g A\n', double(ia));
fprintf('  ib      = %.6g A\n', double(ib));
fprintf('  ic      = %.6g A\n', double(ic));
fprintf('  duty    = [%.6g, %.6g]\n', min(double(duty_values)), max(double(duty_values)));

values = [double(wm), double(theta_e), double(ia), double(ib), double(ic)];

if any(~isfinite(values))
    error('Open-loop smoke test failed: non-finite feedback value detected.');
end

if any(~isfinite(double(duty_values))) || any(duty_values < 0) || any(duty_values > 1)
    error('Open-loop smoke test failed: duty command is outside [0, 1].');
end

fprintf('Open-loop smoke test passed.\n');

function value = last_sample(signal)
value = signal.Data(end);
end

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
