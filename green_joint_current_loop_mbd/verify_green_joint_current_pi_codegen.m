function verify_green_joint_current_pi_codegen()
%% Verify generated C shape for green-joint current PI
%
% This is a lightweight guardrail for the agreed first production strategy:
% Vd-priority voltage allocation plus back-calculation anti-windup.

script_dir = fileparts(mfilename('fullpath'));
model = 'green_joint_current_loop_model';
code_dir = fullfile(script_dir, [model '_ert_rtw']);
step_c = fullfile(code_dir, 'GreenJointCurrentLoopStep.c');
types_h = fullfile(code_dir, 'green_joint_current_loop_types.h');

assert_file_exists(step_c);
assert_file_exists(types_h);

step_text = fileread(step_c);
types_text = fileread(types_h);

assert_contains(step_text, ...
    'rtb_vd_cmd = fmaxf(fminf', ...
    'Generated C no longer clamps Vd before allocating Vq.');
assert_contains(step_text, ...
    'VoltageLimitRatio * VoltageModulationRatio * rtu_loop_in->vbus', ...
    'Generated C no longer applies modulation headroom inside the physical PI voltage limit.');
assert_contains(step_text, ...
    'rtb_vq_norm * rtb_vq_norm - rtb_voltage_mag', ...
    'Generated C no longer subtracts Vd squared from Vlimit squared.');
assert_contains(step_text, ...
    'sqrtf', ...
    'Generated C no longer computes a square-root voltage limit/magnitude.');
assert_contains(step_text, ...
    'rtb_vd_cmd -', ...
    'Generated C no longer applies D-axis back-calculation anti-windup.');
assert_contains(step_text, ...
    'rtb_q_integrator_state -', ...
    'Generated C no longer applies Q-axis back-calculation anti-windup.');
assert_contains(step_text, ...
    'rty_loop_out->vd_mod = VoltageModulationRatio *', ...
    'Generated C no longer applies MBD-owned final modulation headroom.');
assert_contains(step_text, ...
    'rty_loop_out->vq_mod = VoltageModulationRatio *', ...
    'Generated C no longer applies MBD-owned final Q-axis modulation headroom.');
assert_contains(step_text, ...
    '/* Product: ''<S1>/inv_voltage_limit''', ...
    'Generated C no longer computes one reciprocal for voltage normalization.');
assert_contains(step_text, ...
    ' / fmaxf(', ...
    'Generated C no longer computes reciprocal with guarded voltage limit denominator.');
assert_contains(step_text, ...
    'rtb_vd_norm = rtb_vd_cmd *', ...
    'Generated C no longer uses reciprocal multiplication for D-axis normalization.');
assert_contains(step_text, ...
    'rtb_voltage_mag_norm = rtb_voltage_mag *', ...
    'Generated C no longer uses reciprocal multiplication for voltage magnitude normalization.');
assert_normalization_divide_count(step_text, 1);
assert_contains(types_text, ...
    'vd_mod', ...
    'Generated output interface no longer exposes final D-axis modulation command.');
assert_contains(types_text, ...
    'vq_mod', ...
    'Generated output interface no longer exposes final Q-axis modulation command.');

for forbidden = ["duty", "sector", "theta_e", " ia", " ib", " ic"]
    if contains(types_text, forbidden)
        error('Generated interface unexpectedly contains "%s".', forbidden);
    end
end

fprintf('\nGreen-joint current PI codegen verification passed.\n');
fprintf('  Strategy: Vd-priority allocation + back-calculation anti-windup\n');
end

function assert_file_exists(file_name)
if ~exist(file_name, 'file')
    error('Expected generated file does not exist: %s', file_name);
end
end

function assert_contains(text, pattern, message)
if ~contains(text, pattern)
    error('%s\nMissing pattern: %s', message, pattern);
end
end

function assert_normalization_divide_count(text, expected_count)
patterns = [" / rtb_", " / Voltage", " / local", " / rtu_", " / fmaxf("];
count = 0;
for i = 1:numel(patterns)
    count = count + count_substrings(text, patterns(i));
end

if count ~= expected_count
    error(['Generated C should use one reciprocal divide for voltage ' ...
        'normalization. Found %d divide-like patterns, expected %d.'], ...
        count, expected_count);
end
end

function count = count_substrings(text, pattern)
count = numel(strfind(text, char(pattern)));
end
