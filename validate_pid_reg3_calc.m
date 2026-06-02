function result = validate_pid_reg3_calc()
% Validate the discrete behavior of pid_reg3_calc against a standard
% back-calculation PI implementation under a few representative scenarios.

fprintf('Validating pid_reg3_calc anti-windup behavior...\n');

params = struct( ...
    'Kp', 0.8, ...
    'Ki', 0.15, ...
    'Kc', 0.35, ...
    'OutMax', 1.0, ...
    'OutMin', -1.0);

cases = [
    struct('name', 'linear_small_step', 'ref', 0.1 * ones(1, 5), 'fdb', zeros(1, 5)), ...
    struct('name', 'upper_saturation_hold', 'ref', 3.0 * ones(1, 40), 'fdb', zeros(1, 40)), ...
    struct('name', 'upper_saturation_recovery', 'ref', [3.0 * ones(1, 20), -0.5 * ones(1, 20)], 'fdb', zeros(1, 40)) ...
    ];

result = struct();
for case_index = 1:numel(cases)
    case_def = cases(case_index);
    user_log = simulate_case(case_def.ref, case_def.fdb, params, @pid_reg3_step);
    ref_log = simulate_case(case_def.ref, case_def.fdb, params, @standard_backcalc_step);

    metrics = struct();
    metrics.max_abs_out_diff = max(abs(user_log.Out - ref_log.Out));
    metrics.max_abs_ui_diff = max(abs(user_log.Ui - ref_log.Ui));
    metrics.final_out = user_log.Out(end);
    metrics.final_ui = user_log.Ui(end);
    metrics.saturated_samples = nnz(abs(user_log.Out - user_log.OutPreSat) > 1e-12);
    metrics.first_sample_out = user_log.Out(1);
    metrics.second_sample_ui = user_log.Ui(min(2, numel(user_log.Ui)));

    result.(case_def.name).user = user_log;
    result.(case_def.name).reference = ref_log;
    result.(case_def.name).metrics = metrics;

    fprintf('\nCase: %s\n', case_def.name);
    fprintf('  max |Out_user - Out_ref| = %.6f\n', metrics.max_abs_out_diff);
    fprintf('  max |Ui_user - Ui_ref|   = %.6f\n', metrics.max_abs_ui_diff);
    fprintf('  saturated samples        = %d\n', metrics.saturated_samples);
    fprintf('  user final Out / Ui      = %.6f / %.6f\n', metrics.final_out, metrics.final_ui);
end

fprintf('\nChecks:\n');

linear_ok = result.linear_small_step.metrics.max_abs_out_diff < 1e-12;
fprintf('  1) Linear region equivalence: %s\n', pass_fail(linear_ok));

delay_detected = result.upper_saturation_hold.metrics.max_abs_ui_diff > 1e-3;
fprintf('  2) One-sample anti-windup delay relative to standard back-calculation: %s\n', pass_fail(delay_detected));

recovery_ok = result.upper_saturation_recovery.user.Out(end) < 0;
fprintf('  3) Recovery after sign reversal: %s\n', pass_fail(recovery_ok));

fprintf('\nConclusion:\n');
fprintf('  - The implementation works as a discrete PI with output saturation.\n');
fprintf('  - Its anti-windup action is based on the previous sample OutPreSat, so it is not identical to standard same-sample back-calculation.\n');
fprintf('  - Ki must already include the sampling-period effect, otherwise the integral gain is too large by a factor of Ts^-1.\n');
fprintf('  - Ui clamping is functional but conservative because it uses the full output limits before adding the proportional term.\n');
end

function log = simulate_case(ref, fdb, params, step_fcn)
state = struct( ...
    'Ref', 0, ...
    'Fdb', 0, ...
    'Err', 0, ...
    'Kp', params.Kp, ...
    'Ki', params.Ki, ...
    'Kc', params.Kc, ...
    'Up', 0, ...
    'Ui', 0, ...
    'OutPreSat', 0, ...
    'Out', 0, ...
    'OutMax', params.OutMax, ...
    'OutMin', params.OutMin);

sample_count = numel(ref);
log = struct('Err', zeros(1, sample_count), 'Up', zeros(1, sample_count), ...
    'Ui', zeros(1, sample_count), 'OutPreSat', zeros(1, sample_count), ...
    'Out', zeros(1, sample_count));

for sample_index = 1:sample_count
    state.Ref = ref(sample_index);
    state.Fdb = fdb(sample_index);
    state = step_fcn(state);
    log.Err(sample_index) = state.Err;
    log.Up(sample_index) = state.Up;
    log.Ui(sample_index) = state.Ui;
    log.OutPreSat(sample_index) = state.OutPreSat;
    log.Out(sample_index) = state.Out;
end
end

function state = pid_reg3_step(state)
state.Err = state.Ref - state.Fdb;
state.Up = state.Kp * state.Err;

if state.OutPreSat > state.OutMax
    if state.Err < 0
        state.Ui = state.Ui + state.Ki * state.Err;
    else
        state.Ui = state.Ui + state.Kc * (state.OutMax - state.OutPreSat);
    end
elseif state.OutPreSat < state.OutMin
    if state.Err > 0
        state.Ui = state.Ui + state.Ki * state.Err;
    else
        state.Ui = state.Ui + state.Kc * (state.OutMin - state.OutPreSat);
    end
else
    state.Ui = state.Ui + state.Ki * state.Err;
end

state.Ui = min(state.OutMax, max(state.OutMin, state.Ui));
state.OutPreSat = state.Up + state.Ui;
state.Out = min(state.OutMax, max(state.OutMin, state.OutPreSat));
end

function state = standard_backcalc_step(state)
state.Err = state.Ref - state.Fdb;
state.Up = state.Kp * state.Err;

unsat = state.Up + state.Ui + state.Ki * state.Err;
sat = min(state.OutMax, max(state.OutMin, unsat));
state.Ui = state.Ui + state.Ki * state.Err + state.Kc * (sat - unsat);
state.OutPreSat = state.Up + state.Ui;
state.Out = min(state.OutMax, max(state.OutMin, state.OutPreSat));
end

function text = pass_fail(condition)
if condition
    text = 'PASS';
else
    text = 'FAIL';
end
end