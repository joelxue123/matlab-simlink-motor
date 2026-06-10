%% Function test for motor_clarke_park_model
%
% This test runs the Simulink module with several known abc/theta cases and
% compares the output bus fields against a MATLAB reference calculation.

clear; clc;

script_dir = fileparts(mfilename('fullpath'));
harness = 'motor_clarke_park_function_test_harness';

if bdIsLoaded(harness)
    close_system(harness, 0);
end

try
    open_paths = Simulink.data.dictionary.getOpenDictionaryPaths;
catch
    open_paths = {};
end

for i = 1:numel(open_paths)
    try
        dd = Simulink.data.dictionary.open(open_paths{i});
        discardChanges(dd);
        close(dd);
    catch err
        warning('Could not discard and close data dictionary "%s": %s', ...
            open_paths{i}, err.message);
    end
end

try
    Simulink.data.dictionary.closeAll;
catch err
    warning('Could not close open data dictionaries before test rebuild: %s', ...
        err.message);
end

run(fullfile(script_dir, 'build_motor_clarke_park_model.m'));

cfg = evalin('base', 'motor_clarke_park_codegen_config');
model = cfg.modelName;
model_file = fullfile(script_dir, [model '.slx']);
harness_file = fullfile(script_dir, [harness '.slx']);

load_system(model_file);

if bdIsLoaded(harness)
    close_system(harness, 0);
end

if exist(harness_file, 'file')
    delete(harness_file);
end

new_system(harness);
set_param(harness, ...
    'DataDictionary', cfg.dictionaryName, ...
    'StopTime', cfg.sampleTime, ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', cfg.sampleTime);

add_input_constant(harness, 'ia', [45 45 95 75], cfg.currentTypeName, cfg.sampleTime);
add_input_constant(harness, 'ib', [45 95 95 125], cfg.currentTypeName, cfg.sampleTime);
add_input_constant(harness, 'ic', [45 145 95 175], cfg.currentTypeName, cfg.sampleTime);
add_input_constant(harness, 'theta_e', [45 195 95 225], cfg.angleTypeName, cfg.sampleTime);

add_block('simulink/Signal Routing/Bus Creator', ...
    [harness '/motor_bus_creator'], ...
    'Position', [155 50 170 220], ...
    'Inputs', '4', ...
    'UseBusObject', 'on', ...
    'BusObject', cfg.inputBusName, ...
    'NonVirtualBus', 'on');

add_block([model '/' cfg.functionName], [harness '/' cfg.functionName], ...
    'Position', [235 80 395 160]);

add_block('simulink/Signal Routing/Bus Selector', [harness '/dq_selector'], ...
    'Position', [455 55 470 190], ...
    'OutputSignals', 'i_alpha,i_beta,id,iq');

add_to_workspace(harness, 'i_alpha_actual', [535 45 625 75]);
add_to_workspace(harness, 'i_beta_actual', [535 85 625 115]);
add_to_workspace(harness, 'id_actual', [535 125 625 155]);
add_to_workspace(harness, 'iq_actual', [535 165 625 195]);

add_display(harness, 'i_alpha_display', [675 45 765 75]);
add_display(harness, 'i_beta_display', [675 85 765 115]);
add_display(harness, 'id_display', [675 125 765 155]);
add_display(harness, 'iq_display', [675 165 765 195]);

name_line(add_line(harness, 'ia/1', 'motor_bus_creator/1', ...
    'autorouting', 'on'), 'ia');
name_line(add_line(harness, 'ib/1', 'motor_bus_creator/2', ...
    'autorouting', 'on'), 'ib');
name_line(add_line(harness, 'ic/1', 'motor_bus_creator/3', ...
    'autorouting', 'on'), 'ic');
name_line(add_line(harness, 'theta_e/1', 'motor_bus_creator/4', ...
    'autorouting', 'on'), 'theta_e');
add_line(harness, 'motor_bus_creator/1', [cfg.functionName '/1'], 'autorouting', 'on');
add_line(harness, [cfg.functionName '/1'], 'dq_selector/1', 'autorouting', 'on');
add_line(harness, 'dq_selector/1', 'i_alpha_actual/1', 'autorouting', 'on');
add_line(harness, 'dq_selector/2', 'i_beta_actual/1', 'autorouting', 'on');
add_line(harness, 'dq_selector/3', 'id_actual/1', 'autorouting', 'on');
add_line(harness, 'dq_selector/4', 'iq_actual/1', 'autorouting', 'on');
add_line(harness, 'dq_selector/1', 'i_alpha_display/1', 'autorouting', 'on');
add_line(harness, 'dq_selector/2', 'i_beta_display/1', 'autorouting', 'on');
add_line(harness, 'dq_selector/3', 'id_display/1', 'autorouting', 'on');
add_line(harness, 'dq_selector/4', 'iq_display/1', 'autorouting', 'on');

test_cases = [ ...
    struct('name', 'alpha_axis_theta_0', ...
        'ia', 1.0, 'ib', -0.5, 'ic', -0.5, 'theta_e', 0.0); ...
    struct('name', 'alpha_axis_theta_90deg', ...
        'ia', 1.0, 'ib', -0.5, 'ic', -0.5, 'theta_e', pi/2); ...
    struct('name', 'beta_axis_theta_0', ...
        'ia', 0.0, 'ib', sqrt(3)/2, 'ic', -sqrt(3)/2, 'theta_e', 0.0); ...
    struct('name', 'arbitrary_theta_30deg', ...
        'ia', 2.0, 'ib', -1.0, 'ic', -1.0, 'theta_e', pi/6)];

tolerance = 4.0 / 2^cfg.currentFractionLength;
max_error = 0;

fprintf('\nRunning motor Clarke/Park functional test:\n');
fprintf('  Model: %s\n', model_file);
fprintf('  Harness: %s\n', harness_file);
fprintf('  Cases: %d\n\n', numel(test_cases));

save_system(harness, harness_file);

for k = 1:numel(test_cases)
    c = test_cases(k);
    set_input_values(harness, c);

    sim_out = sim(harness, ...
        'ReturnWorkspaceOutputs', 'on', ...
        'SaveOutput', 'off');

    actual = struct( ...
        'i_alpha', last_sample(sim_out.get('i_alpha_actual')), ...
        'i_beta', last_sample(sim_out.get('i_beta_actual')), ...
        'id', last_sample(sim_out.get('id_actual')), ...
        'iq', last_sample(sim_out.get('iq_actual')));
    expected = reference_clarke_park(c.ia, c.ib, c.ic, c.theta_e, cfg);

    err = max(abs([ ...
        actual.i_alpha - expected.i_alpha, ...
        actual.i_beta - expected.i_beta, ...
        actual.id - expected.id, ...
        actual.iq - expected.iq]));
    max_error = max(max_error, err);

    fprintf('  %-24s  max_error = %.3g\n', c.name, err);
    fprintf('    actual:   alpha=% .8f beta=% .8f id=% .8f iq=% .8f\n', ...
        actual.i_alpha, actual.i_beta, actual.id, actual.iq);
    fprintf('    expected: alpha=% .8f beta=% .8f id=% .8f iq=% .8f\n', ...
        expected.i_alpha, expected.i_beta, expected.id, expected.iq);
end

save_system(harness, harness_file);

fprintf('\nMaximum error: %.3g\n', max_error);
if max_error > tolerance
    error('Motor Clarke/Park functional test failed. Tolerance: %.3g', tolerance);
end

fprintf('Motor Clarke/Park functional test passed.\n');
fprintf('Saved visual test harness:\n  %s\n', harness_file);

function add_input_constant(model, block_name, position, data_type, sample_time)
add_block('simulink/Sources/Constant', [model '/' block_name], ...
    'Position', position, ...
    'Value', '0', ...
    'OutDataTypeStr', data_type, ...
    'SampleTime', sample_time);
end

function add_to_workspace(model, variable_name, position)
add_block('simulink/Sinks/To Workspace', [model '/' variable_name], ...
    'Position', position, ...
    'VariableName', variable_name, ...
    'SaveFormat', 'Timeseries');
end

function add_display(model, block_name, position)
add_block('simulink/Sinks/Display', [model '/' block_name], ...
    'Position', position);
end

function set_input_values(model, c)
set_param([model '/ia'], 'Value', num2str(c.ia, '%.9g'));
set_param([model '/ib'], 'Value', num2str(c.ib, '%.9g'));
set_param([model '/ic'], 'Value', num2str(c.ic, '%.9g'));
set_param([model '/theta_e'], 'Value', num2str(single(c.theta_e), '%.9g'));
end

function name_line(line_handle, signal_name)
set_param(line_handle, 'Name', signal_name);
end

function value = last_sample(signal)
value = signal.Data(end);
end

function out = reference_clarke_park(ia, ib, ic, theta_e, cfg)
ia = quantize_current(ia, cfg);
ib = quantize_current(ib, cfg);
ic = quantize_current(ic, cfg);

i_alpha = quantize_current((2/3) * (ia - 0.5*ib - 0.5*ic), cfg);
i_beta = quantize_current((1/sqrt(3)) * (ib - ic), cfg);
cos_t = single(cos(theta_e));
sin_t = single(sin(theta_e));

out = struct;
out.i_alpha = i_alpha;
out.i_beta = i_beta;
out.id = quantize_current(i_alpha * cos_t + i_beta * sin_t, cfg);
out.iq = quantize_current(-i_alpha * sin_t + i_beta * cos_t, cfg);
end

function y = quantize_current(x, cfg)
scale = 2^cfg.currentFractionLength;
max_raw = 2^(cfg.currentWordLength - 1) - 1;
min_raw = -2^(cfg.currentWordLength - 1);
raw = floor(double(x) * scale);
raw = min(max(raw, min_raw), max_raw);
y = raw / scale;
end
