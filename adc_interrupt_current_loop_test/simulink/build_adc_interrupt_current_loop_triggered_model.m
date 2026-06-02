%% Build a Simulink current-loop model with explicit interrupt triggers
% This model compares two timing architectures:
%   1. A fixed 50 us timer interrupt triggers the current PI loop.
%   2. A fixed 50 us HW_INT triggers the current PI loop through an
%      ADC-sample-hold style measurement path.
%
% The trigger sample time is a scalar 50e-6, not a [period offset] vector.

clear; clc;

script_dir = fileparts(mfilename('fullpath'));
root_dir = fileparts(script_dir);
results_dir = fullfile(root_dir, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

cfg = default_config();

model = 'adc_interrupt_current_loop_triggered';
model_file = fullfile(script_dir, [model '.slx']);

if bdIsLoaded(model)
    close_system(model, 0);
end

shadow_warning_state = warning('off', 'Simulink:Engine:MdlFileShadowing');
new_system(model);
warning(shadow_warning_state);

set_param(model, ...
    'StartTime', '0', ...
    'StopTime', num2str(cfg.T_end), ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', num2str(cfg.dt), ...
    'SystemTargetFile', 'grt.tlc', ...
    'ProdHWDeviceType', 'Intel->x86-64 (Linux 64)', ...
    'TargetHWDeviceType', 'Intel->x86-64 (Linux 64)', ...
    'ReturnWorkspaceOutputs', 'on', ...
    'SaveOutput', 'on', ...
    'SaveFormat', 'Dataset', ...
    'SignalLogging', 'on', ...
    'GenerateReport', 'on');

%% Shared reference
add_block('simulink/Sources/Step', [model '/i_ref_step'], ...
    'Position', [40 125 90 155], ...
    'Time', num2str(cfg.ref_step_time), ...
    'Before', num2str(cfg.i_ref_initial), ...
    'After', num2str(cfg.i_ref_final), ...
    'SampleTime', num2str(cfg.dt));

%% Fixed timer interrupt path: direct current feedback
add_function_call_generator(model, 'Fixed_50us_Timer_INT', ...
    [145 55 225 95], cfg.Tint, 0.0);
add_current_loop_isr(model, 'CurrentLoop_Timer_ISR', [255 55 415 135], cfg, 'current_loop_timer_isr');
add_rl_plant(model, 'Plant_without_ADC_delay', [540 55 700 135], cfg);

%% Fixed HW interrupt path: sample-hold measurement
add_block('simulink/Discrete/Zero-Order Hold', [model '/ADC_sample_hold'], ...
    'Position', [125 300 185 340], ...
    'SampleTime', sample_time_string(cfg.Tint, 0.0));

add_function_call_generator(model, 'Fixed_50us_HW_INT', ...
    [145 215 225 255], cfg.Tint, 0.0);
add_current_loop_isr(model, 'CurrentLoop_ADC_ISR', ...
    [255 215 415 295], cfg, 'current_loop_adc_isr');

add_block('simulink/Discrete/Zero-Order Hold', [model '/PWM_shadow_load'], ...
    'Position', [465 220 525 260], ...
    'SampleTime', sample_time_string(cfg.Tint, 0.0));
add_rl_plant(model, 'Plant_with_ADC_delay', [540 215 700 295], cfg);

%% Scopes and outputs
add_block('simulink/Signal Routing/Mux', [model '/current_mux'], ...
    'Position', [785 100 790 180], ...
    'Inputs', '3');
add_block('simulink/Sinks/Scope', [model '/current_scope'], ...
    'Position', [835 95 895 185]);

add_block('simulink/Signal Routing/Mux', [model '/voltage_mux'], ...
    'Position', [785 255 790 335], ...
    'Inputs', '3');
add_block('simulink/Sinks/Scope', [model '/voltage_scope'], ...
    'Position', [835 250 895 340]);

out_names = {'i_ref', 'i_without_adc_delay', 'i_with_adc_delay', ...
    'i_adc_sampled', 'v_without_adc_delay', 'v_adc_shadow', ...
    'v_with_adc_applied', 'pi_integrator_without', 'pi_integrator_with'};
for k = 1:numel(out_names)
    y = 410 + (k - 1)*35;
    add_block('simulink/Sinks/Out1', [model '/' out_names{k}], ...
        'Position', [885 y 915 y+20], ...
        'Port', num2str(k));
end

%% Wiring: fixed timer interrupt path
add_line(model, 'Fixed_50us_Timer_INT/1', 'CurrentLoop_Timer_ISR/trigger', 'autorouting', 'on');
add_line(model, 'i_ref_step/1', 'CurrentLoop_Timer_ISR/1', 'autorouting', 'on');
add_line(model, 'Plant_without_ADC_delay/1', 'CurrentLoop_Timer_ISR/2', 'autorouting', 'on');
add_line(model, 'CurrentLoop_Timer_ISR/1', 'Plant_without_ADC_delay/1', 'autorouting', 'on');

%% Wiring: ADC EOC interrupt path
add_line(model, 'Fixed_50us_HW_INT/1', 'CurrentLoop_ADC_ISR/trigger', 'autorouting', 'on');
add_line(model, 'i_ref_step/1', 'CurrentLoop_ADC_ISR/1', 'autorouting', 'on');
add_line(model, 'Plant_with_ADC_delay/1', 'ADC_sample_hold/1', 'autorouting', 'on');
add_line(model, 'ADC_sample_hold/1', 'CurrentLoop_ADC_ISR/2', 'autorouting', 'on');
add_line(model, 'CurrentLoop_ADC_ISR/1', 'PWM_shadow_load/1', 'autorouting', 'on');
add_line(model, 'PWM_shadow_load/1', 'Plant_with_ADC_delay/1', 'autorouting', 'on');

%% Wiring: scopes
add_line(model, 'i_ref_step/1', 'current_mux/1', 'autorouting', 'on');
add_line(model, 'Plant_without_ADC_delay/1', 'current_mux/2', 'autorouting', 'on');
add_line(model, 'Plant_with_ADC_delay/1', 'current_mux/3', 'autorouting', 'on');
add_line(model, 'current_mux/1', 'current_scope/1', 'autorouting', 'on');

add_line(model, 'CurrentLoop_Timer_ISR/1', 'voltage_mux/1', 'autorouting', 'on');
add_line(model, 'CurrentLoop_ADC_ISR/1', 'voltage_mux/2', 'autorouting', 'on');
add_line(model, 'PWM_shadow_load/1', 'voltage_mux/3', 'autorouting', 'on');
add_line(model, 'voltage_mux/1', 'voltage_scope/1', 'autorouting', 'on');

%% Wiring: output ports
name_line(add_line(model, 'i_ref_step/1', 'i_ref/1', 'autorouting', 'on'), 'i_ref');
name_line(add_line(model, 'Plant_without_ADC_delay/1', 'i_without_adc_delay/1', 'autorouting', 'on'), 'i_without_adc_delay');
name_line(add_line(model, 'Plant_with_ADC_delay/1', 'i_with_adc_delay/1', 'autorouting', 'on'), 'i_with_adc_delay');
name_line(add_line(model, 'ADC_sample_hold/1', 'i_adc_sampled/1', 'autorouting', 'on'), 'i_adc_sampled');
name_line(add_line(model, 'CurrentLoop_Timer_ISR/1', 'v_without_adc_delay/1', 'autorouting', 'on'), 'v_without_adc_delay');
name_line(add_line(model, 'CurrentLoop_ADC_ISR/1', 'v_adc_shadow/1', 'autorouting', 'on'), 'v_adc_shadow');
name_line(add_line(model, 'PWM_shadow_load/1', 'v_with_adc_applied/1', 'autorouting', 'on'), 'v_with_adc_applied');
name_line(add_line(model, 'CurrentLoop_Timer_ISR/2', 'pi_integrator_without/1', 'autorouting', 'on'), 'pi_integrator_without');
name_line(add_line(model, 'CurrentLoop_ADC_ISR/2', 'pi_integrator_with/1', 'autorouting', 'on'), 'pi_integrator_with');

Simulink.BlockDiagram.arrangeSystem(model);
set_param(model, 'SimulationCommand', 'update');
save_system(model, model_file);

fprintf('Built triggered Simulink model:\n  %s\n', model_file);
fprintf('Fixed interrupt period     = %.3f us\n', cfg.Tint*1e6);
fprintf('Function-call sample time  = %s s\n', sample_time_string(cfg.Tint, 0.0));

%% Local helpers
function cfg = default_config()
    cfg.R = 0.5;
    cfg.L = 1.0e-3;
    cfg.V_limit = 24.0;

    cfg.Tint = 50e-6;
    cfg.Tpwm = cfg.Tint;
    cfg.dt = 1e-6;
    cfg.T_end = 20e-3;

    cfg.ref_step_time = 2e-3;
    cfg.i_ref_initial = 0.0;
    cfg.i_ref_final = 10.0;

    cfg.current_loop_bw_hz = 800.0;
    wc = 2*pi*cfg.current_loop_bw_hz;
    cfg.Kp = cfg.L * wc;
    cfg.Ki = cfg.R * wc;
    cfg.integrator_limit = cfg.V_limit;

    cfg.adc_sample_phase = 0.0;
    cfg.adc_conversion_delay = 0.0;
end

function add_function_call_generator(model, name, position, period, offset)
    block = [model '/' name];
    add_block(sprintf('simulink/Ports &\nSubsystems/Function-Call\nGenerator'), ...
        block, ...
        'Position', position, ...
        'sample_time', sample_time_string(period, offset));
end

function ts = sample_time_string(period, offset)
    if abs(offset) < eps
        ts = sprintf('%.12g', period);
    else
        ts = sprintf('[%.12g %.12g]', period, offset);
    end
end

function add_current_loop_isr(model, name, position, cfg, function_name)
    subsystem = [model '/' name];
    add_block(sprintf('simulink/Ports &\nSubsystems/Function-Call\nSubsystem'), ...
        subsystem, ...
        'Position', position);
    configure_reusable_function(subsystem, function_name);

    set_param([subsystem '/In1'], 'Name', 'i_ref', 'Position', [35 65 65 85]);
    set_param([subsystem '/Out1'], 'Name', 'v_cmd', 'Position', [395 70 425 90]);
    set_param([subsystem '/function'], 'Position', [190 15 210 35]);

    safe_delete_line(subsystem, 'i_ref/1', 'v_cmd/1');

    add_block('simulink/Sources/In1', [subsystem '/i_meas'], ...
        'Position', [35 135 65 155], ...
        'Port', '2');
    add_block('simulink/Sinks/Out1', [subsystem '/integral_state'], ...
        'Position', [395 135 425 155], ...
        'Port', '2');

    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [subsystem '/PI_Core'], ...
        'Position', [150 65 320 165]);
    set_matlab_function_code([subsystem '/PI_Core'], current_pi_code(cfg));

    add_line(subsystem, 'i_ref/1', 'PI_Core/1', 'autorouting', 'on');
    add_line(subsystem, 'i_meas/1', 'PI_Core/2', 'autorouting', 'on');
    add_line(subsystem, 'PI_Core/1', 'v_cmd/1', 'autorouting', 'on');
    add_line(subsystem, 'PI_Core/2', 'integral_state/1', 'autorouting', 'on');

    Simulink.BlockDiagram.arrangeSystem(subsystem);
end

function code = current_pi_code(cfg)
    code = sprintf([ ...
        'function [v_cmd, integral_state] = fcn(i_ref, i_meas)\n' ...
        '%%#codegen\n' ...
        'persistent integrator\n' ...
        'if isempty(integrator)\n' ...
        '    integrator = 0.0;\n' ...
        'end\n' ...
        'Ts = %.17g;\n' ...
        'Kp = %.17g;\n' ...
        'Ki = %.17g;\n' ...
        'V_limit = %.17g;\n' ...
        'integrator_limit = %.17g;\n' ...
        'e = i_ref - i_meas;\n' ...
        'p_term = Kp * e;\n' ...
        'integrator = integrator + Ki * Ts * e;\n' ...
        'integrator = min(max(integrator, -integrator_limit), integrator_limit);\n' ...
        'v_unsat = p_term + integrator;\n' ...
        'v_cmd = min(max(v_unsat, -V_limit), V_limit);\n' ...
        'if v_cmd ~= v_unsat\n' ...
        '    integrator = v_cmd - p_term;\n' ...
        '    integrator = min(max(integrator, -integrator_limit), integrator_limit);\n' ...
        'end\n' ...
        'integral_state = integrator;\n' ...
        'end\n'], ...
        cfg.Tint, cfg.Kp, cfg.Ki, cfg.V_limit, cfg.integrator_limit);
end

function add_rl_plant(model, name, position, cfg)
    subsystem = [model '/' name];
    add_block('built-in/Subsystem', subsystem, 'Position', position);

    add_block('simulink/Sources/In1', [subsystem '/v_applied'], ...
        'Position', [35 70 65 90]);
    add_block('simulink/Math Operations/Sum', [subsystem '/v_minus_Ri'], ...
        'Position', [105 65 130 95], ...
        'Inputs', '+-');
    add_block('simulink/Math Operations/Gain', [subsystem '/one_over_L'], ...
        'Position', [170 65 225 95], ...
        'Gain', num2str(1/cfg.L, 17));
    add_block('simulink/Discrete/Discrete-Time Integrator', [subsystem '/current_integrator'], ...
        'Position', [265 65 325 95], ...
        'gainval', '1', ...
        'InitialCondition', '0', ...
        'SampleTime', num2str(cfg.dt, 17));
    add_block('simulink/Math Operations/Gain', [subsystem '/R_gain'], ...
        'Position', [265 145 325 175], ...
        'Gain', num2str(cfg.R, 17));
    add_block('simulink/Sinks/Out1', [subsystem '/i'], ...
        'Position', [390 70 420 90]);

    add_line(subsystem, 'v_applied/1', 'v_minus_Ri/1', 'autorouting', 'on');
    add_line(subsystem, 'v_minus_Ri/1', 'one_over_L/1', 'autorouting', 'on');
    add_line(subsystem, 'one_over_L/1', 'current_integrator/1', 'autorouting', 'on');
    add_line(subsystem, 'current_integrator/1', 'i/1', 'autorouting', 'on');
    add_line(subsystem, 'current_integrator/1', 'R_gain/1', 'autorouting', 'on');
    add_line(subsystem, 'R_gain/1', 'v_minus_Ri/2', 'autorouting', 'on');

    Simulink.BlockDiagram.arrangeSystem(subsystem);
end

function set_matlab_function_code(block_path, code)
    rt = sfroot;
    chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', block_path);
    if isempty(chart)
        error('Could not find MATLAB Function chart: %s', block_path);
    end
    chart.Script = code;
end

function configure_reusable_function(block_path, function_name)
    set_param(block_path, ...
        'RTWSystemCode', 'Reusable function', ...
        'RTWFcnNameOpts', 'User specified', ...
        'RTWFcnName', function_name, ...
        'RTWFileNameOpts', 'User specified', ...
        'RTWFileName', function_name);
end

function safe_delete_line(system, src, dst)
    try
        delete_line(system, src, dst);
    catch
    end
end

function name_line(line_handle, signal_name)
    set_param(line_handle, 'Name', signal_name);
end
