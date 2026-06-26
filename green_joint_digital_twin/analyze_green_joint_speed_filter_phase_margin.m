%% Analyze green-joint speed-estimator filter phase margin impact
%
% Firmware source of truth:
%   Core/Src/main.c speed estimate path
%     motor angle finite difference, Ts_est = SPEED_SAMPLE_BASE_DT_S * SPD_CNT
%     joint_speed_est = alpha * joint_speed_raw + (1 - alpha) * joint_speed_est
%
% This script keeps the analysis independent from Control System Toolbox:
% it evaluates the open-loop frequency response directly on a frequency grid.

clear;
clc;

script_dir = fileparts(mfilename('fullpath'));
workspace_dir = fileparts(fileparts(script_dir));
green_joint_dir = fullfile(workspace_dir, 'green-joint');
results_dir = fullfile(script_dir, 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end

cfg.module_ids = ["1615", "1620"];
cfg.pwm_freq_hz = 40000.0;
cfg.adc_isr_divider = 2.0;
cfg.speed_estimator_count = 8.0;
cfg.current_loop_sample_time_s = cfg.adc_isr_divider / cfg.pwm_freq_hz;
cfg.speed_estimator_sample_time_s = ...
    cfg.current_loop_sample_time_s * cfg.speed_estimator_count;
cfg.production_alpha = 0.1;
cfg.alpha_sweep = [0.05 0.1 0.2 0.3 0.5];
cfg.speed_loop_read_delay_s = 0.5 * 500e-6;
cfg.current_loop_delay_s = 1.5 * cfg.current_loop_sample_time_s;
cfg.frequency_hz = logspace(log10(0.5), log10(500.0), 2400).';

case_defs = [ ...
    make_case("no_filter", NaN, false, false); ...
    make_case("diff_window_only", NaN, true, false)];
for alpha = cfg.alpha_sweep
    case_defs = [case_defs; ...
        make_case(sprintf("diff_plus_iir_alpha_%s", alpha_token(alpha)), ...
        alpha, true, true)]; %#ok<AGROW>
end

rows = struct([]);
freq_rows = struct([]);

for module_id = cfg.module_ids
    module_cfg = load_module_config(green_joint_dir, module_id);
    base_result = analyze_margin(module_cfg, cfg, case_defs(1));

    for i = 1:numel(case_defs)
        result = analyze_margin(module_cfg, cfg, case_defs(i));
        result.phase_margin_loss_vs_no_filter_deg = ...
            base_result.phase_margin_deg - result.phase_margin_deg;
        rows = [rows; result]; %#ok<AGROW>
    end

    production_case = make_case("production_diff_plus_iir_alpha_0p1", ...
        cfg.production_alpha, true, true);
    freq_result = evaluate_frequency_response(module_cfg, cfg, production_case);
    for k = 1:numel(freq_result.frequency_hz)
        freq_rows = [freq_rows; struct( ... %#ok<AGROW>
            'module_id', string(module_cfg.module_id), ...
            'frequency_hz', freq_result.frequency_hz(k), ...
            'open_loop_gain_db', freq_result.open_loop_gain_db(k), ...
            'open_loop_phase_deg', freq_result.open_loop_phase_deg(k), ...
            'measurement_gain_db', freq_result.measurement_gain_db(k), ...
            'measurement_phase_deg', freq_result.measurement_phase_deg(k))];
    end
end

summary_table = struct2table(rows);
summary_file = fullfile(results_dir, ...
    'green_joint_speed_filter_phase_margin_summary.csv');
writetable(summary_table, summary_file);

frequency_table = struct2table(freq_rows);
frequency_file = fullfile(results_dir, ...
    'green_joint_speed_filter_phase_margin_frequency_response.csv');
writetable(frequency_table, frequency_file);

plot_file = fullfile(results_dir, ...
    'green_joint_speed_filter_phase_margin.png');
plot_summary(summary_table, plot_file);

fprintf('\nGreen-joint speed-filter phase margin analysis\n');
fprintf('  Ts_current             = %.9g us\n', ...
    cfg.current_loop_sample_time_s * 1e6);
fprintf('  Ts_estimator           = %.9g us\n', ...
    cfg.speed_estimator_sample_time_s * 1e6);
fprintf('  production alpha       = %.9g\n', cfg.production_alpha);
fprintf('  current-loop delay     = %.9g us\n', ...
    cfg.current_loop_delay_s * 1e6);
fprintf('  speed-loop read delay  = %.9g us\n', ...
    cfg.speed_loop_read_delay_s * 1e6);
fprintf('  summary                = %s\n', summary_file);
fprintf('  frequency response     = %s\n', frequency_file);
fprintf('  plot                   = %s\n\n', plot_file);

disp(summary_table(:, {'module_id', 'filter_case', ...
    'alpha', 'gain_crossover_hz', 'phase_margin_deg', ...
    'phase_margin_loss_vs_no_filter_deg', ...
    'measurement_phase_at_speed_bw_deg'}));

function case_def = make_case(name, alpha, include_diff_window, include_iir)
case_def = struct( ...
    'name', string(name), ...
    'alpha', alpha, ...
    'include_diff_window', include_diff_window, ...
    'include_iir', include_iir);
end

function token = alpha_token(alpha)
token = strrep(sprintf('%.3g', alpha), '.', 'p');
end

function module_cfg = load_module_config(green_joint_dir, module_id)
config_file = fullfile(green_joint_dir, 'Module', 'Config', ...
    ['green_joint_' char(module_id) '_config.json']);
if ~exist(config_file, 'file')
    error('Missing module config: %s', config_file);
end
module_cfg = jsondecode(fileread(config_file));
end

function result = analyze_margin(module_cfg, cfg, case_def)
freq_result = evaluate_frequency_response(module_cfg, cfg, case_def);

gain = 10 .^ (freq_result.open_loop_gain_db / 20.0);
cross_index = find(gain(1:end - 1) >= 1.0 & gain(2:end) < 1.0, 1);
if isempty(cross_index)
    gain_crossover_hz = NaN;
    phase_at_crossover_deg = NaN;
    phase_margin_deg = NaN;
    measurement_phase_at_crossover_deg = NaN;
else
    x = log(freq_result.frequency_hz);
    y = log(gain);
    x_gc = interp1(y(cross_index:cross_index + 1), ...
        x(cross_index:cross_index + 1), 0.0, 'linear');
    gain_crossover_hz = exp(x_gc);
    phase_at_crossover_deg = interp1(x, ...
        freq_result.open_loop_phase_deg, x_gc, 'linear');
    phase_margin_deg = 180.0 + phase_at_crossover_deg;
    measurement_phase_at_crossover_deg = interp1(x, ...
        freq_result.measurement_phase_deg, x_gc, 'linear');
end

speed_bw_hz = module_cfg.speed_loop.bandwidth_hz;
measurement_phase_at_speed_bw_deg = interp1( ...
    freq_result.frequency_hz, freq_result.measurement_phase_deg, ...
    speed_bw_hz, 'linear');
measurement_gain_at_speed_bw_db = interp1( ...
    freq_result.frequency_hz, freq_result.measurement_gain_db, ...
    speed_bw_hz, 'linear');

result = struct( ...
    'module_id', string(module_cfg.module_id), ...
    'filter_case', case_def.name, ...
    'alpha', case_def.alpha, ...
    'speed_loop_bandwidth_hz', module_cfg.speed_loop.bandwidth_hz, ...
    'speed_loop_kp', module_cfg.speed_loop.speed_kp, ...
    'speed_loop_ki', module_cfg.speed_loop.speed_ki, ...
    'speed_loop_kaw', module_cfg.speed_loop.speed_kaw, ...
    'speed_estimator_sample_time_us', ...
        cfg.speed_estimator_sample_time_s * 1e6, ...
    'gain_crossover_hz', gain_crossover_hz, ...
    'phase_at_crossover_deg', phase_at_crossover_deg, ...
    'phase_margin_deg', phase_margin_deg, ...
    'measurement_phase_at_crossover_deg', ...
        measurement_phase_at_crossover_deg, ...
    'measurement_phase_at_speed_bw_deg', ...
        measurement_phase_at_speed_bw_deg, ...
    'measurement_gain_at_speed_bw_db', ...
        measurement_gain_at_speed_bw_db, ...
    'phase_margin_loss_vs_no_filter_deg', NaN);
end

function freq_result = evaluate_frequency_response(module_cfg, cfg, case_def)
frequency_hz = cfg.frequency_hz;
omega = 2 * pi * frequency_hz;
s = 1j * omega;

ts_speed = module_cfg.speed_loop.sample_time_s;
z_speed = exp(s * ts_speed);
speed_pi = module_cfg.speed_loop.speed_kp + ...
    module_cfg.speed_loop.speed_ki * ts_speed ./ (z_speed - 1.0);

gear_ratio = module_cfg.gear_ratio;
speed_loop_equiv_inertia = ...
    module_cfg.rotor_inertia_kg_m2 * gear_ratio;
plant_iq_to_joint_speed = module_cfg.torque_constant_nm_per_a ./ ...
    (speed_loop_equiv_inertia * s);

current_bandwidth_rad_s = ...
    2 * pi * module_cfg.current_loop.reference_bandwidth_hz;
current_loop = 1.0 ./ (1.0 + s ./ current_bandwidth_rad_s);

measurement = speed_measurement_filter(s, cfg, case_def);
delay = exp(-s * (cfg.current_loop_delay_s + cfg.speed_loop_read_delay_s));

open_loop = speed_pi .* current_loop .* plant_iq_to_joint_speed .* ...
    measurement .* delay;

freq_result = struct( ...
    'frequency_hz', frequency_hz, ...
    'open_loop_gain_db', 20 * log10(abs(open_loop)), ...
    'open_loop_phase_deg', unwrap(angle(open_loop)) * 180 / pi, ...
    'measurement_gain_db', 20 * log10(abs(measurement)), ...
    'measurement_phase_deg', unwrap(angle(measurement)) * 180 / pi);
end

function measurement = speed_measurement_filter(s, cfg, case_def)
measurement = ones(size(s));

if case_def.include_diff_window
    x = s * cfg.speed_estimator_sample_time_s;
    diff_window = (1.0 - exp(-x)) ./ x;
    diff_window(abs(x) < 1e-12) = 1.0;
    measurement = measurement .* diff_window;
end

if case_def.include_iir
    z_est = exp(s * cfg.speed_estimator_sample_time_s);
    measurement = measurement .* ...
        (case_def.alpha ./ (z_est - (1.0 - case_def.alpha)) .* z_est);
end
end

function plot_summary(summary_table, plot_file)
production_rows = contains(summary_table.filter_case, ...
    "diff_plus_iir_alpha_0p1");

figure_handle = figure('Visible', 'off');
bar(categorical(summary_table.module_id(production_rows)), ...
    summary_table.phase_margin_deg(production_rows));
grid on;
ylabel('Phase margin (deg)');
title('green-joint speed loop with production speed filter');
saveas(figure_handle, plot_file);
close(figure_handle);
end
