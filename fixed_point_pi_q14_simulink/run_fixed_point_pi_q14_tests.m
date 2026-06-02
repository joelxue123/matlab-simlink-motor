%% Run fixed-point Q14 PI model tests and generate report

clear; clc; close all;

script_dir = fileparts(mfilename('fullpath'));
run(fullfile(script_dir, 'build_fixed_point_pi_q14_model.m'));

script_dir = fileparts(mfilename('fullpath'));
results_dir = fullfile(script_dir, 'results');
fig_dir = fullfile(results_dir, 'figures');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end
if ~exist(fig_dir, 'dir')
    mkdir(fig_dir);
end

model = 'fixed_point_pi_q14';
load_system(model);

q14_lsb = 2^-14;
iq24_lsb = 2^-24;
u_limit_tol = 2 * q14_lsb;
u_max_q14 = quant_floor_sat(1.8, 16, 14);
u_min_q14 = -u_max_q14;

tests = struct( ...
    'id', {'small_signal', 'large_signal', 'saturation', 'truncation', 'overflow'}, ...
    'name', {'小信号测试', '大信号测试', '饱和测试', '截断测试', '溢出测试'}, ...
    'description', { ...
        '小误差闭环输入，验证无饱和时 PI 输出、误差和积分状态正常。', ...
        '较大但不应饱和的误差输入，验证动态范围和积分累加正常。', ...
        '大误差输入，验证输出限幅和抗积分饱和逻辑。', ...
        '非 Q14 整点输入，验证输入和误差按 Floor 方式量化。', ...
        '超出 Q14 输入范围的极端输入，验证定点转换饱和而不回绕。'}, ...
    'ref', {0.02, 0.15, 1.0, 0.123456, 3.0}, ...
    'feedback', {0.015, 0.02, 0.2, 0.023456, -3.0});

results = repmat(empty_result(), numel(tests), 1);
all_waveforms = cell(numel(tests), 1);

for k = 1:numel(tests)
    set_param([model '/ref_step'], 'After', num2str(tests(k).ref, '%.17g'));
    set_param([model '/feedback_step'], 'After', num2str(tests(k).feedback, '%.17g'));
    set_param(model, 'SimulationCommand', 'update');

    sim_out = sim(model, 'StopTime', '0.02', 'ReturnWorkspaceOutputs', 'on');
    waveform = collect_waveform(sim_out);
    all_waveforms{k} = waveform;

    results(k) = evaluate_result(tests(k), waveform, q14_lsb, iq24_lsb, u_max_q14, u_min_q14, u_limit_tol);
    plot_single_case(tests(k), waveform, results(k), fig_dir);
end

plot_summary(tests, all_waveforms, fig_dir);

save(fullfile(results_dir, 'fixed_point_pi_q14_test_results.mat'), ...
    'tests', 'results', 'all_waveforms', 'q14_lsb', 'iq24_lsb', 'u_max_q14', 'u_min_q14');

codegen_status = run_codegen_check(model, script_dir);
write_report(tests, results, codegen_status, results_dir, fig_dir, q14_lsb, iq24_lsb, u_max_q14, u_min_q14);

fprintf('Fixed-point PI tests finished.\n');
fprintf('  Report:  %s\n', fullfile(results_dir, 'fixed_point_pi_q14_test_report.md'));
fprintf('  Figures: %s\n', fig_dir);

function result = empty_result()
result = struct( ...
    'pass', false, ...
    'criterion', '', ...
    'final_u', NaN, ...
    'final_error', NaN, ...
    'final_integral', NaN, ...
    'peak_abs_u', NaN, ...
    'peak_abs_error', NaN, ...
    'sat_ratio', NaN, ...
    'expected_error_q14', NaN, ...
    'error_quantization_abs_error', NaN, ...
    'input_ref_q14', NaN, ...
    'input_feedback_q14', NaN, ...
    'has_nan_or_inf', true);
end

function waveform = collect_waveform(sim_out)
u_log = sim_out.get('u_log');
error_log = sim_out.get('error_log');
integral_log = sim_out.get('integral_log');
sat_log = sim_out.get('sat_log');

waveform.t = u_log.Time(:);
waveform.u = double(squeeze(u_log.Data));
waveform.error = double(squeeze(error_log.Data));
waveform.integral = double(squeeze(integral_log.Data));
waveform.sat = logical(squeeze(sat_log.Data));

waveform.u = waveform.u(:);
waveform.error = waveform.error(:);
waveform.integral = waveform.integral(:);
waveform.sat = waveform.sat(:);
end

function result = evaluate_result(test, waveform, q14_lsb, iq24_lsb, u_max_q14, u_min_q14, u_limit_tol)
result = empty_result();
result.final_u = waveform.u(end);
result.final_error = waveform.error(end);
result.final_integral = waveform.integral(end);
result.peak_abs_u = max(abs(waveform.u));
result.peak_abs_error = max(abs(waveform.error));
result.sat_ratio = mean(waveform.sat);
result.input_ref_q14 = quant_floor_sat(test.ref, 16, 14);
result.input_feedback_q14 = quant_floor_sat(test.feedback, 16, 14);
result.expected_error_q14 = quant_floor_sat(result.input_ref_q14 - result.input_feedback_q14, 16, 14);
result.error_quantization_abs_error = abs(result.final_error - result.expected_error_q14);
result.has_nan_or_inf = any(~isfinite(waveform.u)) || any(~isfinite(waveform.error)) || any(~isfinite(waveform.integral));

inside_output_limit = all(waveform.u <= u_max_q14 + u_limit_tol) && all(waveform.u >= u_min_q14 - u_limit_tol);
no_invalid_number = ~result.has_nan_or_inf;

switch test.id
    case 'small_signal'
        result.criterion = '无 NaN/Inf，输出不饱和，最终输出小于 0.2，误差 Q14 量化正确。';
        result.pass = no_invalid_number && inside_output_limit && result.sat_ratio == 0 && ...
            abs(result.final_u) < 0.2 && result.error_quantization_abs_error <= q14_lsb;
    case 'large_signal'
        result.criterion = '无 NaN/Inf，输出不饱和，峰值输出大于 0.8 且低于限幅，误差 Q14 量化正确。';
        result.pass = no_invalid_number && inside_output_limit && result.sat_ratio == 0 && ...
            result.peak_abs_u > 0.8 && result.peak_abs_u < u_max_q14 && result.error_quantization_abs_error <= q14_lsb;
    case 'saturation'
        result.criterion = '无 NaN/Inf，输出进入饱和，最终输出贴近正限幅，积分状态被抗饱和逻辑保持。';
        result.pass = no_invalid_number && inside_output_limit && result.sat_ratio > 0.5 && ...
            abs(result.final_u - u_max_q14) <= u_limit_tol && abs(result.final_integral) <= iq24_lsb;
    case 'truncation'
        result.criterion = '无 NaN/Inf，误差等于 ref 和 feedback 分别 Floor 到 Q14 后的差值再 Floor 到 Q14。';
        result.pass = no_invalid_number && inside_output_limit && result.sat_ratio == 0 && ...
            result.error_quantization_abs_error <= q14_lsb;
    case 'overflow'
        result.criterion = '无 NaN/Inf，超范围输入被定点饱和，不发生回绕，输出保持在限幅内并置位饱和标志。';
        result.pass = no_invalid_number && inside_output_limit && result.sat_ratio > 0.5 && ...
            result.peak_abs_error <= 2.0 + q14_lsb && abs(result.final_u - u_max_q14) <= u_limit_tol;
    otherwise
        result.criterion = '未定义。';
        result.pass = false;
end
end

function plot_single_case(test, waveform, result, fig_dir)
fig = figure('Visible', 'off', 'Name', test.name, 'Color', 'w');
tiledlayout(fig, 4, 1, 'TileSpacing', 'compact');

nexttile;
plot(waveform.t, waveform.u, 'LineWidth', 1.2);
grid on;
ylabel('u Q14');
title(sprintf('%s - %s', test.name, pass_text(result.pass)));

nexttile;
plot(waveform.t, waveform.error, 'LineWidth', 1.2);
grid on;
ylabel('error Q14');

nexttile;
plot(waveform.t, waveform.integral, 'LineWidth', 1.2);
grid on;
ylabel('integral IQ24');

nexttile;
stairs(waveform.t, double(waveform.sat), 'LineWidth', 1.2);
grid on;
ylabel('sat');
xlabel('Time (s)');

exportgraphics(fig, fullfile(fig_dir, [test.id '.png']), 'Resolution', 160);
close(fig);
end

function plot_summary(tests, all_waveforms, fig_dir)
fig = figure('Visible', 'off', 'Name', 'Fixed-point PI test summary', 'Color', 'w');
tiledlayout(fig, 3, 1, 'TileSpacing', 'compact');

nexttile;
hold on;
for k = 1:numel(tests)
    plot(all_waveforms{k}.t, all_waveforms{k}.u, 'LineWidth', 1.1, 'DisplayName', tests(k).name);
end
grid on;
ylabel('u Q14');
title('Fixed-point Q14 PI test summary');
legend('Location', 'eastoutside');

nexttile;
hold on;
for k = 1:numel(tests)
    plot(all_waveforms{k}.t, all_waveforms{k}.error, 'LineWidth', 1.1, 'DisplayName', tests(k).name);
end
grid on;
ylabel('error Q14');

nexttile;
hold on;
for k = 1:numel(tests)
    stairs(all_waveforms{k}.t, double(all_waveforms{k}.sat), 'LineWidth', 1.1, 'DisplayName', tests(k).name);
end
grid on;
ylabel('sat');
xlabel('Time (s)');

exportgraphics(fig, fullfile(fig_dir, 'summary.png'), 'Resolution', 170);
close(fig);
end

function codegen_status = run_codegen_check(model, script_dir)
codegen_status = struct('pass', false, 'message', '', 'function_found', false, 'source_file', '');
try
    slbuild(model);
    source_file = fullfile(script_dir, [model '_ert_rtw'], [model '.c']);
    codegen_status.source_file = source_file;
    if exist(source_file, 'file')
        code_text = fileread(source_file);
        codegen_status.function_found = contains(code_text, 'fixed_point_pi_FixedPointPI_Q14');
    end
    codegen_status.pass = codegen_status.function_found;
    if codegen_status.pass
        codegen_status.message = '代码生成成功，已找到 FixedPointPI_Q14 的可复用 C 函数。';
    else
        codegen_status.message = '代码生成完成，但未找到预期的可复用 C 函数名。';
    end
catch ME
    codegen_status.pass = false;
    codegen_status.message = ME.message;
end
end

function write_report(tests, results, codegen_status, results_dir, fig_dir, q14_lsb, iq24_lsb, u_max_q14, u_min_q14)
report_file = fullfile(results_dir, 'fixed_point_pi_q14_test_report.md');
[~, fig_folder] = fileparts(fig_dir);
lines = strings(0, 1);
lines(end+1) = '# Fixed-point Q14 PI 测试报告';
lines(end+1) = '';
lines(end+1) = sprintf('生成时间：%s', string(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')));
lines(end+1) = '';
lines(end+1) = '## 被测对象';
lines(end+1) = '';
lines(end+1) = '- 模型：fixed_point_pi_q14';
lines(end+1) = '- 可复用模块：FixedPointPI_Q14';
lines(end+1) = '- 输入/输出接口：Q14，`sfix16_En14`';
lines(end+1) = '- 积分内部状态：IQ24，`sfix32_En24`';
lines(end+1) = '- 增益参数类型：`sfix32_En20`';
lines(end+1) = sprintf('- Q14 LSB：%.12g', q14_lsb);
lines(end+1) = sprintf('- IQ24 LSB：%.12g', iq24_lsb);
lines(end+1) = sprintf('- 输出限幅量化值：上限 %.12g，下限 %.12g', u_max_q14, u_min_q14);
lines(end+1) = '';
lines(end+1) = '## 总览图';
lines(end+1) = '';
lines(end+1) = sprintf('![summary](%s/summary.png)', fig_folder);
lines(end+1) = '';
lines(end+1) = '## 测试汇总';
lines(end+1) = '';
lines(end+1) = '| 测试 | 结果 | final u | final error | final integral | peak abs u | sat ratio | 误差量化偏差 |';
lines(end+1) = '|---|---:|---:|---:|---:|---:|---:|---:|';
for k = 1:numel(tests)
    lines(end+1) = sprintf('| %s | %s | %.8f | %.8f | %.8f | %.8f | %.2f %% | %.3g |', ...
        tests(k).name, pass_text(results(k).pass), results(k).final_u, results(k).final_error, ...
        results(k).final_integral, results(k).peak_abs_u, 100 * results(k).sat_ratio, ...
        results(k).error_quantization_abs_error);
end
lines(end+1) = '';
lines(end+1) = '## 详细结果';
lines(end+1) = '';
for k = 1:numel(tests)
    lines(end+1) = sprintf('### %s', tests(k).name);
    lines(end+1) = '';
    lines(end+1) = sprintf('- 说明：%s', tests(k).description);
    lines(end+1) = sprintf('- 输入：ref = %.12g，feedback = %.12g', tests(k).ref, tests(k).feedback);
    lines(end+1) = sprintf('- 判据：%s', results(k).criterion);
    lines(end+1) = sprintf('- 结果：%s', pass_text(results(k).pass));
    lines(end+1) = sprintf('- Q14 输入量化：ref_q14 = %.12g，feedback_q14 = %.12g，expected_error_q14 = %.12g', ...
        results(k).input_ref_q14, results(k).input_feedback_q14, results(k).expected_error_q14);
    lines(end+1) = sprintf('- 误差量化偏差：%.12g', results(k).error_quantization_abs_error);
    lines(end+1) = '';
    lines(end+1) = sprintf('![%s](%s/%s.png)', tests(k).id, fig_folder, tests(k).id);
    lines(end+1) = '';
end
lines(end+1) = '## 代码生成检查';
lines(end+1) = '';
lines(end+1) = sprintf('- 结果：%s', pass_text(codegen_status.pass));
lines(end+1) = sprintf('- 说明：%s', codegen_status.message);
if strlength(string(codegen_status.source_file)) > 0
    lines(end+1) = sprintf('- 生成源文件：%s', codegen_status.source_file);
end
lines(end+1) = '';
lines(end+1) = '## 结论';
lines(end+1) = '';
if all([results.pass]) && codegen_status.pass
    lines(end+1) = '所有定点功能测试通过，可复用 PI 子系统已成功生成可复用 C 函数。';
else
    lines(end+1) = '存在未通过项，请根据上面的详细结果检查模型或测试阈值。';
end

writelines(lines, report_file);
end

function y = quant_floor_sat(x, word_length, fraction_length)
lo = -2^(word_length - fraction_length - 1);
hi = (2^(word_length - 1) - 1) / 2^fraction_length;
y = floor(x * 2^fraction_length) / 2^fraction_length;
y = min(max(y, lo), hi);
end

function text = pass_text(pass)
if pass
    text = 'PASS';
else
    text = 'FAIL';
end
end
