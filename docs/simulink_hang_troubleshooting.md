# Simulink Hang Troubleshooting

本文记录 2026-06-23 排查 `green_joint_current_loop_mbd` 时遇到的
Simulink “模型更新”卡死现象。后续 AI 接手 MBD/Simulink 任务时，先按本文件
检查，不要直接反复打开模型或强杀 MATLAB。

## 现象

用户在 MATLAB Desktop / Simulink 中打开或更新模型时，界面长时间卡在
“模型更新”附近。

观察到的进程特征：

```text
MATLAB -desktop 持续高 CPU
MATLABWebUI / MATLABWindow renderer 进程持续运行
MCR 0 interpret / ProductsInit / QT GuiThread 等线程活跃
```

同时，`matlab -batch` 在无桌面会话干扰时可以通过 smoke test 和 codegen：

```bash
matlab -batch "run('matlab-practice/green_joint_current_loop_mbd/run_green_joint_current_loop_smoke_test.m')"
matlab -batch "run('matlab-practice/green_joint_current_loop_mbd/generate_green_joint_current_loop_code.m')"
```

这说明当前问题不是 PI 算法本身形成代数环或求解器死循环，而更像是
Simulink Desktop/UI、缓存、文件占用和脚本重建流程之间的冲突。

## 本次根因判断

高风险组合：

```text
MATLAB Desktop 正打开生成模型或数据字典
  + build 脚本删除/重建同名 .slx/.sldd
  + 脚本自动 arrangeSystem / SimulationCommand update
  -> Simulink Desktop/CEF/模型缓存可能卡住
```

本次重点文件：

```text
matlab-practice/green_joint_current_loop_mbd/green_joint_current_loop_model.slx
matlab-practice/green_joint_current_loop_mbd/green_joint_current_loop_interface.sldd
matlab-practice/green_joint_current_loop_mbd/green_joint_current_loop_model.slxc
```

原先的危险代码模式：

```matlab
if exist(model_file, 'file')
    delete(model_file);
end

...

Simulink.BlockDiagram.arrangeSystem(subsystem);
Simulink.BlockDiagram.arrangeSystem(model);
set_param(model, 'SimulationCommand', 'update');
```

这些代码在 batch 中通常能跑通，但在 Desktop 已经打开同名模型时，风险明显增加。

## 已做修复

`green_joint_current_loop_mbd` 已做以下修复：

- `build_green_joint_current_loop_model.m` 不再在 build 阶段自动
  `SimulationCommand='update'`。
- `build_green_joint_current_loop_model.m` 不再自动调用
  `Simulink.BlockDiagram.arrangeSystem(...)`。
- 新增 `assert_green_joint_safe_rebuild_environment.m`。
- `build_green_joint_current_loop_model.m` 和
  `generate_green_joint_current_loop_dictionary.m` 在删除/重建 `.slx/.sldd`
  前调用安全检查。
- `generate_green_joint_current_loop_code.m` 保留 codegen 守卫：
  `verify_green_joint_current_pi_codegen()`。

安全检查目的：

```text
如果其它 MATLAB 进程持有生成模型/字典文件，先报错并停止重建，
不要让 Simulink 卡在模型更新或文件缓存等待里。
```

## 后续 AI 操作规则

1. 不要在 MATLAB Desktop 打开模型时运行会重建 `.slx/.sldd` 的脚本。
2. 需要重建模型时，先关闭 Desktop 中的模型和数据字典。
3. 优先使用 `matlab -batch` 运行 build/smoke/codegen。
4. 构建脚本默认不要自动 `open_system(...)`。
5. 构建脚本默认不要自动 `arrangeSystem(...)`，除非明确是手动整理布局任务。
6. build 脚本可以保存模型，但 update 应由 smoke test 或 codegen 显式触发。
7. 不要在同一时间启动多个 MATLAB 进程重建同一个模块。
8. 如果必须从 Desktop 调试，先确认没有 batch 脚本正在跑。

推荐命令：

```bash
matlab -batch "run('matlab-practice/green_joint_current_loop_mbd/run_green_joint_current_loop_smoke_test.m')"
matlab -batch "run('matlab-practice/green_joint_current_loop_mbd/generate_green_joint_current_loop_code.m')"
```

## 排查命令

查看 MATLAB/Simulink 相关进程：

```bash
ps -eo pid,ppid,stat,etime,%cpu,%mem,rss,wchan:30,cmd | rg -i "MATLAB|Simulink|MATLABWebUI|MATLABWindow"
```

查看线程热点：

```bash
ps -T -p <MATLAB_PID> -o pid,tid,stat,psr,pcpu,pmem,wchan:30,comm | sort -k5 -nr | head -60
```

查看模型/字典是否被其它进程持有：

```bash
lsof matlab-practice/green_joint_current_loop_mbd/green_joint_current_loop_model.slx \
     matlab-practice/green_joint_current_loop_mbd/green_joint_current_loop_interface.sldd \
     matlab-practice/green_joint_current_loop_mbd/green_joint_current_loop_model.slxc
```

查看 MATLAB crash/log：

```bash
ls -lt ~/matlab_crash_dump.* ~/java.log.* 2>/dev/null | head
```

## 如何取消模型更新

优先级：

```text
MATLAB Command Window: Ctrl+C
Simulink 进度条: Cancel
终端温和中断: kill -INT <PID>
正常结束: kill -TERM <PID>
最后手段: kill -KILL <PID>
```

不要一开始就 `kill -KILL`，否则可能丢失未保存模型，甚至留下临时缓存。

## 判断标准

如果 batch smoke/codegen 通过，而 Desktop 卡死，优先怀疑：

- Desktop UI/CEF 渲染层卡住。
- 模型或数据字典被多个 MATLAB 会话同时访问。
- 构建脚本删除/重建了 Desktop 正打开的 `.slx/.sldd`。
- 自动 `arrangeSystem` 或打开大量 Scope/窗口触发 UI 卡顿。

如果 batch smoke/codegen 也卡住，才进一步怀疑：

- 模型结构错误。
- 数据字典损坏。
- 代数环/类型传播/Bus 配置问题。
- 第三方库加载问题。

## 当前结论

`green_joint_current_loop_mbd` 当前算法和 codegen 已通过验证。后续卡死排查应优先从
Desktop 会话、文件占用、自动 update/layout 触发点入手，而不是先怀疑 PI 算法。
