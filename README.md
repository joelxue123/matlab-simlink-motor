# average-inverter 电机控制起步说明

这个目录现在是一个最小控制骨架，适合用 average-inverter 模型先把表贴式永磁同步电机控制器跑通，再决定是否切到开关级逆变器。

## 适用场景

- 先验证控制算法，不关心 PWM 开关纹波
- 做 Surface Mount PMSM 的 FOC 闭环
- 后续准备迁移到 C2000 或其他实时控制器

## 推荐模型结构

建议你在 Simulink 中按下面的链路搭建：

1. 速度给定
2. 速度 PI，输出 `iq_ref`
3. `id_ref = 0`
4. `abc -> dq` 电流变换
5. `d/q` 电流 PI
6. 解耦前馈，得到 `vd_ref` 和 `vq_ref`
7. `dq -> abc` 电压指令
8. average-inverter
9. PMSM 电机模型
10. 采样电流、转速、电角度，回馈到控制器

## average-inverter 的作用

average-inverter 不是逐开关器件的模型，而是平均化电压源模型。优点是：

- 仿真快
- 便于先调 PI 和观测控制结构是否正确
- 在控制设计阶段更稳定

它的限制也很明确：

- 看不到真实 PWM 纹波
- 不适合分析死区、电流采样噪声、开关损耗
- 做最终硬件一致性验证之前，最好再切一次开关级模型

### 你截图里的 Average-Value Inverter 是什么逻辑

你发的那个子系统，本质上是在做两件事：

1. 去掉三相指令里的公共模分量
2. 按直流母线电压生成最终三相输出电压

常见表达式可以写成：

```matlab
Vabc = (Vabc_cmd - mean(Vabc_cmd))
```

如果输入是调制量而不是电压指令，则会进一步乘上 `Vdc` 或对应比例因子。

当前这个工程里，我已经把 [average_inverter_foc.slx](average_inverter_foc.slx) 的 `Average Inverter` 子系统改成了和截图更一致的接口：

- 输入 `Vabc_mod`
- 输入 `Vdc`
- 三相调制量求平均值
- 每相减去公共模
- 调制量限幅到 `[-1, 1]`
- 最后乘以 `Vdc/2` 得到三相输出电压

也就是说，当前工程里的 average inverter 语义已经对齐为：

```matlab
Vabc = (Vabc_mod - mean(Vabc_mod)) * Vdc / 2
```

这样它和你截图里的平均值逆变器接口就是同一类结构了。

## 已提供文件

- `motor_control_params.m`：生成一组可直接用于 Surface Mount PMSM FOC 的基线参数
- `build_average_inverter_foc_model.m`：自动生成 Simulink 控制框架模型
- `average_inverter_foc.slx`：已生成的 Simulink 起步模型
- `build_model_and_open.m`：生成并尝试打开模型

## Simulink 接口现状

现在这个工程已经不是只有 MATLAB 脚本了，还包含一个可加载的 Simulink 模型：

- `average_inverter_foc.slx`

这个模型当前是控制框架接口，顶层已经包含：

- 速度给定
- 速度 PI 子系统
- 电流参考子系统
- `abc -> dq` 子系统
- 电流 PI 子系统
- `dq -> abc` 子系统
- Average Inverter 子系统
- PMSM Plant 子系统

子系统内部目前是接口占位，方便你先把控制结构和信号链连清楚，再逐步替换成：

- 你自己的方程模型
- Simulink 自带模块
- Simscape Electrical 或 Motor Control Blockset 里的模块

目前其中两个关键部分已经改为直接调用现有库模型，而不是自建占位：

- `Average Inverter` -> `mcbplantlib/Average-Value Inverter`
- `Surface Mount PMSM` -> `Surface Mount PMSM` 现有库块

所以当前工程已经是在复用 MATLAB 现成模型块，而不是完全手工搭建电机和逆变器。

## 参数脚本怎么用

在 MATLAB 命令行先运行：

```matlab
motor_control_params
```

运行后会得到 4 个结构体：

- `motor`
- `inverter`
- `control`
- `simcfg`

这些结构体可以直接绑定到 Simulink 模型参数中。

## 关键控制建议

### 1. 电流环先于速度环调试

先断开速度环，手动给 `iq_ref`，观察：

- `iq` 是否快速跟踪
- `id` 是否稳定在 0
- `vd/vq` 是否没有明显饱和

### 2. 带宽分层

建议：

- 电流环带宽约为速度环的 8 到 15 倍
- 控制采样周期 `Ts_ctrl` 明显小于电流环时间常数

当前脚本给的是：

- 电流环 `800 Hz`
- 速度环 `80 Hz`

这是一个比较稳妥的起点，不一定是最终值。

### 3. 电压限幅必须做

average-inverter 也要保留调制限幅，否则 PI 会在电压不足时积分发散。当前脚本里把输出限幅放在：

- `control.pi_id.output_limit`
- `control.pi_iq.output_limit`

## 当前电机假设

当前参数脚本按 Surface Mount PMSM 建模，主要特征是：

- `Ld = Lq`
- 无显著凸极效应
- `id_ref` 默认取 `0`
- 转矩主要由 `iq` 和永磁磁链决定

如果你的电机铭牌或辨识结果不是这个特征，例如 `Ld` 和 `Lq` 差很多，那就更像 Interior PMSM，不应该继续沿用当前这套基线。

## 如果你要做异步电机而不是 PMSM

这个骨架仍然能复用，但需要替换：

- 电机参数模型
- 转矩公式
- 解耦项
- 观测器或磁链估算部分

也就是说，控制框架不变，但参数与控制律不能直接照搬。

## 建议的下一步

1. 先搭一个 `average-inverter + PMSM + FOC` 的最小 Simulink 模型
2. 用 `motor_control_params.m` 接通参数
3. 先验证 `iq` 阶跃响应
4. 再闭合速度环
5. 最后再考虑离散化细节、饱和保护和硬件映射

## Linux 下 C2000 ControlSUITE 报错处理

如果你在 Linux 的 MATLAB R2023b 上打开 C2000 示例后，遇到类似下面的错误：

```text
The model is using 'ControlSUITE' third-party tool, but your product is not set up to use that third-party tool.
```

先区分两种情况：

- 你确实还没有完成 `c2000setup`
- 你已经完成过配置，但当前工作目录被示例切到了 `/tmp/Examples/...`，导致第三方工具注册文件按相对路径查找失败

这个仓库里提供了一个辅助脚本：

- `fix_controlsuite_registry_linux.m`

用途是把 MATLAB 安装目录下的 C2000 third-party registry 文件，镜像到当前工作目录下的相对路径位置，用来绕过 Linux R2023b 下示例目录触发的相对路径解析问题。

在 MATLAB 里可以这样用：

```matlab
cd('/tmp/Examples/R2023b/mcb/FOCQepExample')
fix_controlsuite_registry_linux
```

如果你是在 Simulink GUI 里点 Build 编译，更稳妥的用法是先运行：

```matlab
prepare_c2000_gui_build_linux('/tmp/Examples/R2023b/mcb/FOCQepExample')
```

这个脚本会做两件事：

- 在 GUI 编译要用到的示例目录下补齐相对 registry 路径
- 把 MATLAB 当前工作目录切到该示例目录，避免 GUI 编译时仍然从错误目录解析 `R2023b/...` 相对路径

如果你想先看清楚到底是哪一步缺失，可以先运行：

```matlab
diagnose_c2000_gui_build_linux('/tmp/Examples/R2023b/mcb/FOCQepExample')
```

这个诊断脚本会检查：

- 示例目录是否存在
- `matlabroot` 下的 C2000 registry 源文件是否存在
- 示例目录下相对路径 registry 是否已经补齐
- MATLAB 当前工作目录是否正好就是 GUI 编译目录

如果这 4 项都通过，但 GUI 编译还是继续报 `ControlSUITE` 未配置，那么问题基本就收敛到 `c2000setup` 中配置的第三方工具路径本身，例如：

- CCS 路径无效
- ControlSUITE 路径无效
- 当前芯片依赖的第三方工具并没有真正安装完成

如果脚本报找不到源 registry 文件，说明你的 C2000 支持包本身还没有完成配置，这时还是要先运行：

```matlab
c2000setup
```

另外需要注意：部分处理器依赖的 `ControlSUITE` 安装资源只提供 Windows 安装器。Linux 场景通常需要先在 Windows 上安装，再把对应文件夹复制到 Linux，然后在 `c2000setup` 中指向该目录。

如果你愿意，我下一步可以直接继续给你补：

1. 一个 MATLAB 版的 PI 控制器离散化参数计算脚本
2. 一个更完整的 Simulink 模块连接清单
3. 如果你的对象其实是异步电机，我可以改成 IM 控制版本