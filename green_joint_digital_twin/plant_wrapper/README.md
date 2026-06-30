# Green Joint Plant Wrapper

本目录用于定义 `green-joint` 数字孪生里的 plant 组合边界。

PlantWrapper 负责统一电机、逆变器、传感器、延时和日志回放接口。
ControllerWrapper 不直接依赖某个具体 plant。

## 目标 variants

### average_v1

主判据模型：

```text
DqToAbcDutyStep duty [0, 1]
  -> Average-Value Inverter
  -> Surface Mount PMSM
  -> MotorClarkeParkStep
```

用途：

```text
电流环调参
速度环调参
大多数闭环动态验证
硬件波形主对齐
```

### switching_study

专项模型：

```text
PWM / deadtime / sampling window / switching ripple
```

用途：

```text
验证死区补偿、ADC 采样窗口、PWM 更新延迟。
不作为日常 PI 参数整定主模型。
```

### log_replay

真实日志回放：

```text
hardware log
  -> measured id/iq/theta/wm/vbus/load estimate
  -> controller comparison
```

用途：

```text
对齐硬件数据。
反推参数。
验证生成代码和固件输出一致性。
```

## PlantWrapper 输出合同

```text
id_fbk, A
iq_fbk, A
ia, A
ib, A
ic, A
theta_e, rad
wm_meas, rad/s
vbus_meas, V
load_torque, N*m
```

## 规则

```text
plant 默认不 codegen。
平均电压模型是 green-joint 电流环和速度环主判断模型。
开关级模型只做专项验证。
日志回放不能替代物理模型，但必须用于最终硬件对齐。
```
