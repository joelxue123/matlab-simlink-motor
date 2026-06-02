# Studies

这个目录是新的实验入口层。

目标只有三个：

- 每个算法验证单独一个子目录
- 每个子目录只保留该实验自己的入口、说明和输出
- 公共逻辑继续复用根目录已有库，不再把新入口散落回根目录

## 目录约定

每个实验子目录建议固定包含下面几项：

- `README.md`：说明实验目的、入口、关键参数、输出物
- 1 个主入口函数：例如 `run_study.m`
- 可选的复现实验入口：例如 `reproduce_validation.m`
- `outputs/`：该实验生成的 `mat/csv/txt` 结果默认写这里

## 现有实验

- `cogging_position_scan/`：位置扫描建表与验证
- `cogging_torque_comp/`：齿槽转矩补偿研究
- `inertia_identification/`：基于速度环阶跃响应的等效惯量辨识基线
- `电流环验证/`：电流环 PI 参数算法验证
- `vibration_comp/`：基础振动补偿展示

## 使用方式

建议以后都从各自实验子目录进入，而不是直接在根目录跑入口：

```matlab
cd studies/cogging_position_scan
result = run_study;
```

这样生成的结果会集中写到该实验自己的 `outputs/` 下，不会继续污染根目录。

## 以后新增实验怎么放

直接复制 `_template/` 新建一个子目录，再把该算法验证的入口接进去。

不要再把新的 `run_xxx.m`、`compare_xxx.m`、`export_xxx.m` 直接堆到项目根目录。