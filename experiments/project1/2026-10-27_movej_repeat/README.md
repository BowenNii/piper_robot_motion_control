# Project1真机执行验证与收尾分析

当前正式入口：[project1_closeout/REPORT.md](project1_closeout/REPORT.md)。图按01～10阅读。

## 目录

- `project1_closeout/`：按周期重新解码的34段数据、质量修正对照、收尾报告、统计表、10张PNG和10张可交互FIG。
- `j1_0p5/`、`j4_0p5/`、`j5_0p5/`、`j6_0p5/`、`multi_abc/`：原始34段主实验，保留全部重复轮，不移动、不覆盖。
- 最初`run01`、旧分析与旧`decoded`已移出；当前工作树不包含历史归档，不承诺可恢复。正式解码只使用`project1_closeout/decoded`。

## MATLAB查看

进入`MATLAB/experiments/project1/2026-10-27_movej_repeat`后：

```matlab
view_project1_figure(1)   % 数据质量
view_project1_figure(2)   % C六轴去程路径
view_project1_figure(4)   % 六轴关节角对比
view_project1_figure(9)   % 位置、姿态和空间偏差
```

图窗数据提示可读取具体点，切换放大镜后局部放大。FIG保存原始曲线数据，并非PNG像素放大。

## 离线复现

1. `cmake --build build/debug --target project1_feedback_export`。
2. `/usr/bin/python3 tools/ros2/prepare_project1_closeout.py`：只读原始日志、验证SHA256、按2A2起帧对齐；已有`project1_closeout`拒绝覆盖。
3. MATLAB运行`analyze_today`。已有`project1_closeout/matlab`拒绝覆盖；重跑前归档生成目录，不删除原始记录。

不移动现有结果的离线重新导出示例（新目录必须不存在）：

```bash
/usr/bin/python3 tools/ros2/prepare_project1_closeout.py --output-dir /tmp/piper_closeout_new
```

默认从正式结果的`decode_comparison.csv`读取历史对照列，也可用`--baseline-comparison`指定保存的表；缺少旧统计时写NaN，不伪造对照。不自动覆盖正式MATLAB结果。

`analyze_today`默认读取`config/project1_analysis.json`；null保持“未设任务容差”的语义，显式cfg覆盖默认值。新增配置不改写已保存结果。

本次输出使用空验收容差，只报告性能，没有宣布所有任务达标。未来实验前可通过`analyze_today(cfg)`明确设置`joint_tolerance_deg`、`position_tolerance_mm`、`orientation_tolerance_deg`，以计算超限时间占比和最长连续超限。不要为让既有数据通过而事后选择容差。

## 本次软件核查

解码器、有序拼帧、CAN解析与MATLAB指标已核查。旧60秒终点IK/2mm测试不再对应当前CLI，已移出保留测试范围；当前CLI回归检查标准CSV、时长、拒绝覆盖以及明确拒绝旧参数，不代表恢复旧模式。
当前测试入口、数据核查范围与构建选项以仓库README和CMakePresets为准。已有34段数据与FIG保持不变。
