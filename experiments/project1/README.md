# Project1正式保留实验

主目录只保留四个阶段，按以下顺序阅读：

| 阶段           | 数据与报告                                            | MATLAB入口                                            |
| -------------- | ----------------------------------------------------- | ----------------------------------------------------- |
| 静态FK         | `2026-10-03_Multibody_static _fk`                     | 对应日期的`matlab_plot.m`                             |
| 动态FK         | `2026-10-05_dynamic_fk`                               | 对应日期的`matlab_plot.m`                             |
| 离线IK精简版   | `2026-10-18_ik_trajectory`                            | 对应日期的`matlab_plot.m`                             |
| 真机执行与收尾 | `2026-10-27_movej_repeat/project1_closeout/REPORT.md` | 对应日期的`analyze_today.m`、`view_project1_figure.m` |

全部MATLAB入口位于`MATLAB/experiments/project1/`。离线IK保留标准CSV、两份统计和6张核心图；相邻解步长仍检查，但不再单独绘图。

10月2日、10月24日、10月25日实验及对应旧入口已移出主目录。当前工作树不含历史归档目录，不承诺可恢复；需要历史内容时使用独立备份。保留的四阶段与最终34段数据不受影响。

实验模板移到`templates/experiment`。通用数学、FK/IK、轨迹算法与验证测试保留。

## 当前只读回归

```bash
ctest --test-dir build/debug --output-on-failure
/usr/bin/python3 -m unittest discover -s tests/integration -p 'test_project1_closeout_data.py' -v
/usr/bin/python3 -m unittest discover -s tests/integration -p 'test_movej_single_joint_probe.py' -v
```

MATLAB加载`setup_piper_paths`与`MATLAB/tests`后运行：`test_ik_trajectory_analysis`、`test_project1_closeout_metrics`、`test_project1_closeout_data`。均不发送真机指令。
