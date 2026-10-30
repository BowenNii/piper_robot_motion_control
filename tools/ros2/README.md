# Project1保留的ROS适配与离线分析工具

正式实验索引：[experiments/project1/README.md](../../experiments/project1/README.md)。

| 工具 | 用途 |
| --- | --- |
| `csv_trajectory_preview.py` | 离线CSV检查与RViz显示，不控制真机 |
| `csv_action_sender.py` | 默认只检查；显式执行也只允许隔离模拟控制器 |
| `movej_single_joint_probe.py` | ABC指定单轴/六轴执行验证，prepare/check/execute分开 |
| `real_ik_trajectory.py` | ABC复用的RobotIO、门控、锁、反馈与图发现等基础设施 |
| `numeric_log_buffer.py` | 固定缓冲日志，避免发送循环频繁分配 |
| `prepare_project1_closeout.py` | 只读CAN日志，按周期解码并生成收尾清单 |

## 查看与离线检查

```bash
/usr/bin/python3 tools/ros2/csv_trajectory_preview.py --check-only
/usr/bin/python3 tools/ros2/csv_action_sender.py
/usr/bin/python3 -m unittest discover -s tests/integration -p 'test_project1_closeout_data.py' -v
```

默认CSV保留在`experiments/project1/2026-10-18_ik_trajectory/data_csv/trajectory.csv`。
RViz播放需加载ROS环境，并保持真机断电、不启动真机驱动；`--state-playback`只发布独立显示消息，不作碰撞检查或硬件放行。

ABC工具说明：[MOVEJ_SINGLE_JOINT_PROBE.md](MOVEJ_SINGLE_JOINT_PROBE.md)。禁止重放历史prepare目录；程序变化后必须重新准备新目录并人工检查运动空间。门控关闭不等于撤销底层目标或硬件急停。

## 已归档

五方案发送/预览、官方bag导出、CAN重复与调度诊断已移出主目录。当前工作树没有历史归档，恢复需独立备份。旧大幅预览CSV、60秒终点IK和2mm试验不再作为当前入口。

当前C++ `ik_trajectory_validation`只支持输出路径和时长；不要调用旧文档中的`--wide-preview`、`--relative-target`或`--endpoint-joint-target`。归档脚本只供追溯，不能从归档位置直接执行。
