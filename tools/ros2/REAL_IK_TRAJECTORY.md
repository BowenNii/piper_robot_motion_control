# 真机基础设施的保留范围

`real_ik_trajectory.py`暂保留RobotIO、只读驱动配置、进程锁、门控清理、反馈新鲜度与图发现等实现，供ABC工具复用。共用参数读取函数已从旧五方案工具移入本文件。

原10月24日60秒/2mm实验及专用入口已移出；当前工作树不包含历史归档，恢复需要独立备份。当前C++生成器不提供那些旧CLI模式，不以保留Python函数名作为可执行功能保证。

当前实验与分析入口见[工具索引](README.md)、[ABC说明](MOVEJ_SINGLE_JOINT_PROBE.md)和`experiments/project1/2026-10-27_movej_repeat/project1_closeout/REPORT.md`。

本次清理未修改运动限值、使能/停止方式或控制接口。仅调整共用函数位置和分析/测试范围。未进行任何真机动作。
