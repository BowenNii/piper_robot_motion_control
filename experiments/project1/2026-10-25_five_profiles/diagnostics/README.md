# 隔离调度对照，不是真机运动记录

运行tools/ros2/isolated_schedule_probe.py --compare会自动顺序比较三种模式：

- baseline：原回调/列表日志，循环中创建ROS目标消息。
- preallocated：只提前创建目标消息，其余保持一致。
- buffered：再将反馈、状态、调度、回调日志替换为固定容量数值数组。

所有模式都保持GC开启，不改变真机程序的GC策略，不初始化RobotIO，不创建真机服务/Action客户端，不打开CAN。固定DDS域93和/offline_diagnostics/*话题。回放使用历史数据值和新的200Hz时钟，不复制历史接收卡顿。

命令（在source ROS/Jazzy及agx_arm_ws后）：

    /usr/bin/python3 /home/nbw/piper_robot_motion_control/tools/ros2/isolated_schedule_probe.py \
      --compare \
      --plan /home/nbw/piper_robot_motion_control/experiments/project1/2026-10-25_five_profiles/runs/scheme1_return01/trajectory.csv \
      --feedback /home/nbw/piper_robot_motion_control/experiments/project1/2026-10-25_five_profiles/runs/scheme1_return01/execution__lmrsp_m/feedback.csv \
      --output-dir /home/nbw/piper_robot_motion_control/experiments/project1/2026-10-25_five_profiles/diagnostics/schedule_compare01

一次三组约3分钟，不需要动作或录像。输出目录必须不存在；复测换02/03，不能同时并行多个比较程序。电脑保持相近使用状态，记录期间不人为增加负载。

comparison.csv汇总：最大“构造完成准备发布时”的迟到、>=20ms次数、最大GC暂停、反馈接收空档、消息构造耗时、开始/结束峰值RSS。保留全部结果，不能凭单次最好值宣布修复。

每组子目录：

- schedule.csv：计划/实际时刻，构造前及构造后迟到，构造/发布耗时、CPU时间、反馈年龄。
- gc_events.csv：GC开始时间、持续时间、代次、线程ID；只记录运动学调度测试区间。
- feedback.csv、status.csv、callback_times.csv：模拟接收和回调耗时。
- memory.csv：Linux峰值RSS（KiB）随时间，不是实时RSS；峰值不下降正常，不能单靠它证明不存在内存泄漏。
- summary.json：模式、时间、输入来源、是否完成、峰值指标。
- replay_process.log：隔离反馈生成进程日志。

判断时先看迟到和接收间隔是否稳定，再看GC和内存代价。更高的起始内存可能是预分配的预期代价。先做完整对照，再换顺序重复至少一轮；不同运行的系统负载/GC状态也会影响结果。这不是USB-CAN、真实驱动或固件测试，不是机械臂安全认证。
