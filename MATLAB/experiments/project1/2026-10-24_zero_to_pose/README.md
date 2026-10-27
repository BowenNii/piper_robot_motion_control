# 零位到目标位姿的终点IK实验

目标T由参考角[0.3,0.3,-0.3,0.3,0.3,0.5]rad的FK定义。
C++从新鲜实际近零起点求DLS终点IK，再从实际起点到该解生成60秒、3001点
关节空间五次轨迹。末端不保证直线；只对终点求IK，不宣称逐点笛卡尔控制。
CSV中path_mode=1区分该模式；中间FK自洽误差不是独立的IK精度证明，
终点另对原始T_goal做回代检查。

已回零后保持现有真机驱动（5%、fast_mode=false），关闭MoveIt及其他发送器。
新终端source ROS和工作空间，执行：

```bash
/usr/bin/python3 /home/nbw/piper_robot_motion_control/tools/ros2/real_ik_trajectory.py prepare-endpoint \
 /home/nbw/piper_robot_motion_control/experiments/project1/2026-10-24_zero_to_pose/run01
```

不发运动目标；要求反馈各轴在零附近0.002rad内，且未超原始URDF限位。
不裁剪/伪造起点，不自动回零；不符合就拒绝。已有目录换run02。
生成后可在隔离RViz显示中用csv_trajectory_preview.py的--state-playback检查trajectory.csv；
也可以用analyze_ik_trajectory复核准备好的CSV，不需要旧独立预览脚本。
这不是碰撞检测；必须现场确认全程机械臂、夹爪、线缆与桌面/周边无干涉。

准备成功、起点未变且现场确认安全后才执行：

```bash
/usr/bin/python3 /home/nbw/piper_robot_motion_control/tools/ros2/real_ik_trajectory.py execute \
 /home/nbw/piper_robot_motion_control/experiments/project1/2026-10-24_zero_to_pose/run01 \
 --confirm-real-hardware
```

只发六轴MOVE J位置，不动夹爪；保留0.25秒反馈过期、0.01rad跟随偏差、
20ms调度迟到、0.005rad终点偏差保护，原小幅2mm模式的限制不变。
按50Hz发60秒，再观察2秒并关门控。MOVE J再次平滑，不保证实机严格跟踪该时序。
Ctrl+C请求软件保持再关门，不等于硬件急停；关门控不撤销底层已接受的目标。
记录位于run01/execution_*，原数据不覆盖。

## 本轮真机绘图（已指定真实执行目录）

在MATLAB命令窗口运行：

```matlab
run('/home/nbw/piper_robot_motion_control/MATLAB/experiments/project1/2026-10-24_zero_to_pose/matlab_plot.m')
```

默认读取run01/execution_mwd2hy56，生成六轴跟随、关节偏差、末端XYZ与姿态变化、
末端位姿偏差、记录/发送时间质量共5张图。图窗名称含“60秒真机IK”，不会与
2mm实验混淆。旧matlab_preview.m已清理归档，日常直接使用本入口。

脚本使用summary保存的单调时钟原点对齐，不采用2mm旧记录的尾段估计，也不
将不同采样频率的CSV逐行相减。对齐后比较仍包含反馈传输/排队影响，不据此
测单向通信延迟。位置和姿态在本轮都会改变，姿态变化图不能误写成“姿态保持”。

结果写入该execution下matlab_analysis，包括figures中的5张PNG、统计CSV和MAT。
只覆盖自身派生结果，不改动commands/feedback/status.csv、summary或trajectory.csv。
末端所有曲线均为同一标称URDF模型的FK估计，不是外部实测精度。

## 抖动专项排查

matlab_diagnose_jitter.m读取本轮analysis_results.mat，不连接ROS，生成启动5～10秒
与36.175秒时间事件的两张放大图和统计表，保存在同一execution的
matlab_analysis/jitter_diagnostics。核对结果、驱动/SDK审查及run02复测步骤见jitter_report.md。
本次未修改真机发送参数或控制方案；复测目的为重复性及反馈来源验证，不是已修复抖动。

## 清理后保留内容

- 本目录：matlab_plot.m（正式真机分析入口）、README.md。
- 真机run01：trajectory.csv、manifest.json、generator.log。
- execution_mwd2hy56：commands.csv、feedback.csv、status.csv、summary.json。
- execution_mwd2hy56/matlab_analysis：5张图、统计CSV、analysis_results.mat。
- 公共C++轨迹程序、Python真机工具、MATLAB分析库及回归测试均保留。
- 2mm真机实验、静态rosbag和离线Action原始记录归档至仓库archives/2026-10-24_preliminary。
- 对应前置MATLAB实验目录一同归档；主目录只保留本轮zero_to_pose。

恢复包在仓库archives/2026-10-24_auxiliary/auxiliary.tar.gz；详见该目录README。
