# 五组项目1完整实验：操作与代码阅读

只新增五组实验，不删除10-18/10-24。所有方案都是终点IK后的关节空间路径，不保证末端直线；MOVE J可能重新规划。规划合规不等于碰撞/力矩安全认证。代码不会使能、自动回零或循环往返。8°/s组不是因为旧回零速度较高就获准执行。

## 阅读顺序

1. core/include/piper_control/trajectory/comparison_profile.hpp：三种时间规律解析位置、速度、加速度。
2. apps/project1/joint_profile_comparison.cpp：指定实测起点、固定目标解支、生成五组轨迹与FK；默认不连接硬件。
3. apps/project1/project1_feedback_export.cpp：CAN日志解码、六帧组装、FK、模型/CAN误差、原始电机反馈。
4. MATLAB/experiments/project1/2026-10-25_five_profiles/matlab_plot.m：参考轨迹图。
5. MATLAB/algorithm_library/experiment_analysis/analyze_project1_complete.m：12类图和统计的计算逻辑；绘图样式不必逐行读。
6. tools/ros2/five_profiles_experiment.py：只需理解prepare/check/execute/analyze及日志结构，不必深挖DDS。

原ik_trajectory_validation.cpp保持不动。通用真机保护/反馈订阅复用real_ik_trajectory.py中的RobotIO，故现在不能删除这个工具；旧实验数据目录并非运行依赖。

## 离线准备（可以现在做）

    cd /home/nbw/piper_robot_motion_control
    cmake --build build/debug --target joint_profile_comparison project1_feedback_export -j 2
    /usr/bin/python3 tests/integration/test_five_profiles_complete.py

完整参考已放在experiments/project1/2026-10-25_five_profiles/reference_complete，共10份CSV，含解析导数及C++ FK位姿。它们不是自动允许执行的真实准备记录。MATLAB运行本实验matlab_plot.m看参考图。

## 晚上：先只执行方案2去程

前提：驱动反馈恢复正常、状态无报警、近零实测起点稳定；已人工检查路径/线缆/负载，硬件停止方式可用。保持原低速配置piper/can0、fast_mode=false、speed_percent=5、auto_enable=false。需要人工确认机械臂已经使能；本工具不自动使能。停止其他MoveIt/Action/运动发送器，隔离域84的RViz纯显示不控制真机。不要改变接口来尝试消除抖动。

    source /opt/ros/jazzy/setup.bash
    source /home/nbw/agx_arm_ws/install/setup.bash

先准备（会关门控、读取实测起点，不发运动目标）：

    /usr/bin/python3 /home/nbw/piper_robot_motion_control/tools/ros2/five_profiles_experiment.py prepare \
      /home/nbw/piper_robot_motion_control/experiments/project1/2026-10-25_five_profiles/runs/scheme2_out01 \
      --scheme 2 --direction outbound

必须检查输出和generator.log。严格URDF边界检查失败时，不会裁剪角度或放宽限位，先排查。准备后若起点变化超过0.001rad会拒绝发送，需要新目录重新prepare。

可选再离线复核：

    /usr/bin/python3 /home/nbw/piper_robot_motion_control/tools/ros2/five_profiles_experiment.py check \
      /home/nbw/piper_robot_motion_control/experiments/project1/2026-10-25_five_profiles/runs/scheme2_out01

开始视频录制，人工确认环境安全后，单程执行：

    /usr/bin/python3 /home/nbw/piper_robot_motion_control/tools/ros2/five_profiles_experiment.py execute \
      /home/nbw/piper_robot_motion_control/experiments/project1/2026-10-25_five_profiles/runs/scheme2_out01 \
      --confirm-real-hardware --path-clear-and-stop-ready

自动录原始CAN并记录运动前后各3秒，ROS数据/发送日志暂存有界内存，运动后写出。candump可能因SocketCAN回环设置无法看到本机命令帧，这会标记为不可用，不伪称已验证CAN线上发送周期。Ctrl+C请求软件保持和关门，不能替代硬件急停；通信失效时软件停止可能无法完成。

正常结束仍应看summary.success和cleanup，并在现场确认已停止。出现明显抖动、持续超差、反馈中断、时间戳停更、报警就停止后续组别，不继续升速。日志中断/迟到20ms会中止，不集中补发。

## 返回零位：单独准备，不自动倒放

到位稳定后，使用新目录和实测起点：

    /usr/bin/python3 /home/nbw/piper_robot_motion_control/tools/ros2/five_profiles_experiment.py prepare \
      /home/nbw/piper_robot_motion_control/experiments/project1/2026-10-25_five_profiles/runs/scheme2_return01 \
      --scheme 2 --direction return

再次人工确认后，用同样execute命令指向scheme2_return01。不是调用move_home，也不自动串联下一方案。其余方案替换scheme编号和新目录名；建议2→4，其他是否执行看现场结果，3最后考虑。更改编号不能绕过安全检查。

## 运动后离线分析

记下终端输出的完整execution_*目录，将下列路径替换为实际路径，不能原样复制占位符：

    /usr/bin/python3 /home/nbw/piper_robot_motion_control/tools/ros2/five_profiles_experiment.py analyze /实际路径/execution_实际编号

只解析日志，不发命令。decoded目录必须不存在，防止覆盖。

MATLAB中：

    addpath('/home/nbw/piper_robot_motion_control/MATLAB'); setup_piper_paths;
    analyze_project1_complete('/实际路径/execution_实际编号');

或者运行本实验matlab_analyze.m，默认选择runs下最新记录。重复分析前保留或改名analysis_complete，不覆盖原始数据。

## 图与证据边界

12张图：关节位置、角度跟随误差、规划/差分速度、规划/差分加速度、高速电机速度、电流/估算力矩、XYZ轨迹、位姿跟随误差、MATLAB/C++交叉验算、MATLAB/CAN偏差、六帧/高速反馈间隔、发送时序/最小奇异值。完整R在MAT中保留，姿态用SO(3)角而非直接减RPY。

2ms是接收跨度报告线，不代表硬件同步采样精度。离线组装窗口50ms用于保留超2ms样本；超过50ms/重复ID导致的丢弃、未完成帧均在decode_summary.json，不隐瞒过滤情况。MATLAB/CAN偏差不是外部定位精度。

高速反馈单位按本地SDK缩放，速度/位置轴侧和减速比未确认，单独标为motor_reported，不直接与关节规划速度相减。估算力矩J1..3=I*1.18125/4，J4..6=I*.95844，按厂家答复，不是独立力矩测量。差分加速度尖峰不能认定真实机械加速度。

方案4保留正确的加速度匹配后最高约4.672°/s，无匀速段；不是严格的同速度+同加速度三方对照。去回程使用实测起点，峰值/时长可能略有差异，均记录在manifest。

旧实验目录现在保留，等新记录分析验证完整且备份后再按用户要求清理。

## 调度缓存改进版（2026-10-25）

`five_profiles_experiment.py` 在打开门控前预创建全部JointState消息，
反馈、状态和发送日志共用 `numeric_log_buffer.py` 的固定容量数值缓存
（每种日志最多20000行），结束后流式写出CSV。共享RobotIO旧调用默认行为不变。
GC始终开启，20ms迟到、反馈失效、跟随误差及停止/关门保护没有放宽。
summary记录 `execution_implementation=preallocated_numeric_buffers_v1`。
这减少Python长寿命行对象，不是硬实时保证，也不等于已解决机械抖动。

隔离验证见 diagnostics/schedule_compare02；不操作CAN或真机。
恢复实验不要重发中断的scheme1_return01；现场确认回零且稳定后重新准备新目录：

    /usr/bin/python3 /home/nbw/piper_robot_motion_control/tools/ros2/five_profiles_experiment.py prepare \
      /home/nbw/piper_robot_motion_control/experiments/project1/2026-10-25_five_profiles/runs/scheme1_out02 \
      --scheme 1 --direction outbound

检查准备结果，拍视频并确认路径清空、停止手段就绪后才人工执行：

    /usr/bin/python3 /home/nbw/piper_robot_motion_control/tools/ros2/five_profiles_experiment.py execute \
      /home/nbw/piper_robot_motion_control/experiments/project1/2026-10-25_five_profiles/runs/scheme1_out02 \
      --confirm-real-hardware --path-clear-and-stop-ready

目录已存在时换新编号，不覆盖。回程同样从实测起点单独prepare新目录。
