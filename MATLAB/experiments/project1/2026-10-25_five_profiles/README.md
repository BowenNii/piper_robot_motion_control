# 五组真机实验与官方运动：今日分析

明天先运行/阅读 `analyze_today.m`：它显式选择有效的十段记录，不会误选失败轮或旧方案2。
已生成图片可直接查看，不必重新运行。

在MATLAB中：

    cd('/home/nbw/piper_robot_motion_control/MATLAB/experiments/project1/2026-10-25_five_profiles');
    analyze_today;

完整对比输出：`experiments/project1/2026-10-25_five_profiles/comparison_analysis/figures`。
各单程完整12图：各有效`execution_*/analysis_complete/figures`。
复跑时已有单程结果直接加载，但对比目录已存在会拒绝覆盖，保护旧结果。

阅读顺序：

1. `analyze_today.m`：有效数据选择和整体流程。
2. `MATLAB/algorithm_library/experiment_analysis/compare_five_profiles.m`：统一20ms差分、腕部局部窗口、官方裁分及对比边界。
3. 同目录`analyze_project1_complete.m`：单轮时间对齐、已发参考、FK交叉验算与统计。绘图排版可略读。
4. `tools/ros2/export_official_bag.py`仅为离线MCAP提取，不控制机器人。

先看`outbound/return_06_wrist_angles_separate`和`07_wrist_velocity_separate`，逐方案看清腕部原始台阶和零速/反向；再看`04_wrist_detail`叠加、速度、电流、跟随误差，最后查看单轮通信/FK验证。
原始区间差分保留零速和反向，不用平滑掩盖台阶；20ms图只是统一网格估计，不是真实速度。
官方按首次可检测的运动对齐，没有内部规划参考和命令时间，不能计算其全过程跟随误差或启动延迟。
尾部后续关节变化不纳入正式官方运动对照。位姿误差不是外部实测精度。

## 原始离线参考说明

入口：运行本目录matlab_plot.m。C++生成器为apps/project1/joint_profile_comparison.cpp，时间规律封装在core/include/piper_control/trajectory/comparison_profile.hpp。

旧ik_trajectory_validation.cpp及旧数据不改动。本实验是固定终点IK解后的关节路径，不是SE(3)逐点IK，不要求末端走直线。用目标参考关节角作为IK初值固定解支，不能据此声称已经验证零位初值的IK收敛性。

生成命令（输出目录必须不存在，父目录已存在）：

    ./build/debug/joint_profile_comparison /新的绝对路径/输出目录

五组最大速度上限依次为1、5、8、5、5°/s。前三组五次位置曲线；第四组积分五次速度过渡；第五组梯形速度。方案4/5匹配方案2按20ms取整后的实际解析峰值速度和加速度，不匹配差分尖峰。短距离自动降低峰值速度。

重要更正：整程五次位置曲线的峰值加速度为(10/sqrt(3))*D/T²。最大位移0.5rad时，方案2约1.42859°/s²，不是先前口头估算的2.687°/s²。方案4如同时保持该加速度和相同速度，需要加减速距离约32.71°，超过当前28.65°；因此本次自动降到约4.67196°/s，没有匀速段。这样保住加速度匹配，但不再是速度与加速度同时匹配的严格对照。若坚持5°/s并保留匀速段，必须改变路径距离或提高加速度，不能三项同时满足。当前生成器不会擅自提高加速度或改变目标。

每组去/回程分别导出，都是均匀20ms采样。4/5解析运动结束到最后网格点可能有不足20ms的终点保持，不拉伸曲线，因此不破坏峰值匹配。方案5的加速度阶跃产生未定义的瞬时加加速度，CSV有限jerk字段不包含冲激，不能用它认证平滑性。

CSV包含解析角度、速度、加速度、加加速度、路径进度和加权物体雅可比最小奇异值。奇异值只记录，不做硬件安全阈值。离线检查关节角、模型速度限值、规划速度/加速度和相邻增量；不检查碰撞、负载力矩及真实控制接口。

回程仅是理论目标到零的离线参考。真机必须依据实测起点重新核对/生成，不能自动往返。数据结构刻意不兼容现有发送器，无真机执行入口。OFFLINE_ONLY.txt只在全部输出成功后写出，不代表允许真机执行。
