# 60秒真机实验抖动排查

对象：run01/execution_mwd2hy56。只读既有记录与本机驱动/SDK；未发送真机命令，
未修改驱动、固件、速度、增益、控制模式或真机发送器。现场抖动由操作者报告，
没有同期视频/原始CAN记录，所以不能把所有记录跳变直接认定为物理往返。

复现：在MATLAB运行本目录matlab_diagnose_jitter.m。读取已有matlab_analysis/analysis_results.mat，
输出两张放大图、startup_stats/event_stats/window_stats.csv及MAT，均在原execution的
matlab_analysis/jitter_diagnostics内，不新增日期实验文件夹。

## 1. 5～10秒启动段：已确认事实

计划关节目标连续、单调。J4/J5每20ms的最大计划增量约0.00005778rad，J6约0.00009630rad。
但记录中J4/J5/J6在启动时有较长保持段，随后约0.0053rad（0.30度）的跳变。
J4在约7.6～8.0秒出现接近0与0.3度之间的多次切换，包括约0.00529rad反向跳变，
并非仅仅单调台阶跟随。J4误差峰值0.005685rad@7.975秒；J5为0.005258rad@7.745秒；
J6为0.005276rad@6.465秒。5～10秒的反馈消息时间戳最大间隔约5.10ms，接收间隔约5.70ms。
J4在此窗口有21次幅度超过0.001rad的反向记录变化；J5/J6没有同等级的反向变化。
以反馈离开初始值0.001rad为筛查条件，J4/J5/J6首次明显变化时最近目标累计角度
分别约0.284/0.301/0.302度。三者接近0.3度是值得核查的启动阈值线索，
但不能因此宣称固件存在已经证实的0.3度死区；不同关节摩擦和反馈行为也可能产生类似现象。

这段异常没有伴随同等量级的记录断流，也不是因为DLS每周期切换逆解分支：
本实验只在运动前求一次终点IK，之后发送固定、平滑的关节五次轨迹。
无原始CAN/视频，尚无法区分电机真实跳动、固件反馈行为、SDK解析/缓存或ROS组包问题。

## 2. 约36.175秒时间事件：已确认事实

第1808号目标的发送返回时刻约36.174797秒。返回时刻间隔34.324ms，
发送前调度迟到仅0.844ms，发送前检查至发送后记录之间约13.953ms。
同一时刻反馈接收间隔约14.453ms，但附近消息时间戳最大间隔约5.032ms。
这与主机线程调度/发布调用路径的短暂停顿相容；不能据此声称CAN电机执行周期为34ms。
现有日志没有publish入口/出口两侧完整细分计时，不能进一步归因到DDS或某个线程。

事件前0.5秒/后0.5秒：模型位置偏差峰值约0.604/0.492mm，姿态约0.255/0.255度，
没有看到该事件引起明显更大的偏差峰值。该现象不能解释6～8秒的启动异常或全程锯齿。
这不是宣称主机节拍完全无影响，只是现有数据不支持把它当作唯一根因。

## 3. 当前接口审查

本机pyAgxArm 1.0.0，历史驱动日志识别S-V1.9-0并采用v189配置；本次未重新查询真机。
本机ROS源码与install版本一致。/control/move_j回调按名称取六轴position，再调用SDK move_j。
没有使用轨迹点的time_from_start、velocity或acceleration，未向固件传入整条60秒轨迹。
因此上位机的五次时间律不等于固件严格按同一时间律执行。

SDK move_j将关节角按0.001度（约1.745e-5rad）量化，调用模式设置和关节目标发送。
自动模式设置默认开启；相同0x151模式帧默认有0.1秒重复节流，不能说每20ms必定重发模式帧。
没有在读取的ROS回调/SDK move_j中找到0.1度/0.3度的软件死区。
记录约0.1度/0.3度台阶明显大于0.001度的协议数值步距，不能简单用协议量化解释。
是否固件每次重置规划、低速死区如何设置等属于固件内部行为，当前源码不能确认。

SDK get_joint_angles分别读取joint12、joint34、joint56缓存，将joint56时间戳作为整体时间戳；
未在该函数内检查三组年龄差或形成锁保护原子快照。ROS joint_states还分别读取电机速度/力矩。
所以消息时间戳连续不代表六轴严格同步；但三组不同步本身也不能直接解释J4单轴在两个值间切换。
没有每组CAN原始帧，当前只能将反馈链路问题列为重点待查项。

SDK将move_js标为无平滑/无轨迹规划，并警告冲击、振荡风险，不建议为试错直接切换fast_mode/MIT。
当前MOVE J实现不提供时间轨迹执行保证，50Hz可发送也不等于已证明适配高质量实时跟踪。

## 可能原因及证据强弱

1. 优先排查反馈采集/解析/缓存链路：J4启动段二值式切换是直接线索，但需原始CAN验证。
2. 低速摩擦/死区或固件对密集MOVE J目标的处理：与保持后跳变及现场观察相容，尚未证实。
3. 主机发送/接收调度：确有一次事件，但时间与启动最大异常不一致，不能作为全程主因。
4. 动力学/负载/机构因素：不能排除，但没有力矩控制实验或可靠的同源状态证据，暂不认定。
5. IK求解分支跳变：当前运动期间没有逐点求逆，且实发计划平滑，不支持此解释。

## 晚上复测：不改控制方案，补齐观测

若曾出现明显冲击、碰撞、异响、姿态失稳或报警，不按原轨迹直接重跑；先处理现场问题。
否则，在确认安全、已有驱动维持5%/fast_mode=false、已按验证流程回零后，
新建run02执行同一目标、同一60秒轨迹，只做一次基线重复。
原run01不能覆盖或直接从目标姿态重复执行。prepare仍要求新鲜近零反馈。

```bash
source /opt/ros/jazzy/setup.bash
source /home/nbw/agx_arm_ws/install/setup.bash
/usr/bin/python3 /home/nbw/piper_robot_motion_control/tools/ros2/real_ik_trajectory.py prepare-endpoint \
 /home/nbw/piper_robot_motion_control/experiments/project1/2026-10-24_zero_to_pose/run02
```

准备成功后，另开终端被动记录原始CAN（不改变接口参数，不发送CAN控制帧）：

```bash
timeout --signal=INT 90s candump -L can0 > /home/nbw/piper_robot_motion_control/experiments/project1/2026-10-24_zero_to_pose/run02/can_run02.log
```

该终端保持记录，90秒会结束；请在开始记录后尽快在原终端执行下面命令。
同时录制现场视频，包含完整机械臂，尤其关注前10秒腕部动作，口述/标记开始时刻。
视频时间不是精确同步时钟，不用于宣称毫秒级相位关系。

```bash
/usr/bin/python3 /home/nbw/piper_robot_motion_control/tools/ros2/real_ik_trajectory.py execute \
 /home/nbw/piper_robot_motion_control/experiments/project1/2026-10-24_zero_to_pose/run02 \
 --confirm-real-hardware
```

若出现明显抖动/冲击或趋势恶化，不等60秒结束：Ctrl+C请求电子保持并关门控；
如无效使用现场停止措施。软件保持不是硬件急停。一次没抖也不能证明问题彻底消失。
返回时提供新execution目录、can_run02.log和视频，优先对照0x2A6中的J4原始值与ROS反馈。
本次没有修改发送器或增益，避免同时更改多个变量使对照失去意义。

## 核对的源码

- ROS回调：/home/nbw/agx_arm_ws/src/agx_arm_ros/src/agx_arm_ctrl/agx_arm_ctrl/agx_arm_ctrl_single_node.py
- SDK：/home/nbw/.local/lib/python3.12/site-packages/pyAgxArm/protocols/can_protocol/drivers/piper/default/driver.py
- v189/v188驱动、default/parser.py、core/driver_context.py、core/arm_driver_abstract.py。
- 官方SDK源码：https://github.com/agilexrobotics/pyAgxArm/blob/master/pyAgxArm/protocols/can_protocol/drivers/piper/default/driver.py
- 官方固件参考：https://github.com/agilexrobotics/pyAgxArm/blob/master/docs/piper/firmware_reference.md

结论以本机安装代码与实际记录为准；在线master可能与本机版本不同，不能据此推断固件内部实现。
