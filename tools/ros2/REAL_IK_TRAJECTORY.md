# 真机小幅IK轨迹实验

本页的10秒/2mm限制用于原prepare模式。新增prepare-endpoint为独立的
零位到指定T实验：60秒3001点，终点DLS求解后走关节五次路径。
目标仅限本次参考角[0.3,0.3,-0.3,0.3,0.3,0.5]对应位姿；不保证末端直线。
操作见MATLAB/experiments/project1/2026-10-24_zero_to_pose/README.md。
原小幅模式的幅度限制没有放宽，两种模式不能混用CSV。
原10月24日2mm记录现位于archives/2026-10-24_preliminary/experiments/project1下；
以下small_ik_followup路径仅供以后新建实验使用，不重建已归档的日期目录。

程序：`tools/ros2/real_ik_trajectory.py`。数学计算仍由C++的
`apps/project1/ik_trajectory_validation.cpp`完成。默认在当前反馈位姿上，
沿基座Z平移2mm、保持模型末端link6姿态，五次时间缩放，10秒、501点。

与模拟`csv_action_sender.py`分开；没有删除或绕过模拟工具的硬件检查。
本工具直接以50Hz发布`/control/move_j`位置目标，不经模拟Action控制器。
驱动默认MOVE J会再次平滑，因此10秒仅为上位机发送时长，不保证真机严格跟踪时间曲线。
不宣称末端实测精度、硬实时或碰撞安全。

## 开始前

- 机械臂已通过此前的小幅单点实验，处于正常CAN控制状态；只保留一个根命名空间的驱动。
- 保留现有驱动：piper、can0、auto_enable=false、fast_mode=false、speed_percent=5。
- 关闭MoveIt、其他发送器和其他运动话题发布者；不再运行预览或离线Action执行。
- 保持机械臂当前姿态，不回零；prepare会从新反馈取起点，不使用聊天中旧角度。
- 人工确认机械臂/夹爪/线缆与桌面、周围物体无干涉，停止措施已准备好。
- 工具不自动使能。驱动的启动参数auto_enable=false并不表示机械臂已经失能。
- 不需要另启rosbag记录：本工具自动记录指令、真实反馈与状态。

## 操作步骤

代码更改后首次编译（已编译的情况下可跳过）：

```bash
cmake --build /home/nbw/piper_robot_motion_control/build/debug --target ik_trajectory_validation -j 4
```

新终端加载ROS环境：

```bash
source /opt/ros/jazzy/setup.bash
source /home/nbw/agx_arm_ws/install/setup.bash
```

准备新实验；此步只会请求关闭控制门控、读取反馈、生成文件，不发运动目标：

```bash
/usr/bin/python3 /home/nbw/piper_robot_motion_control/tools/ros2/real_ik_trajectory.py prepare \
  /home/nbw/piper_robot_motion_control/experiments/project1/small_ik_followup/run01
```

目录必须不存在，已有目录换run02等新名字，不覆盖/删除旧实验。
如果准备失败，查看generator.log；不得使用未生成manifest.json的不完整目录执行。
prepare会关闭门控，不能在其他终端同时请求运动或开门。

可选纯离线复核（不初始化ROS）：

```bash
/usr/bin/python3 /home/nbw/piper_robot_motion_control/tools/ros2/real_ik_trajectory.py check \
  /home/nbw/piper_robot_motion_control/experiments/project1/small_ik_followup/run01
```

准备成功、当前位置未变化且现场确认安全后，显式执行：

```bash
/usr/bin/python3 /home/nbw/piper_robot_motion_control/tools/ros2/real_ik_trajectory.py execute \
  /home/nbw/piper_robot_motion_control/experiments/project1/small_ik_followup/run01 \
  --confirm-real-hardware
```

execute再次检查，随后开门控，约10秒发送501点，终点观察2秒，再关门控。
只发送六轴position，不发送夹爪目标，不自动修改速度百分比、使能、回零或切MIT。
第一次完成后不要直接重复execute：当前位置已变，应在新目录重新prepare。
如果起点差超过0.001rad，程序拒绝，不会自行移动到旧起点。

## 实验自设检查与保护

这些阈值不是厂家参数、物理精度指标或安全认证：

- 当前反馈取名映射六轴，时间戳须持续更新；角度与状态接收及时间戳更新年龄均≤0.25秒。
- 同机ROS时钟与关节消息时间戳差绝对值≤0.25秒，防止积压旧消息被误认为新反馈。
- 状态必须为CAN控制、MOVE J、未报告故障，六轴超限及通信异常标志均为false。
- 近期0.5秒关节位置范围≤0.001rad；不以静止时有非零值的电机速度字段推导关节速度。
- 相对目标位移≤2mm，单轴相对起点最大变化≤0.05rad，相邻目标差≤0.002rad。
- 必须501点、均匀20ms、10秒；从q重新差分检查导数，不只相信CSV的dq/ddq列。
- 离线峰值速度≤0.02rad/s、加速度≤0.05rad/s²，起终点导数接近零。
- 实际反馈与最近已发目标最大关节差≤0.01rad；终点观察后最大差≤0.005rad且状态报告到位。
- 发送晚到一个完整采样周期（20ms）即终止，不跨周期集中补发。
- CSV、C++可执行程序和URDF摘要须与prepare时一致；原CSV不裁剪、不放大，不人工补偿关节偏差。
- 执行前检查没有其他运动发布者；命令接收者仅允许预期驱动，其他命令订阅器也会被拒绝。
  图发现查询不放入定时发送循环，执行期间禁止另外启动任何发送器/改模式；这不是访问控制或安全隔离。
- 本工具prepare/execute有进程锁，不允许并行运行；不能阻止其他SDK或人工操作。

## 停止与限制

运行中Ctrl+C/SIGTERM、反馈中断、状态故障、跟随偏差或调度异常会终止后续发送。
已发过目标的异常退出：先请求`/emergency_stop`电子保持，再请求`/control_enable=false`。
正常完成：只关门控。不会自动失能，避免失能后姿态失去支撑。

电子保持服务返回不等于物理急停成功。当前驱动实现是根据当前反馈重发位置目标；
通信/驱动失效时可能无效，Empty响应也不提供物理停止确认。
门控只拒绝后续运动话题，不撤销已在底层执行的目标。
进程崩溃、kill -9、主机断电、CAN中断均没有硬件级停止保障，须保留现场停止措施。
准备或执行发生错误时不要盲目重试、放宽保护线或反复补发终点。

## 文件与结果

实验目录包含：

- trajectory.csv：C++导出的完整501点轨迹；不覆盖原实验。
- manifest.json：起终角、位移、生成参数、导数统计和文件摘要。
- generator.log：C++求解与检查输出。
- execution_*/commands.csv：目标采样时间、实际发送经过时间、调度晚到量、六轴命令。
- execution_*/feedback.csv：真实编码器角、反馈时间戳、接收时间、驱动报告速度/力矩。
- execution_*/status.csv：CAN模式、故障和到位状态等。
- execution_*/summary.json：成功/失败、发送点数、最终关节偏差、停止请求与门控清理结果。

新记录的summary同时保存trajectory_start_receive_elapsed_s，供MATLAB将指令
时间与反馈接收时间按同一单调时钟对齐。旧记录缺少该字段，只能做有说明的
时间估计；不要直接按行相减或用曲线拟合时移来掩盖跟随滞后。

短实验日志有界缓存在内存中，关闭门控后写盘，避免磁盘写入阻塞反馈/发送。
异常被程序正常处理时仍保存；进程崩溃、强杀或主机断电可能丢失尚未落盘的记录。

速度字段标为reported，来自电机状态，不假定等于编码器关节角差分。
两类反馈及发布并非同步原子快照；接收时间不能直接作为单向通信延迟。
所谓success仅为程序自设条件通过，不能据此宣称机器人达到某个末端精度或可无人值守。

## 开发测试

```bash
/usr/bin/python3 -m unittest discover -s tests/unit -p test_real_ik_trajectory.py -v
source /opt/ros/jazzy/setup.bash
source /home/nbw/agx_arm_ws/install/setup.bash
/usr/bin/python3 tests/integration/test_real_ik_trajectory_ros.py -v
```

集成测试只在独立ROS域96（本机发现）内使用独立进程假驱动，默认200Hz反馈，
可用环境变量FAKE_FEEDBACK_HZ=50复测50Hz反馈。不连接CAN或日常ROS域。
测试检查prepare不运动、完整发送、起点拒绝、故障/断流/Ctrl+C保持和关门控；
不验证真实动力学、CAN发送能力或固件连续MOVE J效果，不能替代真机验证。
