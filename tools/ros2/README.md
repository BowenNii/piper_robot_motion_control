# CSV轨迹的RViz预览

## 真机小幅IK轨迹（与模拟工具分开）

`real_ik_trajectory.py`提供prepare/check/execute三个独立步骤。
默认当前反馈位姿上移2mm、姿态不变，C++生成10秒501点CSV；
只有execute且显式确认才开门控并发布MOVE J。
操作步骤、实验阈值、记录内容与停止局限见[真机说明](REAL_IK_TRAJECTORY.md)。
不要为上真机而删除下方模拟Action工具的硬件检查。

## CSV Action 离线执行工具

`csv_action_sender.py`与下方预览工具不同：它向固定的
`/arm_controller/follow_joint_trajectory`发送真实Action请求，但只允许模拟控制器。
默认仅检查CSV，不初始化ROS；必须显式加`--execute`才发送。

前提：真机断电，不启动`agx_arm_ctrl`、不配置话题转发。离线demo使用
`follow:=false control_topic:=/offline_preview/joint_states`。
arm_controller及joint_state_broadcaster必须active，先单独完成到CSV起点的离线过渡。

```bash
source /opt/ros/jazzy/setup.bash
source /home/nbw/agx_arm_ws/install/setup.bash
# 只检查，默认原来的小幅CSV：
/usr/bin/python3 /home/nbw/piper_robot_motion_control/tools/ros2/csv_action_sender.py
# 起点就绪后，执行模拟Action：
/usr/bin/python3 /home/nbw/piper_robot_motion_control/tools/ros2/csv_action_sender.py --execute
```

检查包括：CSV原有格式/关节限位、离线速度/加速度均不超过0.1、静止端点导数
足够接近零才在内存中置零（速度≤1e-6、加速度≤1e-4；不改原文件），
全部硬件组件为active的`mock_components/GenericSystem`、控制器active、
隔离话题发布者及订阅者、关节反馈有效且新鲜、起点最大角度差≤0.01rad。
按名字读取关节，不依赖数组顺序；不使用模拟系统的NaN力矩。
不能将这些软件检查当作完整硬件隔离、安全认证或碰撞检查。

执行时Ctrl+C请求取消；反馈接收中断超过0.5秒或执行超时也请求取消。
取消响应仅表示模拟Action接受取消，不是硬件急停保障；目标响应超时可能无法获取
目标句柄，此时会报告状态不确定，不应重复发送。

每次新建并打印`/tmp/piper_csv_action_*`目录，保存`joint_states.csv`
（按名字排列的模拟关节位置及接收/消息时间）、`action_feedback.csv`
（控制器desired/actual/error）、`summary.json`（结果、错误及取消响应）。
记录中的actual是模拟状态，不是真机测量；临时目录需要长期保存时请另行归档。
脚本不会自动到达起点、不会开门控/使能/重试、不会自动重新规划。
如果重复运行时已处于CSV终点，会拒绝起点不一致，须先做新的离线过渡。

测试：

```bash
/usr/bin/python3 -m unittest discover -s tests/unit -p test_csv_action_sender.py -v
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 tests/integration/test_csv_action_sender_ros.py -v
```

集成测试使用独立ROS域137的假服务/Action服务器，不操作日常MoveIt控制器。
它验证发送、日志、起点拒绝及取消协议，不验证真实JointTrajectoryController的插值精度。

## 当前环境推荐方式 独立时钟播放

本机RViz 14.1.23与MoveIt 2.12.4组合存在动画刷新时间单位不匹配，
Planned Path可能瞬间播完或循环闪烁。CSV与DisplayTrajectory消息的时间仍为0～10秒，
不应修改CSV补偿这个显示问题。系统库未修改。

保留已经启动的离线RViz，在Displays中取消勾选MotionPlanning，避免旧动画和当前模型叠加。
点击Add，在By display里选择`moveit_rviz_plugin → RobotState`，不是RobotModel。
新显示项设置：

- Robot Description：`robot_description`。
- Robot State Topic：`/offline_preview/display_robot_state`。
- Robot Alpha：`1`；Visual Enabled开启，Collision Enabled关闭。

然后新终端运行：

```bash
source /opt/ros/jazzy/setup.bash
source /home/nbw/agx_arm_ws/install/setup.bash
/usr/bin/python3 /home/nbw/piper_robot_motion_control/tools/ros2/csv_trajectory_preview.py --state-playback
```

先显示起点倒计时3秒，再由Python单调时钟按原CSV播放10秒，终端每秒输出进度，
最后保留终点模型。脚本退出不会主动隐藏模型。重复运行重新播放，不自动循环。
程序只发布`moveit_msgs/DisplayRobotState`到固定的独立显示话题，
不发布JointState、不连接/control、不执行Action；不依赖Loop Animation和State Display Time。
操作系统和RViz渲染可能引入显示延迟，这不是硬实时执行实验。
保持真机断电，不启动控制驱动。本模式不做碰撞检查，也不替代真机安全评估。

### 大幅度预览轨迹（10秒）

`trajectory_wide_preview.csv`为独立的显示实验，原`trajectory.csv`未改动。
关节1从0.30转到0.80 rad，转动约28.65°，其余关节保持原角度。
沿用五次时间缩放、SE(3)插值和DLS求解；501点，离线限位/残差/步长/导数检查通过。
它未做碰撞或动力学验证，不用于真机执行。

```bash
/usr/bin/python3 /home/nbw/piper_robot_motion_control/tools/ros2/csv_trajectory_preview.py \
  /home/nbw/piper_robot_motion_control/experiments/project1/2026-10-18_ik_trajectory/data_csv/trajectory_wide_preview.csv \
  --state-playback
```

重新生成时须指定一个尚不存在的输出文件名：
`./build/debug/ik_trajectory_validation 新文件.csv 10 --wide-preview`。
不加`--wide-preview`仍使用原来的小幅度实验参数。

## 原Planned Path模式 当前版本不推荐

本目录是ROS显示适配工具，不属于数学core，也不控制真机。
`csv_trajectory_preview.py`把C++生成的时间轨迹CSV转换为
`moveit_msgs/msg/DisplayTrajectory`，只发布一次到`/display_planned_path`。
没有Action执行客户端、服务调用或控制话题Publisher，不改变真机状态。

## 1. RViz环境

真机保持断电，不启动agx_arm_ctrl。终端1：

```bash
source /opt/ros/jazzy/setup.bash
source /home/nbw/agx_arm_ws/install/setup.bash
ros2 launch agx_arm_moveit demo.launch.py \
  arm_type:=piper effector_type:=agx_gripper \
  follow:=false control_topic:=/offline_preview/joint_states
```

`follow:=false`本身不保证隔离真机；不要启动控制驱动或配置话题转发。
本工具不使用`/offline_preview/joint_states`；它是MoveIt模拟控制器的隔离输出。

## 2. 检查CSV并显示

终端2，无需编译CMake或colcon：

```bash
source /opt/ros/jazzy/setup.bash
source /home/nbw/agx_arm_ws/install/setup.bash
/usr/bin/python3 /home/nbw/piper_robot_motion_control/tools/ros2/csv_trajectory_preview.py --check-only
/usr/bin/python3 /home/nbw/piper_robot_motion_control/tools/ros2/csv_trajectory_preview.py
```

默认输入为本项目`experiments/project1/2026-10-18_ik_trajectory/data_csv/trajectory.csv`。
换文件时，在脚本路径后追加CSV绝对路径。必须有`t_s`及六轴q/dq/ddq列，
单位为s、rad、rad/s、rad/s²；每个时间点转换为一个JointTrajectoryPoint。
不重新求IK、不重定时、不插补CSV、不把输入写回文件；夹爪显示为0。
用`--check-only`时只用Python标准库，不初始化ROS或发布消息。

## 3. RViz显示设置

展开左侧Displays中的`MotionPlanning → Planned Path`：

- `Trajectory Topic`选择`/display_planned_path`。
- 打开轨迹显示/动画；若有`Loop Animation`，可启用反复播放。
- `State Display Time`设置为`1x`（一倍速），按CSV的`time_from_start`观看。
  这是MoveIt 2 Jazzy使用的写法，不是`REALTIME`；`3x`为三倍速。
  若仍为`0.05 s`，这是每个路径点的固定显示时间，501点可能播放约25秒，
  不能据此说原CSV的10秒参数错误。
- 不要点击`Plan`或`Plan & Execute`，它们不是加载CSV的步骤。
- 未运动的半透明模型可能仍停留在模拟当前状态，观察的是Planned Path模型。

程序最多等待显示订阅者15秒，发布一次后停留3秒用于通信，然后退出；
RViz自行播放整条轨迹。重新运行脚本可以重新发布，不是执行机器人动作。
没有订阅者时检查上面的显示项、ROS_DOMAIN_ID是否一致以及终端是否source。
不能使用conda或不兼容版本的Python替代ROS对应的`/usr/bin/python3`。

## 4. 边界和测试

这是几何动画，不是动力学仿真，也不自动调用MoveIt碰撞检查。
不检查厂家执行接口、力矩、连续点间安全或真机跟踪，不是硬件放行依据。
看到动画完成或界面状态OK，不能视为真实控制成功。

在仓库根目录运行：

```bash
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -m unittest discover -s tests/unit -p test_csv_trajectory_preview.py -v
```

测试覆盖归档501点、丢列/重复列、NaN/Inf、越界角度、非法时间、纳秒时间碰撞，
以及ROS消息的关节顺序、位置、导数与时间逐项保持一致。

显示通信测试（自动使用独立的ROS_DOMAIN_ID 91和92，不需要RViz）：

```bash
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 tests/integration/test_csv_preview_ros.py -v
```

核对完整消息收发、发布者只有显示话题及ROS基础话题、无人订阅时超时退出。
独立时钟模式另在domain 93实测501点按约10秒收发并保留最终状态；
单元测试用假时钟验证发布开销不会逐点累计到轨迹时长。
