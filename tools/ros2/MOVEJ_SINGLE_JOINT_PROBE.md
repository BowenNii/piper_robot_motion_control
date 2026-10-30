# J1/J4/J5/J6 0.5rad单变量对照

## 六轴ABC综合对照

显式`prepare --all-joints`启用六轴模式，与`--joint`互斥。
共同目标固定为[0.3,0.3,-0.3,0.3,0.3,0.5]rad，不是六轴全0.5。
新目录：experiments/project1/2026-10-27_movej_repeat/multi_abc。
仍分A once、B repeat、C stream，每组去回程独立prepare/execute，不自动串联。
六轴模式逐轴检查起终点包络±0.01rad，stream保留全六轴跟随0.01rad检查；
不把单轴非运动轴保护简单删除后当作六轴模式。所有新目标重新按URDF复核。
固定目标组的内部各轴时间规律未知，C为共同五次进度，故不是严格同速度/加速度对照。
人工确认整臂扫掠空间；无碰撞检查，轻微抖动加剧/持续振荡/异响/报警应停止。
工具仍不切JS/MIT/CPV，不改增益或速度百分比，不自动使能。

    python3 tools/ros2/movej_single_joint_probe.py prepare /新的六轴目录 --all-joints --mode once --direction outbound

## 单轴说明

J4使用`prepare --joint 4`，新根目录`experiments/project1/2026-10-27_movej_repeat/j4_0p5`。
它会带动后续腕部/夹爪，必须重新检查0到0.5rad整段空间和线缆，不自动串联其他轴。

J5使用`prepare --joint 5`，新根目录`experiments/project1/2026-10-27_movej_repeat/j5_0p5`。
它会改变腕部/夹爪俯仰及重力负载，必须检查整个路径与台面、连杆和线缆的间隙；
不能照搬J6旋转的空间安全结论。没有碰撞检查或自动避障。

显式参数`prepare --joint 6`用于J6；`--joint 1`用于J1；`--joint 5`用于J5，不允许任意关节。
数据记录joint字段，包络保护对应指定轴，另外五轴均受非运动轴保护。
旧J1数据保持不变；代码升级后不得继续执行旧prepare目录，应新建目录重新准备。
J6使用新根目录`experiments/project1/2026-10-27_movej_repeat/j6_0p5`，不混入J1记录。
J6只旋转腕部末轴，不开合夹爪；其他五轴仍要求在零位附近。
需要人工确认工具/线缆旋转扫掠空间，不能把URDF限位通过当成碰撞安全证明。

仅MOVE J；不切JS/MIT/CPV，不改变驱动速度百分比，不自动使能或回零。
J1 0↔0.5rad，其他五轴保持准备时的实测角度（要求在零位±0.005rad）。
范围变大不代表自动安全，必须人工确认整个扫掠空间、夹爪/线缆和停止手段。
0.5rad约28.65°。旧0.1rad实验保留，不与新幅度混为单变量比较。

## 三种模式

- once：观察窗口开始时只发一次最终目标，然后监测，不补发。
- repeat：每20ms发同一个最终目标，共539次。
- stream：每20ms发五次位置曲线上的中间目标，共539点。

三组命令/观察窗口都是10.76s，并不表示实际运动时长相同。
stream规划峰值约4.992°/s；其他两组由固件决定实际速度，加速度未知。
因此A/B主要比较重复发送，B/C的差别还包含目标大小及实际速度变化，不能单凭结果认定根因。
每段前后各3秒CAN。原始CAN、feedback.csv、status.csv、commands.csv和summary.json自动保存。
固定目标组没有已知内部q(t)，不能按stream曲线计算其跟随误差。

## 五个终端分工

1. 原有驱动，保持运行，fast_mode=false/speed_percent=5/auto_enable=false。
2. prepare/check/execute，每段人工单独调用。
3. 旧ROS bag正常Ctrl+C退出后空闲；新工具已有数值反馈采集，不另开录包。
4. 旧candump正常Ctrl+C退出后空闲；工具自动启动/结束自己的candump。
5. 查看日志/summary，不发送目标、不调用move_home。

数据根目录：experiments/project1/2026-10-27_movej_repeat/j1_0p5。
目录名建议A_out01、A_return01、B_out01、B_return01、C_out01、C_return01。
目录已存在时换新编号，禁止覆盖或重放中断轨迹。

示例：

    python3 tools/ros2/movej_single_joint_probe.py prepare /新目录 --joint 1 --mode once --direction outbound
    python3 tools/ros2/movej_single_joint_probe.py check /新目录
    python3 tools/ros2/movej_single_joint_probe.py execute /新目录 --confirm-real-hardware --path-clear-and-stop-ready

回程必须另prepare新目录，direction=return；不是自动倒放，不用move_home替代。
工具要求驱动已使能；不自动使能。prepare关闭门控，execute检查后开门，结束关闭。
保护包含反馈/状态新鲜度、图中唯一控制源、起点变化0.001rad、
J1段包络±0.01rad、其他轴变化0.005rad、stream跟随0.01rad、迟到20ms、终点0.005rad。
这些是实验保护，不是碰撞安全认证；一帧异常可能在下次检查前已影响运动。
Ctrl+C请求软件保持和关门，不能代替硬件急停。失败后先现场确认状态，不重复execute。

## 离线验证

tests/integration/test_movej_single_joint_probe.py：目标生成、非法起点、三种发送模式、迟到停止、非运动轴变化停止。
只是假IO验证，不代表已做物理硬件认证。工具开发期间没有发送真机命令。
