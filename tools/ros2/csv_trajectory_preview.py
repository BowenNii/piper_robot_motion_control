#!/usr/bin/env python3
"""CSV -> RViz模型预览，仅显示，不执行。

默认读取本项目10-18实验的时间轨迹。先用--check-only验证输入，再运行预览。
不创建Action客户端、服务客户端，不发布JointState或/control/*。
数学算法仍由C++生成、MATLAB验算，本文件只做格式检查和ROS消息转换。
推荐--state-playback：由本程序的单调时钟控制播放，绕开MoveIt旧版动画计时问题。
"""

import argparse
import csv
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[2]
DEFAULT_CSV = REPO / "experiments/project1/2026-10-18_ik_trajectory/data_csv/trajectory.csv"
DEFAULT_URDF = REPO / "models/urdf/piper_description.urdf"
DISPLAY_TOPIC = "/display_planned_path"  # 固定显示话题，不提供任意控制话题选项。
STATE_TOPIC = "/offline_preview/display_robot_state"
JOINTS = [f"joint{i}" for i in range(1, 7)]
Q_COLS = [f"q{i}_rad" for i in range(1, 7)]
DQ_COLS = [f"dq{i}_rad_s" for i in range(1, 7)]
DDQ_COLS = [f"ddq{i}_rad_s2" for i in range(1, 7)]


@dataclass
class PreviewPoint:
    """一行CSV：时间ns，六轴位置rad、速度rad/s和加速度rad/s²。"""
    time_ns: int
    positions: list[float]
    velocities: list[float]
    accelerations: list[float]


def read_joint_limits(urdf: Path) -> list[tuple[float, float]]:
    """URDF -> joint1～joint6的角度边界；这是模型检查，不是真机安全认证。"""
    root = ET.parse(urdf).getroot()
    limits = []
    for name in JOINTS:
        joint = root.find(f"joint[@name='{name}']")
        if joint is None or joint.get("type") != "revolute":
            raise ValueError(f"URDF缺少旋转关节：{name}")
        item = joint.find("limit")
        if item is None:
            raise ValueError(f"URDF缺少limit：{name}")
        lower, upper = float(item.attrib["lower"]), float(item.attrib["upper"])
        if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
            raise ValueError(f"URDF角度边界不合法：{name}")
        limits.append((lower, upper))
    return limits


def load_csv(path: Path, urdf: Path = DEFAULT_URDF) -> list[PreviewPoint]:
    """CSV -> 经检查的样本；必须包含t_s、六轴q/dq/ddq，不补列、不删除异常行。

    检查有限值、角度限位、从0开始的严格递增时间与ROS纳秒可表示性。
    仅转换CSV，不重做FK/IK、导数验算、动力学或碰撞检查。
    """
    limits = read_joint_limits(urdf)
    required = ["t_s", *Q_COLS, *DQ_COLS, *DDQ_COLS]
    points = []
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames or []
        if len(fields) != len(set(fields)):
            raise ValueError("CSV包含重复列名")
        missing = set(required) - set(fields)
        if missing:
            raise ValueError("CSV缺少时间轨迹列：" + ", ".join(sorted(missing)))
        for line, row in enumerate(reader, start=2):
            if None in row or any(row.get(key) is None for key in fields):
                raise ValueError(f"CSV第{line}行列数不一致")
            try:
                values = [float(row[key]) for key in required]
            except (TypeError, ValueError) as exc:
                raise ValueError(f"CSV第{line}行存在空值或非数值") from exc
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"CSV第{line}行存在NaN或Inf")
            t = values[0]
            if t < 0 or t >= 2**31:
                raise ValueError(f"CSV第{line}行时间超出ROS Duration范围")
            ns = round(t * 1_000_000_000)
            if ns >= 2**31 * 1_000_000_000:
                raise ValueError(f"CSV第{line}行时间舍入后超出ROS Duration范围")
            if (not points and t != 0) or (points and ns <= points[-1].time_ns):
                raise ValueError(f"CSV第{line}行时间必须从0开始且按纳秒严格递增")
            q, dq, ddq = values[1:7], values[7:13], values[13:19]
            for name, angle, (lower, upper) in zip(JOINTS, q, limits):
                if not lower <= angle <= upper:
                    raise ValueError(f"CSV第{line}行{name}角度超出URDF边界")
            points.append(PreviewPoint(ns, q, dq, ddq))
    if len(points) < 2:
        raise ValueError("CSV至少需要两个时间采样点")
    return points


def make_display_message(points: list[PreviewPoint]):
    """已检查样本 -> 显示消息，时间和导数均来自CSV，不重新规划。

    延迟导入ROS依赖，使--check-only和CSV单元测试无需启动ROS。
    夹爪仅在模型中显示为0，不产生夹爪控制命令。
    """
    from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint

    display = DisplayTrajectory()
    # model_id留空，使用RViz已加载的模型，避免piper/agx_arm名称不一致。
    display.trajectory_start.is_diff = True
    display.trajectory_start.joint_state.name = [*JOINTS, "gripper"]
    display.trajectory_start.joint_state.position = [*points[0].positions, 0.0]
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.header.frame_id = "base_link"
    trajectory.joint_trajectory.joint_names = list(JOINTS)
    for sample in points:
        point = JointTrajectoryPoint()
        point.positions = sample.positions
        point.velocities = sample.velocities
        point.accelerations = sample.accelerations
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(
            sample.time_ns, 1_000_000_000
        )
        trajectory.joint_trajectory.points.append(point)
    display.trajectory.append(trajectory)
    return display


def publish_preview(points: list[PreviewPoint], wait_seconds: float) -> None:
    """等待显示订阅者并发布一次；只创建一个显示Publisher，不执行轨迹。

    在发布前再次检查解析后的话题名，禁止通过ROS remap改成控制话题。
    发布者停留3秒用于DDS传输；10秒动画由RViz自己播放，无需逐行发送。
    """
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from moveit_msgs.msg import DisplayTrajectory

    rclpy.init(args=[])  # 不将脚本命令行作为ROS remap参数。
    node = None
    try:
        node = rclpy.create_node("piper_csv_preview", use_global_arguments=False)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publisher = node.create_publisher(DisplayTrajectory, DISPLAY_TOPIC, qos)
        if publisher.topic_name != DISPLAY_TOPIC:
            raise RuntimeError("显示话题被改名，拒绝发布")
        print(f"等待RViz订阅 {DISPLAY_TOPIC}，最多{wait_seconds:g}秒……", flush=True)
        deadline = time.monotonic() + wait_seconds
        while publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError("没有显示订阅者：请启动RViz并检查MotionPlanning的Trajectory Topic")
            rclpy.spin_once(node, timeout_sec=0.1)
        publisher.publish(make_display_message(points))
        print("已发布一次显示轨迹；未发送控制命令，未做碰撞检查。", flush=True)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def make_state_message(point: PreviewPoint):
    """单个CSV样本 -> DisplayRobotState，仅用于独立的RobotState显示项。"""
    from moveit_msgs.msg import DisplayRobotState
    msg = DisplayRobotState()
    msg.hide = False
    msg.state.is_diff = True
    msg.state.joint_state.name = [*JOINTS, "gripper"]
    msg.state.joint_state.position = [*point.positions, 0.0]
    msg.state.joint_state.velocity = [*point.velocities, 0.0]
    return msg


def replay_samples(points, emit, clock=time.monotonic, wait=time.sleep):
    """按绝对单调时钟截止时间发布CSV各行，避免逐点sleep累计漂移。

    不修改CSV、不插值、不放大动作；操作系统调度和RViz帧率仍可能造成显示延迟。
    emit接收(样本, 已过秒数)，返回实际总时长；可注入假时钟进行确定性测试。
    """
    start = clock()
    for point in points:
        deadline = start + point.time_ns / 1e9
        while True:
            remaining = deadline - clock()
            if remaining <= 0:
                break
            wait(remaining)
        emit(point, clock() - start)
    return clock() - start


def publish_state_playback(points: list[PreviewPoint], wait_seconds: float) -> None:
    """只发布DisplayRobotState：起点停留3秒 -> 按CSV时间播放 -> 保留终点。

    不依赖Planned Path的1x/Loop Animation，也不发布JointState控制消息。
    RobotState显示插件保存最后收到的姿态，发布者退出后不主动隐藏它。
    """
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from moveit_msgs.msg import DisplayRobotState

    rclpy.init(args=[])
    node = None
    try:
        node = rclpy.create_node("piper_csv_state_preview", use_global_arguments=False)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publisher = node.create_publisher(DisplayRobotState, STATE_TOPIC, qos)
        if publisher.topic_name != STATE_TOPIC:
            raise RuntimeError("状态显示话题被改名，拒绝发布")
        print(f"等待RobotState显示订阅 {STATE_TOPIC}，最多{wait_seconds:g}秒……", flush=True)
        deadline = time.monotonic() + wait_seconds
        while publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError("没有RobotState订阅者：请添加moveit_rviz_plugin/RobotState并设置Robot State Topic")
            rclpy.spin_once(node, timeout_sec=0.1)
        # 显示起点并留出观察时间；不将此停留计入CSV的10秒。
        for count in (3, 2, 1):
            publisher.publish(make_state_message(points[0]))
            print(f"起点已显示，{count}秒后开始播放……", flush=True)
            deadline = time.monotonic() + 1
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.01)
        print(f"开始播放：CSV时长{points[-1].time_ns/1e9:g}秒，只显示、不执行。", flush=True)
        last_second = -1

        def emit(point, elapsed):
            nonlocal last_second
            publisher.publish(make_state_message(point))
            second = int(point.time_ns / 1e9)
            if second != last_second:
                print(f"CSV进度 {point.time_ns/1e9:.2f}s；实际经过 {elapsed:.3f}s", flush=True)
                last_second = second

        elapsed = replay_samples(points, emit,
                                 wait=lambda remaining: rclpy.spin_once(
                                     node, timeout_sec=min(remaining, 0.01)))
        print(f"播放结束：实际经过 {elapsed:.3f}秒；保留终点模型。未发送任何控制指令。", flush=True)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="?", type=Path, default=DEFAULT_CSV,
                        help="含t_s、q1～q6、dq1～dq6和ddq1～ddq6的CSV")
    parser.add_argument("--check-only", action="store_true", help="只检查CSV，不初始化ROS、不发布")
    parser.add_argument("--state-playback", action="store_true",
                        help="推荐：独立时钟逐点播放，RViz需添加RobotState显示项")
    parser.add_argument("--wait-seconds", type=float, default=15,
                        help="等待显示订阅者的秒数，默认15")
    args = parser.parse_args()
    if not math.isfinite(args.wait_seconds) or not 0 < args.wait_seconds <= 60:
        parser.error("--wait-seconds必须在0～60秒内且大于0")
    try:
        points = load_csv(args.csv)
        print(f"CSV检查通过：{args.csv}\n{len(points)}点，总时长{points[-1].time_ns/1e9:g}秒")
        print("起点关节角(rad)：", points[0].positions)
        print("终点关节角(rad)：", points[-1].positions)
        if args.check_only:
            print("仅检查结束；未初始化ROS，也未发布任何消息。")
        elif args.state_playback:
            publish_state_playback(points, args.wait_seconds)
        else:
            print("注意：旧版MoveIt播放器可能瞬间播完；当前环境请优先使用--state-playback。")
            publish_preview(points, args.wait_seconds)
        return 0
    except ImportError as exc:
        print(f"ROS依赖不可用：{exc}。请source ROS2 Jazzy，使用/usr/bin/python3运行。", file=sys.stderr)
    except (OSError, ValueError, KeyError, ET.ParseError, RuntimeError) as exc:
        print(f"预览失败：{exc}", file=sys.stderr)
    except KeyboardInterrupt:
        print("预览发布已取消；本程序不控制真机。", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
