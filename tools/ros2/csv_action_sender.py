#!/usr/bin/env python3
"""CSV -> FollowJointTrajectory 离线执行工具。默认只检查，--execute才发送。

仅适用于本项目根命名空间的GenericSystem模拟控制器；不自动启动/使能真机、
不打开门控、不自动移动到起点。软件检查不是硬件隔离，请保持真机断电。
"""
import argparse
import csv
from dataclasses import replace
import json
import math
from pathlib import Path
import signal
import sys
import tempfile
import time

from csv_trajectory_preview import DEFAULT_CSV, JOINTS, load_csv, make_display_message

ACTION = "/arm_controller/follow_joint_trajectory"
STATE = "/offline_preview/joint_states"


def prepare_points(points):
    """只将近零静止端点导数置零；内部样本不变，原CSV不变。"""
    for point in points:
        if max(map(abs, point.velocities)) > 0.1 + 1e-9:
            raise ValueError("CSV速度超过本实验离线限值0.1 rad/s")
        if max(map(abs, point.accelerations)) > 0.1 + 1e-9:
            raise ValueError("CSV加速度超过本实验离线限值0.1 rad/s²")
    output = list(points)
    for index in (0, len(points) - 1):
        point = points[index]
        if (max(map(abs, point.velocities)) > 1e-6 or
                max(map(abs, point.accelerations)) > 1e-4):
            raise ValueError("轨迹不是近零静止端点，拒绝直接置零")
        output[index] = replace(point, velocities=[0.0]*6, accelerations=[0.0]*6)
    return output


def ordered_positions(names, positions):
    """按名字抽取六轴，拒绝缺失/重复名称和无效位置；不使用effort。"""
    if len(names) != len(positions) or len(names) != len(set(names)):
        raise ValueError("反馈名称/位置长度不匹配或名称重复")
    mapping = dict(zip(names, positions))
    q = [mapping[name] for name in JOINTS]
    if not all(math.isfinite(value) for value in q):
        raise ValueError("反馈关节角非有限数")
    return q


def validate_start(actual, target, tolerance=0.01):
    error = max(abs(a-b) for a, b in zip(actual, target))
    if error > tolerance:
        raise ValueError(f"当前角度与CSV起点不一致：最大偏差{error:.6g} rad > {tolerance} rad；请先完成离线起点过渡")
    return error


def execute(points, source_path=None):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from control_msgs.action import FollowJointTrajectory
    from controller_manager_msgs.srv import ListControllers, ListHardwareComponents

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("csv_offline_action_sender", use_global_arguments=False)
    stop = [False]
    old_handler = signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))
    handle = None
    completed = False
    files = []
    summary = {"action": ACTION, "state_topic": STATE, "success": False,
               "endpoint_derivatives_zeroed": True, "simulation_only": True,
               "source_csv": str(source_path.resolve()) if source_path else None,
               "sample_count": len(points), "duration_s": points[-1].time_ns/1e9}
    output = Path(tempfile.mkdtemp(prefix="piper_csv_action_"))
    print(f"本次记录目录：{output}", flush=True)
    latest = [None, 0.0]
    started = time.monotonic()

    def open_log(name, header):
        stream = (output/name).open("x", newline="", encoding="utf-8")
        files.append(stream)
        writer = csv.writer(stream)
        writer.writerow(header)
        return writer

    states = open_log("joint_states.csv", ["receive_elapsed_s", "header_stamp_s", *JOINTS])
    actions = open_log("action_feedback.csv", ["receive_elapsed_s", "desired_time_s",
        *[f"desired_q{i}" for i in range(1,7)], *[f"actual_q{i}" for i in range(1,7)],
        *[f"error_q{i}" for i in range(1,7)]])

    def state_callback(msg):
        try:
            q = ordered_positions(msg.name, msg.position)
        except (KeyError, ValueError):
            return  # 不更新有效反馈的接收时间；持续无效会超时。
        latest[:] = [q, time.monotonic()]
        states.writerow([latest[1]-started, msg.header.stamp.sec+msg.header.stamp.nanosec/1e9, *q])

    def feedback_callback(msg):
        f = msg.feedback
        actions.writerow([time.monotonic()-started,
            f.desired.time_from_start.sec+f.desired.time_from_start.nanosec/1e9,
            *f.desired.positions, *f.actual.positions, *f.error.positions])

    def wait_future(future, timeout, interruptible=True):
        deadline = time.monotonic()+timeout
        while not future.done():
            if interruptible and stop[0]:
                raise RuntimeError("用户请求取消")
            if time.monotonic() > deadline:
                raise RuntimeError("ROS请求超时")
            rclpy.spin_once(node, timeout_sec=0.05)
        result = future.result()
        if result is None:
            raise RuntimeError("ROS请求没有结果")
        return result

    def call(service, kind):
        client = node.create_client(kind, service)
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError(f"服务不可用：{service}")
        return wait_future(client.call_async(kind.Request()), 5)

    def check_graph():
        # 不允许ROS remap；这是误接防护，不是防恶意节点的安全边界。
        if any("agx_arm_ctrl" in name for name, _ in node.get_node_names_and_namespaces()):
            raise RuntimeError("发现agx_arm_ctrl节点，拒绝离线执行")
        pubs = node.get_publishers_info_by_topic(STATE)
        if len(pubs) != 1 or pubs[0].node_name != "joint_state_broadcaster":
            raise RuntimeError("隔离状态话题必须恰有一个joint_state_broadcaster发布者")
        for sub in node.get_subscriptions_info_by_topic(STATE):
            if not (sub.node_name in (node.get_name(), "robot_state_publisher") or
                    sub.node_name.startswith("move_group")):
                raise RuntimeError(f"隔离话题存在未预期订阅者：{sub.node_name}")

    try:
        subscription = node.create_subscription(JointState, STATE, state_callback, qos_profile_sensor_data)
        if subscription.topic_name != STATE:
            raise RuntimeError("状态话题被重映射")
        hardware = call("/controller_manager/list_hardware_components", ListHardwareComponents)
        if not hardware.component or any(c.plugin_name != "mock_components/GenericSystem" or
                c.state.label != "active" for c in hardware.component):
            raise RuntimeError("只允许全部硬件为active的mock_components/GenericSystem")
        controllers = call("/controller_manager/list_controllers", ListControllers)
        active = {c.name for c in controllers.controller if c.state == "active"}
        if not {"arm_controller", "joint_state_broadcaster"} <= active:
            raise RuntimeError("arm_controller和joint_state_broadcaster必须active")
        deadline = time.monotonic()+5
        while latest[0] is None:
            if time.monotonic() > deadline or stop[0]:
                raise RuntimeError("没有有效的离线关节状态，或用户取消")
            rclpy.spin_once(node, timeout_sec=0.05)
        check_graph()
        validate_start(latest[0], points[0].positions)
        print(f"模拟硬件、控制器、隔离话题与起点检查通过。准备执行{points[-1].time_ns/1e9:g}秒轨迹。", flush=True)
        client = ActionClient(node, FollowJointTrajectory, ACTION)
        if not client.wait_for_server(timeout_sec=5):
            raise RuntimeError("Action服务器不可用")
        # 等待服务器期间位置可能变化，发送前再次获取新状态并检查。
        rclpy.spin_once(node, timeout_sec=0.05)
        if stop[0] or time.monotonic()-latest[1] > 0.5:
            raise RuntimeError("用户取消或反馈过期")
        check_graph()
        validate_start(latest[0], points[0].positions)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = make_display_message(points).trajectory[0].joint_trajectory
        # 一旦发出请求，即使Ctrl+C也先等响应拿到句柄，以便取消已接受目标。
        sent_at = time.monotonic()
        pending = client.send_goal_async(goal, feedback_callback=feedback_callback)
        try:
            handle = wait_future(pending, 5, interruptible=False)
        except RuntimeError:
            print("目标响应超时：无法确认是否已接受！不要重复发送；检查离线控制器。", file=sys.stderr)
            raise
        if not handle.accepted:
            raise RuntimeError("Action拒绝目标")
        print("目标已接受；Ctrl+C请求取消（仅模拟控制器）。", flush=True)
        result_future = handle.get_result_async()
        deadline = time.monotonic()+points[-1].time_ns/1e9+10
        while not result_future.done():
            if stop[0] or time.monotonic() > deadline or time.monotonic()-latest[1] > 0.5:
                raise RuntimeError("用户取消、执行超时或关节反馈中断")
            rclpy.spin_once(node, timeout_sec=0.05)
        result = result_future.result()
        completed = True
        summary.update(status=result.status, error_code=result.result.error_code,
                       error_string=result.result.error_string,
                       action_elapsed_s=time.monotonic()-sent_at)
        if result.status != 4 or result.result.error_code != 0:
            raise RuntimeError(f"Action未成功：{summary}")
        # 给最终状态一个更新机会，再检查与目标的一致性。
        end = time.monotonic()+0.2
        while time.monotonic()<end:
            rclpy.spin_once(node, timeout_sec=0.02)
        if time.monotonic()-latest[1] > 0.5:
            raise RuntimeError("结束时反馈已过期")
        summary["final_error_rad"] = validate_start(latest[0], points[-1].positions)
        summary["success"] = True
        print(f"离线执行成功；Action耗时{summary['action_elapsed_s']:.3f}秒；"
              f"最终最大角度偏差{summary['final_error_rad']:.6g} rad。")
    except Exception as exc:
        summary["exception"] = str(exc)
        raise
    finally:
        if handle is not None and handle.accepted and not completed:
            try:
                cancel = wait_future(handle.cancel_goal_async(), 3, interruptible=False)
                summary["cancel_acknowledged"] = bool(cancel.goals_canceling)
                print(f"取消请求响应：{summary['cancel_acknowledged']}；不等同于硬件急停。")
            except Exception as exc:
                summary["cancel_error"] = str(exc)
                print("取消未确认，请检查离线控制器。", file=sys.stderr)
        for stream in files:
            stream.close()
        with (output/"summary.json").open("x", encoding="utf-8") as stream:
            json.dump(summary, stream, ensure_ascii=False, indent=2)
        node.destroy_node()
        rclpy.shutdown()
        signal.signal(signal.SIGINT, old_handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="?", type=Path, default=DEFAULT_CSV)
    parser.add_argument("--execute", action="store_true", help="明确授权发送给离线模拟Action")
    args = parser.parse_args()
    try:
        points = prepare_points(load_csv(args.csv))
        print(f"检查通过：{args.csv}\n{len(points)}点，{points[-1].time_ns/1e9:g}秒")
        print("起点：", points[0].positions, "\n终点：", points[-1].positions)
        print("起终点近零速度/加速度已在内存中置零；原CSV未修改。")
        if args.execute:
            execute(points, args.csv)
        else:
            print("仅检查，未初始化ROS。确认真机断电和隔离配置后，加--execute进行离线执行。")
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        print(f"失败：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
