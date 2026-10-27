"""独立ROS域的假控制器测试；不接触用户正在运行的MoveIt。"""
import csv
import json
import os
import signal
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest

# 与日常ROS域隔离。运行此文件时不可继承已初始化的rclpy上下文。
os.environ["ROS_DOMAIN_ID"] = "137"
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from controller_manager_msgs.msg import ControllerState, HardwareComponentState

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/"tools/ros2"))
from csv_trajectory_preview import DEFAULT_CSV, JOINTS


class FakeExecutionTest(unittest.TestCase):
    def test_execute_and_logs(self):
        rclpy.init()
        node = rclpy.create_node("joint_state_broadcaster")
        group = ReentrantCallbackGroup()
        pub = node.create_publisher(JointState, "/offline_preview/joint_states", 10)
        current = [[.3,.3,-.6,.3,.3,.5]]
        received = []

        def tick():
            msg = JointState()
            msg.name = ["gripper", *JOINTS]
            msg.position = [0., *current[0]]
            pub.publish(msg)

        timer = node.create_timer(.02, tick, callback_group=group)

        def hardware(req, response):
            c = HardwareComponentState()
            c.name = "FakeSystem"
            c.plugin_name = "mock_components/GenericSystem"
            c.state.label = "active"
            response.component = [c]
            return response

        def controllers(req, response):
            for name in ("arm_controller", "joint_state_broadcaster"):
                c = ControllerState()
                c.name, c.state = name, "active"
                response.controller.append(c)
            return response

        services = [node.create_service(ListHardwareComponents,
            "/controller_manager/list_hardware_components", hardware, callback_group=group),
            node.create_service(ListControllers, "/controller_manager/list_controllers",
                controllers, callback_group=group)]

        def run_goal(handle):
            received.append(handle.request)
            for point in handle.request.trajectory.points:
                if handle.is_cancel_requested:
                    handle.canceled()
                    return FollowJointTrajectory.Result()
                current[0] = list(point.positions)
                f = FollowJointTrajectory.Feedback()
                f.joint_names = JOINTS
                f.desired = point
                f.actual = point
                f.error.positions = [0.]*6
                handle.publish_feedback(f)
                time.sleep(.005)
            handle.succeed()
            result = FollowJointTrajectory.Result()
            result.error_code = 0
            return result

        server = ActionServer(node, FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory", run_goal, callback_group=group,
            cancel_callback=lambda _: CancelResponse.ACCEPT)
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()
        try:
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory)/"trajectory.csv"
                # 缩短测试目标时间，仅验证协议/日志，不测真实控制器插值。
                with DEFAULT_CSV.open(newline="") as stream:
                    reader = csv.DictReader(stream)
                    fields, rows = reader.fieldnames, list(reader)
                for row in rows:
                    row["t_s"] = str(float(row["t_s"])*.1)
                with path.open("w", newline="") as stream:
                    writer = csv.DictWriter(stream, fieldnames=fields)
                    writer.writeheader()
                    writer.writerows(rows)
                result = subprocess.run([sys.executable, str(ROOT/"tools/ros2/csv_action_sender.py"),
                    str(path), "--execute"], capture_output=True, text=True, timeout=25)
                self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
                self.assertEqual(len(received), 1)
                self.assertEqual(len(received[0].trajectory.points), 501)
                self.assertEqual(list(received[0].trajectory.points[-1].velocities), [0]*6)
                log = next(line.split("：",1)[1] for line in result.stdout.splitlines()
                           if line.startswith("本次记录目录："))
                with (Path(log)/"summary.json").open() as stream:
                    self.assertTrue(json.load(stream)["success"])
                self.assertGreater((Path(log)/"joint_states.csv").stat().st_size, 100)
                self.assertGreater((Path(log)/"action_feedback.csv").stat().st_size, 100)
                # 当前已在终点；重复执行原CSV必须因起点不一致而拒绝。
                rejected = subprocess.run([sys.executable, str(ROOT/"tools/ros2/csv_action_sender.py"),
                    str(path), "--execute"], capture_output=True, text=True, timeout=20)
                self.assertNotEqual(rejected.returncode, 0)
                self.assertIn("起点不一致", rejected.stderr)
                self.assertEqual(len(received), 1)
                # 再回到起点，验证Ctrl+C会发取消请求。
                current[0] = [.3,.3,-.6,.3,.3,.5]
                process = subprocess.Popen([sys.executable, str(ROOT/"tools/ros2/csv_action_sender.py"),
                    str(path), "--execute"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                try:
                    deadline = time.monotonic()+10
                    while len(received)<2 and process.poll() is None and time.monotonic()<deadline:
                        time.sleep(.02)
                    self.assertEqual(len(received), 2)
                    process.send_signal(signal.SIGINT)
                    stdout, stderr = process.communicate(timeout=8)
                    self.assertNotEqual(process.returncode, 0)
                    cancel_log = next(line.split("：",1)[1] for line in stdout.splitlines()
                                      if line.startswith("本次记录目录："))
                    with (Path(cancel_log)/"summary.json").open() as stream:
                        self.assertTrue(json.load(stream)["cancel_acknowledged"], stdout+stderr)
                finally:
                    if process.poll() is None:
                        process.kill()
                        process.communicate()
        finally:
            executor.shutdown()
            thread.join(timeout=3)
            server.destroy()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
