"""在单独的ROS_DOMAIN_ID中测试显示消息收发，不连接驱动、不执行Action。

运行前source ROS2 Jazzy，使用/usr/bin/python3。
本测试自己建立显示订阅者，无需RViz、MoveIt节点或真实机械臂。
"""
import os
from pathlib import Path
import subprocess
import sys
import time
import unittest

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from moveit_msgs.msg import DisplayTrajectory, DisplayRobotState

REPO = Path(__file__).resolve().parents[2]
SCRIPT = REPO / "tools/ros2/csv_trajectory_preview.py"
DOMAIN = 91  # 不与用户默认domain中的RViz/驱动交互。


class PreviewTransportTest(unittest.TestCase):
    def test_state_playback_ten_seconds(self):
        rclpy.init(args=[], domain_id=DOMAIN+2)
        node = rclpy.create_node("csv_state_test_receiver", use_global_arguments=False)
        received = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sub = node.create_subscription(DisplayRobotState, "/offline_preview/display_robot_state",
                                       lambda m: received.append((time.monotonic(), m)), qos)
        child = subprocess.Popen([sys.executable, str(SCRIPT), "--state-playback", "--wait-seconds", "5"],
                                 env=dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN+2)),
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            deadline=time.monotonic()+20
            topics=set()
            while time.monotonic()<deadline and child.poll() is None:
                rclpy.spin_once(node, timeout_sec=.01)
                for topic, _ in node.get_topic_names_and_types():
                    if any(info.node_name=="piper_csv_state_preview"
                           for info in node.get_publishers_info_by_topic(topic)):
                        topics.add(topic)
            out, err=child.communicate(timeout=3)
            self.assertEqual(child.returncode,0,out+err)
            self.assertGreater(len(received),400)
            self.assertIn("CSV进度 10.00s",out)
            self.assertLessEqual(topics,{"/offline_preview/display_robot_state","/rosout","/parameter_events"})
            # 初始3秒倒计时后，501点按10秒播放；从首个运动点到终点约9.98秒。
            motion=[(t,m) for t,m in received if m.state.joint_state.position[0]>.300000001]
            self.assertTrue(motion)
            self.assertGreater(motion[-1][0]-motion[0][0],9.5)
            self.assertLess(motion[-1][0]-motion[0][0],11)
            self.assertAlmostEqual(received[-1][1].state.joint_state.position[0],.35,places=8)
            self.assertFalse(received[-1][1].hide)
        finally:
            if child.poll() is None:
                child.terminate(); child.communicate(timeout=3)
            node.destroy_subscription(sub)
            node.destroy_node()
            rclpy.shutdown()

    def test_display_only_transport(self):
        rclpy.init(args=[], domain_id=DOMAIN)
        node = rclpy.create_node("csv_preview_test_receiver", use_global_arguments=False)
        received = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sub = node.create_subscription(DisplayTrajectory, "/display_planned_path", received.append, qos)
        env = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN))
        child = subprocess.Popen([sys.executable, str(SCRIPT), "--wait-seconds", "5"],
                                 env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            deadline = time.monotonic() + 8
            observed_topics = set()
            while time.monotonic() < deadline and child.poll() is None:
                rclpy.spin_once(node, timeout_sec=0.1)
                # DDS发现是异步的，节点尚未出现时不能直接按节点名查询。
                for topic, _ in node.get_topic_names_and_types():
                    if any(info.node_name == "piper_csv_preview"
                           for info in node.get_publishers_info_by_topic(topic)):
                        observed_topics.add(topic)
            stdout, stderr = child.communicate(timeout=3)
            self.assertEqual(child.returncode, 0, stdout + stderr)
            self.assertTrue(received, "没有收到DisplayTrajectory")
            self.assertIn("/display_planned_path", observed_topics)
            self.assertLessEqual(observed_topics,
                                 {"/display_planned_path", "/rosout", "/parameter_events"})
            traj = received[0].trajectory[0].joint_trajectory
            self.assertEqual(len(traj.points), 501)
            self.assertEqual(traj.points[-1].time_from_start.sec, 10)
            self.assertEqual(list(traj.points[0].positions), [.3, .3, -.6, .3, .3, .5])
        finally:
            if child.poll() is None:
                child.terminate()
                child.communicate(timeout=3)
            node.destroy_subscription(sub)
            node.destroy_node()
            rclpy.shutdown()

    def test_no_subscriber_timeout(self):
        # 独立domain且不建立接收者，必须明确报错而非假装发布成功。
        result = subprocess.run(
            [sys.executable, str(SCRIPT), "--wait-seconds", "0.3"],
            env=dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN + 1)),
            capture_output=True, text=True, timeout=8)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("没有显示订阅者", result.stderr)


if __name__ == "__main__":
    unittest.main()
