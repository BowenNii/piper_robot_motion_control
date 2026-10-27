"""标准库CSV测试无需ROS；消息转换测试在source Jazzy后自动启用。"""
import csv
from pathlib import Path
import sys
import tempfile
import unittest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools/ros2"))
from csv_trajectory_preview import (  # noqa: E402
    DEFAULT_CSV, DEFAULT_URDF, DISPLAY_TOPIC, JOINTS,
    load_csv, make_display_message, make_state_message, replay_samples,
)


class PreviewTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / "sample.csv"
        with DEFAULT_CSV.open(newline="") as f:
            reader = csv.DictReader(f)
            self.fields = reader.fieldnames
            self.rows = list(reader)

    def tearDown(self):
        self.tmp.cleanup()

    def write(self):
        with self.path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.fields)
            writer.writeheader()
            writer.writerows(self.rows)

    def reject(self):
        self.write()
        with self.assertRaises(ValueError):
            load_csv(self.path)

    def test_archived_csv(self):
        points = load_csv(DEFAULT_CSV)
        self.assertEqual(len(points), 501)
        self.assertEqual(points[0].time_ns, 0)
        self.assertEqual(points[1].time_ns, 20_000_000)
        self.assertEqual(points[-1].time_ns, 10_000_000_000)
        self.assertEqual(points[0].positions, [.3, .3, -.6, .3, .3, .5])

    def test_missing_column(self):
        self.fields.remove("t_s")
        for row in self.rows:
            del row["t_s"]
        self.reject()

    def test_nan(self):
        self.rows[1]["q1_rad"] = "nan"
        self.reject()

    def test_invalid_joint(self):
        self.rows[1]["q2_rad"] = "-1"
        self.reject()

    def test_duplicate_time(self):
        self.rows[1]["t_s"] = "0"
        self.reject()

    def test_subnanosecond_collision(self):
        self.rows[1]["t_s"] = "1e-12"
        self.reject()

    def test_nonzero_start(self):
        self.rows[0]["t_s"] = ".01"
        self.reject()

    def test_negative_time(self):
        self.rows[1]["t_s"] = "-1"
        self.reject()

    def test_time_overflow(self):
        self.rows[-1]["t_s"] = str(2**31)
        self.reject()

    def test_bad_acceleration(self):
        self.rows[20]["ddq6_rad_s2"] = "inf"
        self.reject()

    def test_one_row(self):
        self.rows = self.rows[:1]
        self.reject()

    def test_duplicate_header(self):
        self.fields.append("t_s")
        self.reject()

    def test_missing_urdf(self):
        with self.assertRaises(OSError):
            load_csv(DEFAULT_CSV, Path(self.tmp.name) / "missing.urdf")

    def test_monotonic_replay(self):
        points = load_csv(DEFAULT_CSV)
        now = [100.0]
        received = []
        def clock():
            return now[0]
        def wait(seconds):
            now[0] += seconds
        def emit(point, elapsed):
            received.append((point.time_ns, elapsed))
            now[0] += .001  # 模拟每次发布耗时1ms，不能使501次耗时累计到计划时间。
        elapsed = replay_samples(points, emit, clock, wait)
        self.assertEqual(len(received), 501)
        self.assertAlmostEqual(received[-1][1], 10, places=9)
        self.assertAlmostEqual(elapsed, 10.001, places=9)
        for time_ns, actual in received:
            self.assertAlmostEqual(time_ns/1e9, actual, places=9)

    def test_state_message(self):
        try:
            import moveit_msgs.msg  # noqa: F401
        except ImportError:
            self.skipTest("source ROS2 Jazzy后可测试消息转换")
        point = load_csv(DEFAULT_CSV)[-1]
        msg = make_state_message(point)
        self.assertFalse(msg.hide)
        self.assertEqual(list(msg.state.joint_state.name), [*JOINTS, "gripper"])
        self.assertEqual(list(msg.state.joint_state.position), [*point.positions, 0.0])

    def test_display_message(self):
        try:
            import moveit_msgs.msg  # noqa: F401
        except ImportError:
            self.skipTest("source ROS2 Jazzy后可测试消息转换")
        points = load_csv(DEFAULT_CSV)
        msg = make_display_message(points)
        self.assertEqual(DISPLAY_TOPIC, "/display_planned_path")
        self.assertEqual(len(msg.trajectory), 1)
        traj = msg.trajectory[0].joint_trajectory
        self.assertEqual(list(traj.joint_names), JOINTS)
        self.assertEqual(len(traj.points), 501)
        self.assertEqual(traj.points[-1].time_from_start.sec, 10)
        self.assertEqual(traj.points[-1].time_from_start.nanosec, 0)
        for source, target in zip(points, traj.points):
            self.assertEqual(list(target.positions), source.positions)
            self.assertEqual(list(target.velocities), source.velocities)
            self.assertEqual(list(target.accelerations), source.accelerations)
            self.assertEqual(target.time_from_start.sec * 10**9
                             + target.time_from_start.nanosec, source.time_ns)
        self.assertEqual(list(msg.trajectory_start.joint_state.name), [*JOINTS, "gripper"])


if __name__ == "__main__":
    unittest.main()
