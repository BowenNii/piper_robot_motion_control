import sys
from dataclasses import replace
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/"tools/ros2"))
from csv_trajectory_preview import DEFAULT_CSV, JOINTS, load_csv
from csv_action_sender import prepare_points, ordered_positions, validate_start


class SenderTests(unittest.TestCase):
    def test_endpoint_cleanup_copy(self):
        source = load_csv(DEFAULT_CSV)
        saved = source[-1].velocities[:]
        result = prepare_points(source)
        self.assertEqual(result[-1].velocities, [0]*6)
        self.assertEqual(result[0].accelerations, [0]*6)
        self.assertEqual(source[-1].velocities, saved)
        self.assertIs(result[10], source[10])

    def test_reject_moving_endpoint(self):
        source = load_csv(DEFAULT_CSV)
        source[-1] = replace(source[-1], velocities=[0.001]*6)
        with self.assertRaises(ValueError):
            prepare_points(source)

    def test_reject_interior_limit(self):
        source = load_csv(DEFAULT_CSV)
        source[10] = replace(source[10], accelerations=[0.2]*6)
        with self.assertRaises(ValueError):
            prepare_points(source)

    def test_names_not_first_six(self):
        self.assertEqual(ordered_positions(["gripper", *JOINTS], [9,1,2,3,4,5,6]), [1,2,3,4,5,6])
        for names, values in [(JOINTS, [0]*5), (JOINTS, [float('nan')]*6),
                              (["joint1"]*6, [0]*6)]:
            with self.assertRaises(ValueError):
                ordered_positions(names, values)

    def test_start_mismatch(self):
        self.assertEqual(validate_start([0]*6, [0]*6), 0)
        with self.assertRaises(ValueError):
            validate_start([0]*6, [.3]*6)


if __name__ == "__main__":
    unittest.main()
