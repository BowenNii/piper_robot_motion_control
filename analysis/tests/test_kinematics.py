"""雅可比从实验脚本提取后的独立验证。"""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from piper_analysis.kinematics.forward import forward
from piper_analysis.kinematics.jacobian import jacobian_body
from piper_analysis.lie_group.transforms import inverse, log_transform
from piper_analysis.robot_model.urdf import load_piper
from piper_analysis.validation.conditioning import weighted_sigma_min


class KinematicsTests(unittest.TestCase):
    def test_body_jacobian_against_fk_increment(self):
        model = load_piper()
        q = np.array([.2, .3, -.5, .1, .2, -.1])
        dq = np.array([.1, -.2, .1, -.3, .1, .2])
        dt = 1e-6
        T = forward(model, q)
        numerical = log_transform(inverse(T) @ forward(model, q + dt * dq)) / dt
        np.testing.assert_allclose(jacobian_body(model, q, T) @ dq, numerical, atol=1e-7)
        self.assertAlmostEqual(weighted_sigma_min(np.diag([1., 2, 3, 4, 5, 6]), .2), .2)


if __name__ == '__main__':
    unittest.main()
