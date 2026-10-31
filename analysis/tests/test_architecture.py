"""工程边界回归：避免算法再次依赖实验文件、绘图或兼容壳。"""
import ast
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / 'analysis/src/piper_analysis'
sys.path.insert(0, str(PACKAGE.parent))


class ArchitectureTests(unittest.TestCase):
    def test_algorithm_dependency_boundaries(self):
        pure = ['common', 'lie_group', 'kinematics', 'dynamics', 'identification', 'control', 'signals', 'validation']
        forbidden = {'data_io', 'plotting', 'experiments', 'robot', 'metrics', 'io', 'project1', 'project2', 'history'}
        for domain in pure:
            for path in (PACKAGE / domain).glob('*.py'):
                if path.name == '__init__.py':
                    continue  # 公共入口含显式兼容转发，实际算法不依赖它。
                for node in ast.walk(ast.parse(path.read_text())):
                    if isinstance(node, ast.ImportFrom) and node.level:
                        parts = (node.module or '').split('.')
                        self.assertFalse(set(parts) & forbidden, f'{path}: {node.module}')
                    if isinstance(node, ast.Import):
                        self.assertFalse(any(alias.name.startswith(('matplotlib', 'rclpy', 'can')) for alias in node.names), str(path))

    def test_plots_do_not_recompute_robot_algorithms(self):
        for path in (PACKAGE / 'plotting').glob('*.py'):
            for node in ast.walk(ast.parse(path.read_text())):
                if isinstance(node, ast.ImportFrom):
                    parts = set((node.module or '').split('.'))
                    self.assertFalse(parts & {'dynamics', 'kinematics', 'identification', 'robot_model', 'experiments'}, str(path))

    def test_legacy_imports_are_same_implementations(self):
        from piper_analysis.robot import forward as old_forward
        from piper_analysis.kinematics.forward import forward
        from piper_analysis.dynamics import computed_torque as old_control
        from piper_analysis.control.computed_torque import computed_torque
        from piper_analysis.project1 import analyze as old_analyze
        from piper_analysis.experiments.project1.pipeline import analyze
        self.assertIs(old_forward, forward)
        self.assertIs(old_control, computed_torque)
        self.assertIs(old_analyze, analyze)

    def test_core_import_does_not_load_visualization_or_ros(self):
        code = (
            f"import sys;sys.path.insert(0,{str(PACKAGE.parent)!r});"
            "from piper_analysis.dynamics.rnea import rnea;"
            "from piper_analysis.robot_model.urdf import load_piper;"
            "assert not any(k.startswith(('matplotlib','rclpy','piper_analysis.plotting',"
            "'piper_analysis.experiments','piper_analysis.data_io')) for k in sys.modules)"
        )
        subprocess.run([sys.executable, '-c', code], check=True)

    def test_time_response_and_delay_helpers(self):
        import numpy as np
        from piper_analysis.validation.response import joint_responses
        from piper_analysis.validation.alignment import fit_delay
        t = np.linspace(0, 1, 101)
        q = np.minimum(2 * t, 1)[:, None]
        rows = joint_responses(t, q, 0., np.array([0.]), np.array([1.]), np.array([0]))
        self.assertAlmostEqual(rows[0]['onset_1pct_s'], .01)
        self.assertAlmostEqual(rows[0]['arrival_99pct_s'], .5)
        commands = np.linspace(-.4, 1.1, 1501)
        actual = (t - .05)[:, None]
        lag, boundary = fit_delay(t, actual, commands, commands[:, None], [0], 1.)
        self.assertAlmostEqual(lag, .05)
        self.assertFalse(boundary)


if __name__ == '__main__':
    unittest.main()
