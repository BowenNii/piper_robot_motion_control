"""端到端重构回归：只在临时目录重算，不覆盖正式结果、不连接硬件。"""
import csv
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'analysis/src'))
from piper_analysis.experiments.project2.demo import run_demo

BASELINE = ROOT / 'experiments/project1/2026-10-27_movej_repeat/project1_closeout/python'


def assert_csv_equal(test, expected: Path, actual: Path) -> None:
    with expected.open() as stream:
        left = list(csv.DictReader(stream))
    with actual.open() as stream:
        right = list(csv.DictReader(stream))
    test.assertEqual(len(left), len(right), str(expected))
    test.assertEqual(set(left[0]), set(right[0]), str(expected))
    for field in left[0]:
        try:
            a = np.array([float(row[field]) for row in left])
            b = np.array([float(row[field]) for row in right])
        except ValueError:
            test.assertEqual([row[field] for row in left], [row[field] for row in right])
            continue
        np.testing.assert_allclose(a, b, atol=1e-12, rtol=1e-12, equal_nan=True,
                                   err_msg=f'{expected}: {field}')


class RefactorRegressionTests(unittest.TestCase):
    def test_project1_full_pipeline(self):
        with tempfile.TemporaryDirectory(prefix='piper_refactor_test_') as folder:
            output = Path(folder) / 'result'
            command = [sys.executable, str(ROOT / 'analysis/scripts/analyze_project1.py'),
                       '--output', str(output)]
            subprocess.run(command, cwd=ROOT, check=True, timeout=110, capture_output=True, text=True)
            files = [path for path in BASELINE.rglob('*.csv') if not path.name.endswith('comparison.csv')]
            self.assertGreaterEqual(len(files), 17)
            for expected in files:
                assert_csv_equal(self, expected, output / expected.relative_to(BASELINE))
            caches = list(BASELINE.rglob('*.npz'))
            self.assertEqual(len(caches), 44)
            for expected in caches:
                with np.load(expected, allow_pickle=False) as left:
                    with np.load(output / expected.relative_to(BASELINE), allow_pickle=False) as right:
                        self.assertEqual(set(left.files), set(right.files))
                        for field in left.files:
                            np.testing.assert_allclose(left[field], right[field], atol=1e-12, rtol=1e-12, equal_nan=True)
            self.assertEqual(len(list(output.rglob('*.png'))), 32)
            self.assertEqual(len(list(output.rglob('*.svg'))), 32)
            # 再次调用必须拒绝覆盖，原有输出不应被重新打开写入。
            rejected = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
            self.assertNotEqual(rejected.returncode, 0)
            self.assertIn('Refusing to overwrite', rejected.stderr)

    def test_project2_demo(self):
        with tempfile.TemporaryDirectory(prefix='piper_dynamics_refactor_test_') as folder:
            output = run_demo(Path(folder) / 'demo')
            expected = ROOT / 'experiments/project2/nominal_demo_python/nominal_dynamics.csv'
            assert_csv_equal(self, expected, output / 'nominal_dynamics.csv')


if __name__ == '__main__':
    unittest.main()
