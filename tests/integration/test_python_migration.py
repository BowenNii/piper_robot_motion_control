"""核查迁移产物及输入摘要；不连接ROS、不重跑MATLAB、不把旧结果当新计算。"""
import csv
import hashlib
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'analysis/src'))
from piper_analysis.robot_model.urdf import load_piper
from piper_analysis.kinematics.forward import forward
from piper_analysis.validation.pose import pose_error
BASE = ROOT/'experiments/project1/2026-10-27_movej_repeat/project1_closeout'


class MigrationTests(unittest.TestCase):
    def test_all_matlab_fields(self):
        with (BASE/'python/matlab_comparison.csv').open() as f:
            checks = list(csv.DictReader(f))
        self.assertGreater(len(checks), 30)
        self.assertTrue(all(r['passed'] == 'True' for r in checks))
        # 独立重新计算比较，不只相信报告中已有True。
        for filename in ['run_metrics.csv', 'response_metrics.csv']:
            with (BASE/'matlab'/filename).open() as f: old = list(csv.DictReader(f))
            with (BASE/'python'/filename).open() as f: new = list(csv.DictReader(f))
            self.assertEqual(len(old), len(new))
            for key in old[0]:
                try:
                    a = np.array([float(r[key]) for r in old]); b = np.array([float(r[key]) for r in new])
                except ValueError:
                    self.assertEqual([r[key] for r in old], [r[key] for r in new]); continue
                np.testing.assert_allclose(a, b, atol=1e-8, rtol=1e-7, equal_nan=True)

    def test_sources_unchanged(self):
        provenance = json.loads((BASE/'python/provenance.json').read_text())
        self.assertTrue(provenance['offline_only'])
        self.assertEqual(len(provenance['source_hashes']), 102)
        for path, expected in provenance['source_hashes'].items():
            self.assertEqual(hashlib.sha256((BASE.parent/path).read_bytes()).hexdigest(), expected, path)
        history = BASE/'python/history'
        hashes = json.loads((history/'source_hashes.json').read_text())
        self.assertEqual(len(hashes), 16)
        for path, expected in hashes.items():
            self.assertEqual(hashlib.sha256((ROOT/path).read_bytes()).hexdigest(), expected, path)

    def test_figures_and_history(self):
        figs = BASE/'python/figures'
        self.assertEqual(len(list(figs.glob('*.png'))), 10)
        self.assertEqual(len(list(figs.glob('*.svg'))), 10)
        for report in (BASE/'python/history').rglob('*comparison.csv'):
            with report.open() as f:
                self.assertTrue(all(r['passed'] == 'True' for r in csv.DictReader(f)), str(report))

    def test_trace_independent_zoh(self):
        model = load_piper()
        inventory = json.loads((BASE/'inventory.json').read_text())
        for r in inventory['records']:
            if r['group'] != 'multi_abc' or r['manifest']['mode'] != 'stream': continue
            num = r['record']
            with np.load(BASE/'python/traces'/f'{num:02d}_plot_data.npz', allow_pickle=False) as d:
                trace = np.genfromtxt(BASE/'python/traces'/f'{num:02d}_stream_errors.csv', names=True, delimiter=',')
                for i in (0, len(d['tr'])//2, len(d['tr'])-1):
                    j = np.searchsorted(d['ct'], d['tr'][i], side='right')-1
                    p, a = pose_error(forward(model, d['qtrace'][i]), forward(model, d['qc'][j]))
                    self.assertAlmostEqual(p*1000, trace['model_position_error_mm'][i], places=8)
                    self.assertAlmostEqual(np.rad2deg(a), trace['model_rotation_error_deg'][i], places=8)

    def test_interactive_reconstruction_headless(self):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from piper_analysis.plotting import view
        for number in range(1, 11):
            with patch.object(plt, 'show') as show:
                view(BASE/'python', number)
                show.assert_called_once()
                self.assertGreater(len(plt.gcf().axes), 0)
                plt.close('all')


if __name__ == '__main__': unittest.main()
