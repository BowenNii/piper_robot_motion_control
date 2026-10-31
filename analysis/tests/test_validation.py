"""python -m unittest discover -s analysis/tests -v；纯离线，不连接硬件。"""
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from piper_analysis.validation.statistics import trace_metrics
from piper_analysis.validation.path import path_distance
from piper_analysis.validation.timing import zoh_indices
from piper_analysis.data_io.config import load_config
from piper_analysis.signals import uniform_resample, estimate_derivatives, sample_derivatives, lowpass_offline


class MetricsTests(unittest.TestCase):
    def test_time_weighted(self):
        m = trace_metrics([0, 1, 3], [1, 3, 99], 2)
        self.assertAlmostEqual(m['rms'], np.sqrt(19/3))
        self.assertEqual(m['peak'], 99); self.assertAlmostEqual(m['over_fraction'], 2/3)
        self.assertEqual(m['longest_over_s'], 2)
        self.assertTrue(np.isnan(trace_metrics([0, 1], [1, 2])['over_fraction']))

    def test_path(self):
        np.testing.assert_allclose(path_distance([[.5, 1, 0], [2, 0, 0]], [[0, 0, 0], [1, 0, 0]]), [1, 1])
        np.testing.assert_allclose(path_distance([[1, 0, 0]], [[0, 0, 0], [0, 0, 0]]), [1])

    def test_zoh(self):
        np.testing.assert_array_equal(zoh_indices([0, .1, .2], [0, .05, .1, .2]), [0, 0, 1, 2])
        with self.assertRaises(ValueError): zoh_indices([0, 1], [-.1])

    def test_invalid(self):
        for t, e in [([0, 0], [1, 2]), ([0, 1], [-1, 0]), ([0, 1], [1, np.nan])]:
            with self.assertRaises(ValueError): trace_metrics(t, e)
        with self.assertRaises(ValueError): trace_metrics([0, 1], [1, 2], True)

    def test_config(self):
        with tempfile.TemporaryDirectory() as folder:
            file = Path(folder)/'config.json'
            file.write_text(json.dumps(dict(schema_version=1, scope='offline_analysis_only', position_tolerance_mm=None)))
            self.assertIsNone(load_config(file)['position_tolerance_mm'])
            file.write_text(json.dumps(dict(schema_version=1, scope='hardware')))
            with self.assertRaises(ValueError): load_config(file)

    def test_signals(self):
        t = np.linspace(0, 1, 101); q = np.column_stack([t**2, t**3])
        v, a = estimate_derivatives(t, q)
        np.testing.assert_allclose(v[:, 0], 2*t, atol=1e-10)
        np.testing.assert_allclose(a[:, 0], 2, atol=1e-9)
        vd, ad = sample_derivatives(q, .01)
        np.testing.assert_allclose(vd[:, 0], 2*t, atol=1e-12)
        np.testing.assert_allclose(ad[:, 0], 2, atol=1e-10)
        grid, values = uniform_resample(t, q, .02, .011)
        np.testing.assert_allclose(values[:, 0], grid**2, atol=1e-10)
        with self.assertRaises(ValueError): uniform_resample(t, q, .02, .005)
        self.assertEqual(lowpass_offline(q, 100, 10).shape, q.shape)

class ValidationHelpersTests(unittest.TestCase):
    def test_motion_phase_definition(self):
        from piper_analysis.validation.phases import split_motion_phases
        t = np.arange(6) / 10
        q = np.array([0., .002, .008, .01, .01, .01])[:, None]
        phase, bounds, segments = split_motion_phases(t, q, change_threshold_deg=.005)
        np.testing.assert_array_equal(phase, [1, 2, 2, 3, 3, 3])
        np.testing.assert_allclose(bounds, [.1, .2])
        self.assertEqual(segments, 1)

    def test_pure_baseline_comparison(self):
        from piper_analysis.validation.baseline import compare_tables
        expected = [dict(label='A', value=1.)]
        self.assertTrue(compare_tables(expected, [dict(label='A', value=1.)])[0]['passed'])
        self.assertFalse(compare_tables(expected, [dict(label='A', value=2.)])[0]['passed'])
        with self.assertRaises(ValueError):
            compare_tables(expected, [dict(label='B', value=1.)])


if __name__ == '__main__':
    unittest.main()
