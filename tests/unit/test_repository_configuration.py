"""构建/离线配置回归，只读取仓库文件，不调用机器人或ROS服务。"""
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools/ros2'))
from prepare_project1_closeout import load_baseline

class ConfigurationTests(unittest.TestCase):
    def test_presets_and_safe_defaults(self):
        p=json.loads((ROOT/'CMakePresets.json').read_text())
        self.assertEqual(p['version'],3)
        configs={v['name']:v for v in p['configurePresets']}
        for name in ('debug','release'):
            c=configs[name]['cacheVariables']
            self.assertEqual(c['BUILD_TESTING'],'ON')
            self.assertEqual(c['PIPER_BUILD_PLACEHOLDER_APPS'],'OFF')
            self.assertEqual(c['PIPER_ENABLE_EXPERIMENT_DATA_TESTS'],'OFF')
        self.assertEqual(configs['project1']['cacheVariables']['PIPER_ENABLE_EXPERIMENT_DATA_TESTS'],'ON')
        self.assertEqual({p['configurePreset'] for p in p['testPresets']},set(configs))

    def test_analysis_limits_are_unset_not_hardware_limits(self):
        c=json.loads((ROOT/'config/project1_analysis.json').read_text())
        self.assertEqual(c['schema_version'],1)
        self.assertEqual(c['scope'],'offline_analysis_only')
        for key in ('joint_tolerance_deg','position_tolerance_mm','orientation_tolerance_deg'):
            self.assertIsNone(c[key])

    def test_existing_baseline_remains_available_without_archive(self):
        baseline=load_baseline(ROOT/'experiments/project1/2026-10-27_movej_repeat/project1_closeout/decode_comparison.csv')
        if not baseline:self.skipTest('source-only checkout has no optional experiment data')
        self.assertEqual(len(baseline),34)
        self.assertGreater(baseline[('j1_0p5','A_out01')]['max_span_s'],.005)

    def test_missing_baseline_is_not_fabricated(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(load_baseline(Path(tmp)/'absent.csv'),{})

    def test_documented_local_entrypoints_exist(self):
        for name in ('CMakeLists.txt','CMakePresets.json','experiments/project1/README.md',
                     'docs/PROJECT_STATUS.md','MATLAB/run_all_tests.m',
                     'MATLAB/experiments/project1/2026-10-27_movej_repeat/analyze_today.m'):
            self.assertTrue((ROOT/name).is_file(),name)

if __name__=='__main__':unittest.main()
