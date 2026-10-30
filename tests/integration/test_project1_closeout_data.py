"""保留收尾数据的只读回归；不导入ROS、不连接CAN、不发送指令。"""
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path
import unittest

REPO=Path(__file__).resolve().parents[2]
ROOT=REPO/'experiments/project1/2026-10-27_movej_repeat'
OUT=ROOT/'project1_closeout'

def table(path):
    with path.open(newline='') as f:return list(csv.DictReader(f))

def execution_path(record):
    return ROOT/record['group']/record['run_name']/Path(record['execution_dir']).name

def decoded_path(record):
    return OUT/'decoded'/Path(record['decoded_dir']).name

class CloseoutDataTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.inventory=json.loads((OUT/'inventory.json').read_text())

    def test_scope_and_sources_unchanged(self):
        records=self.inventory['records'];self.assertEqual(len(records),34)
        self.assertEqual(len(set(self.inventory['selected_record_numbers'])),30)
        for r in records:
            for name,expected in r['raw_hashes'].items():
                with self.subTest(source=name):
                    self.assertEqual(hashlib.sha256((ROOT/name).read_bytes()).hexdigest(),expected)

    def test_aligned_decoding_and_command_counts(self):
        for r in self.inventory['records']:
            with self.subTest(record=r['record']):
                decoder=json.loads((decoded_path(r)/'decode_summary.json').read_text())
                self.assertTrue(decoder['ordered_cycles']);self.assertEqual(decoder['span_over_2ms'],0)
                self.assertEqual(decoder['cycle_resets'],0)
                expected=1 if r['manifest']['mode']=='once' else 539
                self.assertEqual(len(table(execution_path(r)/'commands.csv')),expected)

    def test_main_outputs_and_unset_limits(self):
        rows=table(OUT/'matlab/run_metrics.csv');self.assertEqual(len(rows),34)
        figures=OUT/'matlab/figures'
        self.assertEqual(len(list(figures.glob('*.png'))),10)
        self.assertEqual(len(list(figures.glob('*.fig'))),10)
        self.assertEqual(list(ROOT.parent.glob('2026-10-24*')),[])
        for row in rows:
            self.assertTrue(math.isnan(float(row['joint_over_fraction'])))
            if row['run_name'].startswith(('A','B')):
                self.assertTrue(math.isnan(float(row['stream_joint_peak_deg'])))

    def test_stream_reference_is_latest_actual_publish(self):
        # 独立验证六轴C：不是按CSV行号相减，不通过时移隐藏原始偏差。
        r=next(r for r in self.inventory['records'] if r['record']==33)
        epoch=r['execution_summary']['trajectory_start_epoch_s']
        commands=table(execution_path(r)/'commands.csv')
        ct=[float(c['publish_return_epoch_s'])-epoch for c in commands]
        feedback=table(decoded_path(r)/'feedback_fk.csv')
        paired=[f for f in feedback if ct[0]<=float(f['timestamp_s'])-epoch<=r['manifest']['duration_s']]
        errors=table(OUT/'matlab/traces/33_stream_errors.csv');self.assertEqual(len(errors),len(paired))
        for k in (0,len(errors)//2,len(errors)-1):
            t=float(paired[k]['timestamp_s'])-epoch;index=bisect.bisect_right(ct,t)-1
            for j in range(1,7):
                expected=math.degrees(float(paired[k][f'q{j}_rad'])-float(commands[index][f'q{j}_rad']))
                self.assertAlmostEqual(float(errors[k][f'joint{j}_error_deg']),expected,places=8)

if __name__=='__main__':unittest.main()
