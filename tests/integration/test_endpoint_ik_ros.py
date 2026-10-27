"""零位终点IK模式，域96假驱动；不连接CAN或用户日常ROS域。"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

ROOT=Path(__file__).resolve().parents[2]
TOOL=ROOT/'tools/ros2/real_ik_trajectory.py'
ENV={**os.environ,'ROS_DOMAIN_ID':'96','ROS_AUTOMATIC_DISCOVERY_RANGE':'LOCALHOST',
     'RMW_FASTRTPS_PUBLICATION_MODE':'ASYNCHRONOUS','FAKE_ZERO':'1'}

class EndpointTest(unittest.TestCase):
    def test_full_60_second_execution(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);folder=root/'trial'
            with (root/'driver.log').open('w') as output:
                driver=subprocess.Popen([sys.executable,str(Path(__file__).with_name('fake_real_ik_driver.py')),tmp],
                    env=ENV,stdout=output,stderr=output)
                def run(*args):return subprocess.run([sys.executable,str(TOOL),*map(str,args)],
                    env=ENV,capture_output=True,text=True,timeout=120)
                def state():return json.loads((root/'fake_state.json').read_text())
                try:
                    deadline=time.monotonic()+10
                    while not (root/'fake_state.json').exists():
                        self.assertIsNone(driver.poll())
                        if time.monotonic()>deadline:self.fail('fake driver startup timeout')
                        time.sleep(.05)
                    prepared=run('prepare-endpoint',folder)
                    self.assertEqual(prepared.returncode,0,prepared.stdout+prepared.stderr)
                    self.assertEqual(state()['sent'],0);self.assertFalse(state()['gate'])
                    self.assertEqual(json.loads((folder/'manifest.json').read_text())['sample_count'],3001)
                    checked=run('check',folder);self.assertEqual(checked.returncode,0,checked.stdout+checked.stderr)
                    done=run('execute',folder,'--confirm-real-hardware')
                    self.assertEqual(done.returncode,0,done.stdout+done.stderr)
                    self.assertEqual(state()['sent'],3001);self.assertFalse(state()['gate'])
                    self.assertEqual(state()['stops'],0)
                    summary=json.loads(next(folder.glob('execution_*/summary.json')).read_text())
                    self.assertTrue(summary['success'])
                    self.assertLess(abs(summary['command_elapsed_s']-60),.1)
                    self.assertGreater(summary['trajectory_start_receive_elapsed_s'],0)
                    rejected=run('execute',folder,'--confirm-real-hardware')
                    self.assertNotEqual(rejected.returncode,0);self.assertEqual(state()['sent'],3001)
                    self.assertFalse(state()['gate'])
                finally:
                    driver.send_signal(signal.SIGINT)
                    try:driver.wait(timeout=5)
                    except subprocess.TimeoutExpired:driver.kill();driver.wait(timeout=5)

if __name__=='__main__':unittest.main()
