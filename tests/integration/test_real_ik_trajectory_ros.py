"""域96独立假驱动进程协议测试，不连接用户ROS域或CAN。"""
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
TOOL=ROOT/"tools/ros2/real_ik_trajectory.py"
ENV={**os.environ,"ROS_DOMAIN_ID":"96","ROS_AUTOMATIC_DISCOVERY_RANGE":"LOCALHOST",
     "RMW_FASTRTPS_PUBLICATION_MODE":"ASYNCHRONOUS"}

class FakeRobotTest(unittest.TestCase):
    def test_prepare_execute_reject_and_stop(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);folder=root/"trial"
            with (root/"fake_driver.log").open("w") as log:
                driver=subprocess.Popen([sys.executable,str(Path(__file__).with_name("fake_real_ik_driver.py")),tmp],
                    stdout=log,stderr=log,env=ENV)
                def run(*args):
                    return subprocess.run([sys.executable,str(TOOL),*map(str,args)],
                        capture_output=True,text=True,timeout=120,env=ENV)
                def state():return json.loads((root/"fake_state.json").read_text())
                try:
                    deadline=time.monotonic()+10
                    while not (root/"fake_state.json").exists():
                        self.assertIsNone(driver.poll(),(root/"fake_driver.log").read_text())
                        if time.monotonic()>deadline:self.fail("fake driver startup timeout")
                        time.sleep(.05)
                    prepared=run("prepare",folder)
                    self.assertEqual(prepared.returncode,0,prepared.stdout+prepared.stderr)
                    self.assertFalse(state()["gate"]);self.assertEqual(state()["sent"],0)
                    checked=run("check",folder)
                    self.assertEqual(checked.returncode,0,checked.stdout+checked.stderr)
                    refused=run("execute",folder)
                    self.assertNotEqual(refused.returncode,0);self.assertEqual(state()["sent"],0)
                    result=run("execute",folder,"--confirm-real-hardware")
                    self.assertEqual(result.returncode,0,result.stdout+result.stderr+str(state()))
                    self.assertFalse(state()["gate"]);self.assertEqual(state()["sent"],501)
                    self.assertEqual(state()["stops"],0)
                    completed=next(json.loads(p.read_text()) for p in folder.glob("execution_*/summary.json")
                                   if json.loads(p.read_text())["success"])
                    self.assertEqual(completed["sent_points"],501)
                    self.assertTrue(9.8<completed["command_elapsed_s"]<10.2)
                    refused=run("execute",folder,"--confirm-real-hardware")
                    self.assertNotEqual(refused.returncode,0)
                    self.assertIn("起点",refused.stderr);self.assertEqual(state()["sent"],501)
                    (root/"fake_control.json").write_text(json.dumps({"fault_after":15}))
                    fault=run("execute",folder,"--confirm-real-hardware")
                    self.assertNotEqual(fault.returncode,0,fault.stdout+fault.stderr)
                    self.assertFalse(state()["gate"]);self.assertGreaterEqual(state()["stops"],1)
                    self.assertLess(state()["sent"],501)
                    (root/"fake_control.json").write_text(json.dumps({"stale_after":15}))
                    stale=run("execute",folder,"--confirm-real-hardware")
                    self.assertNotEqual(stale.returncode,0,stale.stdout+stale.stderr)
                    self.assertIn("过期",stale.stderr)
                    self.assertFalse(state()["gate"]);self.assertGreaterEqual(state()["stops"],1)
                    self.assertLess(state()["sent"],501)
                    (root/"fake_control.json").write_text("{}")
                    with (root/"cancel.log").open("w") as output:
                        child=subprocess.Popen([sys.executable,str(TOOL),"execute",str(folder),
                            "--confirm-real-hardware"],stdout=output,stderr=output,env=ENV)
                        try:
                            deadline=time.monotonic()+15
                            while "指令进度 1.00s" not in (root/"cancel.log").read_text():
                                self.assertIsNone(child.poll(),(root/"cancel.log").read_text())
                                if time.monotonic()>deadline:self.fail("cancel test startup timeout")
                                time.sleep(.05)
                            child.send_signal(signal.SIGINT);child.wait(timeout=10)
                            self.assertNotEqual(child.returncode,0)
                            self.assertFalse(state()["gate"]);self.assertGreaterEqual(state()["stops"],1)
                            self.assertLess(state()["sent"],501)
                        finally:
                            if child.poll() is None:child.terminate();child.wait(timeout=10)
                finally:
                    driver.send_signal(signal.SIGINT)
                    try:driver.wait(timeout=5)
                    except subprocess.TimeoutExpired:driver.kill();driver.wait(timeout=5)

if __name__=="__main__":unittest.main()
