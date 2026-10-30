"""纯离线检查和假IO执行，不连接ROS/CAN。"""
import csv
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools/ros2'))
import movej_single_joint_probe as p


class ProbeTest(unittest.TestCase):
    def test_targets(self):
        for direction in ('outbound','return'):
            start=[0. if direction=='outbound' else .5,0,0,0,0,0]
            for mode in p.MODES:
                goal,qs=p.targets(start,direction,mode)
                self.assertEqual(len(qs),539)
                self.assertEqual(qs[-1],goal)
                self.assertTrue(all(q[1:]==start[1:] for q in qs))
                if mode=='stream':
                    self.assertEqual(qs[0],start)
                    self.assertLess(max(abs(b[0]-a[0]) for a,b in zip(qs,qs[1:])),.00175)
                else:self.assertTrue(all(q==goal for q in qs))

    def test_invalid_start(self):
        for q in ([.1,0,0,0,0,0],[0,.01,0,0,0,0],[float('nan'),0,0,0,0,0]):
            with self.assertRaises(ValueError):p.targets(q,'outbound','stream')

    def session(self,folder,mode,joint=1,direction='outbound'):
        folder.mkdir();start=[0.]*6
        if direction=='return':
            if joint=='all':start=list(p.MULTI_GOAL)
            else:start[joint-1]=.5
        goal,qs=p.targets(start,direction,mode,joint)
        with (folder/'targets.csv').open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['index','target_time_s','should_publish',*[f'q{i}_rad' for i in range(1,7)]])
            for k,q in enumerate(qs):w.writerow([k,k*p.DT,int(mode!='once' or k==0),*q])
        kind='movej_multi_joint_probe_v1' if joint=='all' else 'movej_single_joint_probe_v2'
        p.save_json(folder/'manifest.json',dict(kind=kind,joint=joint,mode=mode,direction=direction,
            q_start_rad=start,q_goal_rad=goal,duration_s=p.DURATION,samples=p.COUNT,driver_config={},
            targets_sha256=p.digest(folder/'targets.csv'),urdf_sha256=p.digest(p.DEFAULT_URDF),
            sender_sha256=p.digest(Path(p.__file__))))
        p.save_json(folder/'ready.json',{'prepared':True})

    def run_fake(self,mode,late=False,other_axis=False,joint=1,direction='outbound'):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp)/'run';self.session(folder,mode,joint,direction);clock=types.SimpleNamespace(value=0.)
            initial=json.loads((folder/'manifest.json').read_text())['q_start_rad']
            class IO:
                def __init__(self,*a,**kw):
                    self.q=list(initial);self.started=0.;self.files=[];self.factory=kw['buffer_factory']
                    self.status=types.SimpleNamespace(motion_status=0)
                    def publish(msg):
                        self.q=list(msg.position)
                        if other_axis:self.q[1]=.01
                    self.publisher=types.SimpleNamespace(publish=publish,get_subscription_count=lambda:1)
                    self.node=types.SimpleNamespace(get_subscriptions_info_by_topic=lambda _: [
                        types.SimpleNamespace(node_name='agx_arm_ctrl_single_node',node_namespace='/')])
                def ready(self):pass
                def healthy(self):pass
                def check_graph(self):pass
                def stationary(self):return self.q
                def gate(self,value):pass
                def spin(self,seconds):clock.value+=seconds
                def log(self,path,name,header):
                    f=(path/name).open('w',newline='');b=self.factory(header);self.files.append((f,b));return b
                def stop_and_close(self,stop):return {'gate_close':'closed','stop_requested':stop}
                def close(self):
                    for f,b in self.files:csv.writer(f).writerows(b.export_rows());f.close()
            class Capture:
                def __init__(self,*a,**kw):
                    kw['stdout'].write(b'CAN test\n');kw['stdout'].flush();self.returncode=None
                def poll(self):return self.returncode
                def send_signal(self,_):self.returncode=0
                def communicate(self,**kw):return None,b''
            def monotonic():
                clock.value+=.00001
                if late and clock.value>4:clock.value+=.03
                return clock.value
            fake_time=types.SimpleNamespace(monotonic=monotonic,time=lambda:1000+clock.value)
            module=types.ModuleType('sensor_msgs.msg');module.JointState=type('JointState',(),{})
            with patch.object(p,'RobotIO',IO),patch.object(p,'driver_configuration',return_value={}), \
                 patch.object(p,'time',fake_time),patch.object(p.subprocess,'Popen',Capture), \
                 patch.object(p.shutil,'which',return_value='/fake/candump'), \
                 patch.dict(sys.modules,{'sensor_msgs.msg':module}):
                if late or other_axis:
                    with self.assertRaises(RuntimeError):p.execute(folder)
                else:p.execute(folder)
            result=json.loads(next(folder.glob('execution_*/summary.json')).read_text())
            self.assertEqual(result['success'],not(late or other_axis))
            self.assertEqual(result['cleanup']['stop_requested'],late or other_axis)
            if result['success']:self.assertEqual(result['sent_points'],1 if mode=='once' else 539)

    def test_once(self):self.run_fake('once')
    def test_repeat(self):self.run_fake('repeat')
    def test_stream(self):self.run_fake('stream')
    def test_lateness_stop(self):self.run_fake('stream',late=True)
    def test_other_axis_stop(self):self.run_fake('repeat',other_axis=True)

    def test_j6_targets(self):
        for direction in ('outbound','return'):
            for mode in p.MODES:
                start=[0.]*6
                if direction=='return':start[5]=.5
                goal,qs=p.targets(start,direction,mode,6)
                self.assertTrue(all(q[:5]==start[:5] for q in qs))
                self.assertEqual(qs[-1],goal)
                if mode=='stream':self.assertEqual(qs[0],start)

    def test_j6_all_modes_both_directions(self):
        for direction in ('outbound','return'):
            for mode in p.MODES:
                with self.subTest(direction=direction,mode=mode):self.run_fake(mode,joint=6,direction=direction)

    def test_j6_nonmoving_axis_stop(self):self.run_fake('repeat',joint=6,other_axis=True)

    def test_j6_late_stop(self):self.run_fake('stream',joint=6,late=True)

    def test_j6_envelope(self):
        io=types.SimpleNamespace(q=[0,0,0,0,0,.52],healthy=lambda:None)
        with self.assertRaisesRegex(RuntimeError,'J6超出'):p.guard(io,[0.]*6,[0,0,0,0,0,.5],6)

    def test_reject_unvalidated_joint(self):
        for joint in (0,2,3,7,True):
            with self.assertRaises(ValueError):p.targets([0.]*6,'outbound','once',joint)

    def test_j5_targets(self):
        for direction in ('outbound','return'):
            for mode in p.MODES:
                start=[0.]*6
                if direction=='return':start[4]=.5
                goal,qs=p.targets(start,direction,mode,5)
                self.assertTrue(all(all(q[j]==start[j] for j in (0,1,2,3,5)) for q in qs))
                self.assertEqual(qs[-1],goal)
                if mode=='stream':self.assertEqual(qs[0],start)

    def test_j5_all_modes_both_directions(self):
        for direction in ('outbound','return'):
            for mode in p.MODES:
                with self.subTest(direction=direction,mode=mode):self.run_fake(mode,joint=5,direction=direction)

    def test_j5_nonmoving_axis_stop(self):self.run_fake('repeat',joint=5,other_axis=True)

    def test_j5_late_stop(self):self.run_fake('stream',joint=5,late=True)

    def test_j5_envelope(self):
        io=types.SimpleNamespace(q=[0,0,0,0,.52,0],healthy=lambda:None)
        with self.assertRaisesRegex(RuntimeError,'J5超出'):p.guard(io,[0.]*6,[0,0,0,0,.5,0],5)

    def test_j4_targets(self):
        for direction in ('outbound','return'):
            for mode in p.MODES:
                start=[0.]*6
                if direction=='return':start[3]=.5
                goal,qs=p.targets(start,direction,mode,4)
                self.assertTrue(all(all(q[j]==start[j] for j in (0,1,2,4,5)) for q in qs))
                self.assertEqual(qs[-1],goal)
                if mode=='stream':self.assertEqual(qs[0],start)

    def test_j4_all_modes_both_directions(self):
        for direction in ('outbound','return'):
            for mode in p.MODES:
                with self.subTest(direction=direction,mode=mode):self.run_fake(mode,joint=4,direction=direction)

    def test_j4_nonmoving_axis_stop(self):self.run_fake('repeat',joint=4,other_axis=True)

    def test_j4_late_stop(self):self.run_fake('stream',joint=4,late=True)

    def test_j4_envelope(self):
        io=types.SimpleNamespace(q=[0,0,0,.52,0,0],healthy=lambda:None)
        with self.assertRaisesRegex(RuntimeError,'J4超出'):p.guard(io,[0.]*6,[0,0,0,.5,0,0],4)

    def test_multi_targets(self):
        for direction in ('outbound','return'):
            start=[0.]*6 if direction=='outbound' else list(p.MULTI_GOAL)
            for mode in p.MODES:
                goal,qs=p.targets(start,direction,mode,'all')
                self.assertEqual(goal,p.MULTI_GOAL if direction=='outbound' else [0.]*6)
                self.assertEqual(qs[-1],goal)
                if mode=='stream':
                    self.assertEqual(qs[0],start)
                    for a,b in zip(qs,qs[1:]):
                        self.assertLess(max(abs(x-y) for x,y in zip(a,b)),.00175)
                else:self.assertTrue(all(q==goal for q in qs))

    def test_multi_all_modes_both_directions(self):
        for direction in ('outbound','return'):
            for mode in p.MODES:
                with self.subTest(direction=direction,mode=mode):self.run_fake(mode,joint='all',direction=direction)

    def test_multi_late_stop(self):self.run_fake('stream',joint='all',late=True)

    def test_multi_each_axis_envelope(self):
        for index in range(6):
            for value in (min(0,p.MULTI_GOAL[index])-.02,max(0,p.MULTI_GOAL[index])+.02):
                q=[0.]*6;q[index]=value
                io=types.SimpleNamespace(q=q,healthy=lambda:None)
                with self.assertRaisesRegex(RuntimeError,f'J{index+1}超出'):
                    p.guard(io,[0.]*6,p.MULTI_GOAL,'all')

    def test_multi_wrong_start(self):
        with self.assertRaises(ValueError):p.targets([.1,0,0,0,0,0],'outbound','once','all')
        with self.assertRaises(ValueError):p.targets([0.]*6,'return','stream','all')


if __name__=='__main__':unittest.main()
