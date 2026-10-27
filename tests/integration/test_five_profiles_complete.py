"""离线生成、独立校验、CAN解码及假IO执行测试；不连接ROS/CAN。"""
import csv
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

REPO=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(REPO/'tools/ros2'))
import five_profiles_experiment as experiment
from csv_trajectory_preview import load_csv
from numeric_log_buffer import NumericBuffer

class CompleteTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory();cls.root=Path(cls.temp.name)
        cls.plans=cls.root/'plans'
        subprocess.run([str(experiment.GEN),str(cls.plans)],check=True,capture_output=True)

    @classmethod
    def tearDownClass(cls):cls.temp.cleanup()

    def test_all_profiles_independent_equations(self):
        with (self.plans/'summary.csv').open() as f:configs=list(csv.DictReader(f))
        for c in configs:
            c={k:float(v) for k,v in c.items()}
            for direction in ('outbound','return'):
                pts=load_csv(self.plans/f"scheme{int(c['scheme'])}_{direction}.csv")
                experiment.validate(pts,c)
                pts[2].velocities[0]+=.01
                with self.assertRaises(ValueError):experiment.validate(pts,c)

    def test_measured_return_start(self):
        generated=self.root/'measured_return'
        start=[.299,.299,-.299,.299,.299,.499]
        subprocess.run([str(experiment.GEN),str(generated),'--start',*map(str,start),
            '--goal',*map(str,[0.]*6)],check=True,capture_output=True)
        with (generated/'summary.csv').open() as f:configs=list(csv.DictReader(f))
        for row in configs:
            c={k:float(v) for k,v in row.items()}
            pts=load_csv(generated/f"scheme{int(c['scheme'])}_outbound.csv")
            experiment.validate(pts,c)
            self.assertLess(experiment.difference(pts[0].positions,start),1e-12)
            self.assertLess(max(map(abs,pts[-1].positions)),1e-12)

    def test_decoder_units_and_wide_span(self):
        log=self.root/'synthetic.log';out=self.root/'decoded'
        # 三个样本分别跨度1ms、3ms、1ms。3ms样本应保留并计数。
        rows=[]
        for base,span in [(100.,.001),(100.01,.003),(100.02,.001)]:
            for k in range(6):rows.append(f'({base+k*span/5:.9f}) can0 {0x2A2+k:X}#0000000000000000\n')
            # 速度-1，电流-2A，位置-3，符号与缩放都必须正确。
            rows.append(f'({base+span+.00001:.9f}) can0 251#FC18F830FFFFF448\n')
            rows.append(f'({base+span+.00002:.9f}) can0 254#FC18F830FFFFF448\n')
        log.write_text(''.join(rows))
        subprocess.run([str(experiment.DECODER),str(log),str(out)],check=True,capture_output=True)
        d=json.loads((out/'decode_summary.json').read_text())
        self.assertEqual(d['fk_samples'],3);self.assertEqual(d['span_over_2ms'],1)
        with (out/'motors.csv').open() as f:m=list(csv.DictReader(f))
        self.assertAlmostEqual(float(m[0]['motor_velocity_reported']),-1)
        self.assertAlmostEqual(float(m[0]['motor_position_reported']),-3)
        self.assertAlmostEqual(float(m[0]['current_A']),-2)
        self.assertAlmostEqual(float(m[0]['torque_estimated_Nm']),-2*1.18125/4)
        self.assertAlmostEqual(float(m[1]['torque_estimated_Nm']),-2*.95844)
        self.assertNotEqual(subprocess.run([str(experiment.DECODER),str(log),str(out)],capture_output=True).returncode,0)

    def make_session(self,name):
        folder=self.root/name;folder.mkdir()
        import shutil
        shutil.copyfile(self.plans/'scheme2_outbound.csv',folder/'trajectory.csv')
        with (self.plans/'summary.csv').open() as f:c=list(csv.DictReader(f))[1]
        c={k:float(v) for k,v in c.items()}
        experiment.save_json(folder/'manifest.json',dict(kind='project1_five_profiles_v1',scheme=2,direction='outbound',
            profile=c,hashes={'csv':experiment.digest(folder/'trajectory.csv'),
            'generator':experiment.digest(experiment.GEN),'urdf':experiment.digest(experiment.DEFAULT_URDF)}))
        return folder

    def fake_execute(self,name,late=False,build_late=False):
        folder=self.make_session(name)
        clock=types.SimpleNamespace(value=0.)
        class FakeIO:
            def __init__(self,*args,**kwargs):
                self.q=[0.]*6;self.started=0.;self.status=types.SimpleNamespace(motion_status=0)
                self.publisher=types.SimpleNamespace(topic_name=experiment.COMMAND,
                    get_subscription_count=lambda:1,publish=lambda msg:setattr(self,'q',list(msg.position)))
                self.node=types.SimpleNamespace(get_subscriptions_info_by_topic=lambda _: [
                    types.SimpleNamespace(node_name='agx_arm_ctrl_single_node',node_namespace='/')])
                self.logs=[]
                self.factory=kwargs['buffer_factory'];self.gate_open=False
            def ready(self):pass
            def healthy(self):pass
            def check_graph(self):pass
            def stationary(self):return self.q
            def gate(self,value):self.gate_open=value
            def spin(self,seconds):clock.value+=seconds
            def log(self,path,name,header):
                f=(path/name).open('w',newline='');w=self.factory(header);self.logs.append((f,w));return w
            def stop_and_close(self,stop):return {'gate_close':'closed','stop_requested':stop}
            def close(self):
                for f,w in self.logs:csv.writer(f).writerows(w.export_rows());f.close()
        class FakeCapture:
            def __init__(self,*args,**kwargs):
                kwargs['stdout'].write(b'(100) can0 2A1#0100010000000000\n');kwargs['stdout'].flush();self.returncode=None
            def poll(self):return self.returncode
            def send_signal(self,_):self.returncode=0
            def communicate(self,**_):return None,b''
        def monotonic():
            clock.value+=.0001
            if late and clock.value>4:clock.value+=.03
            return clock.value
        fake_time=types.SimpleNamespace(monotonic=monotonic,time=lambda:1000+clock.value)
        class FakeJointState:
            def __init__(self):
                # 慢构造发生在开始计时前，不应导致运动期间迟到。
                if build_late:clock.value+=.03
        fake_msgs=types.ModuleType('sensor_msgs.msg');fake_msgs.JointState=FakeJointState
        with patch.object(experiment,'RobotIO',FakeIO),patch.object(experiment,'time',fake_time), \
             patch.object(experiment.shutil,'which',return_value='/fake/candump'), \
             patch.object(experiment.subprocess,'Popen',FakeCapture),patch.dict(sys.modules,{'sensor_msgs.msg':fake_msgs}):
            if late:
                with self.assertRaises(RuntimeError):experiment.execute(folder)
            else:experiment.execute(folder)
        path=next(folder.glob('execution_*'))
        result=json.loads((path/'summary.json').read_text())
        self.assertEqual(result['success'],not late)
        self.assertEqual(result['cleanup']['stop_requested'],late)
        if not late:self.assertEqual(result['sent_points'],539)
        self.assertTrue(result['gc_enabled'])
        self.assertEqual(result['lateness_limit_s'],.02)
        self.assertTrue((path/'commands.csv').is_file())

    def test_mock_execution_and_cleanup(self):self.fake_execute('success')
    def test_mock_late_abort(self):self.fake_execute('late',late=True)
    def test_mock_slow_build_before_motion(self):self.fake_execute('build_late',build_late=True)

    def test_numeric_buffer_bounds_and_csv(self):
        b=NumericBuffer(['q','optional','flag'],1)
        b.writerow([.3,'',True])
        self.assertEqual(list(b.export_rows()),[['q','optional','flag'],[.3,'',1.]])
        with self.assertRaises(RuntimeError):b.writerow([1,2,3])
        with self.assertRaises(RuntimeError):NumericBuffer(['q'],1).writerow([1,2])

    def test_gc_disabled_rejected_before_robot_io(self):
        folder=self.make_session('gc_disabled')
        fake_msgs=types.ModuleType('sensor_msgs.msg');fake_msgs.JointState=object
        with patch.dict(sys.modules,{'sensor_msgs.msg':fake_msgs}), \
             patch.object(experiment.gc,'isenabled',return_value=False), \
             patch.object(experiment,'RobotIO') as io:
            with self.assertRaisesRegex(RuntimeError,'GC保持开启'):experiment.execute(folder)
            io.assert_not_called()

if __name__=='__main__':unittest.main()
