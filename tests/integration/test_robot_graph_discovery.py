"""DDS发现等待的假节点测试；不初始化ROS、不调用任何服务。"""
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools/ros2'))
import real_ik_trajectory as module


class GraphTest(unittest.TestCase):
    def make(self,kind):
        clock=types.SimpleNamespace(value=0.)
        def endpoint(name='agx_arm_ctrl_single_node',ns='/'):
            return types.SimpleNamespace(node_name=name,node_namespace=ns)
        def names():
            if kind=='duplicate':return [('agx_arm_ctrl_single_node','/')]*2
            if kind=='wrong_ns':return [('agx_arm_ctrl_single_node','/other')]
            if kind=='absent' or (kind=='delayed' and clock.value<.2):return []
            return [('agx_arm_ctrl_single_node','/')]
        def publishers(topic):
            if topic==module.FEEDBACK:
                if kind=='wrong_feedback':return [endpoint('other')]
                if kind=='duplicate_feedback':return [endpoint(),endpoint()]
                if kind=='delayed' and clock.value<.3:return [endpoint('_NODE_NAME_UNKNOWN_','_NODE_NAMESPACE_UNKNOWN_')]
                return [endpoint()]
            if kind=='other_control':return [endpoint('other_sender')]
            return []
        io=module.RobotIO.__new__(module.RobotIO)
        io.interrupted=kind=='cancel'
        io.node=types.SimpleNamespace(get_node_names_and_namespaces=names,
            get_publishers_info_by_topic=publishers,get_name=lambda:'real_ik_csv_tool')
        io.spin=lambda seconds:setattr(clock,'value',clock.value+seconds)
        return io,clock

    def run_wait(self,kind):
        io,clock=self.make(kind)
        with patch.object(module,'time',types.SimpleNamespace(monotonic=lambda:clock.value)):
            io.wait_for_graph()
        return clock.value

    def test_delayed_graph_eventually_stable(self):
        self.assertGreaterEqual(self.run_wait('delayed'),.6)

    def test_stable_graph_still_observed(self):
        self.assertGreaterEqual(self.run_wait('ready'),.3)

    def test_missing_graph_times_out(self):
        with self.assertRaisesRegex(RuntimeError,'3秒内ROS图未稳定'):self.run_wait('absent')

    def test_conflicts_never_ignored(self):
        for kind in ('duplicate','wrong_ns','wrong_feedback','duplicate_feedback','other_control'):
            with self.subTest(kind=kind):
                with self.assertRaises(RuntimeError):self.run_wait(kind)

    def test_cancel(self):
        with self.assertRaisesRegex(RuntimeError,'用户取消'):self.run_wait('cancel')

    def test_strict_check_does_not_wait(self):
        io,_=self.make('absent')
        with self.assertRaises(module.GraphNotReady):io.check_graph()


if __name__=='__main__':unittest.main()
