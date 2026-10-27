#!/usr/bin/env python3
"""五组CSV的隔离RViz运动学回放；无控制话题、服务或Action客户端。

默认全部去程+回程，真实时间1倍速、每段停留3秒，仅播放一次。
ROS_DOMAIN_ID=84用于隔离本演示；不是动力学仿真或碰撞检查。
"""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile
import time
from csv_trajectory_preview import load_csv, DEFAULT_URDF, STATE_TOPIC, make_state_message as legacy_state_message

ROOT=Path(__file__).resolve().parents[2]
DATA=ROOT/'experiments/project1/2026-10-25_five_profiles'
if (DATA/'reference_complete'/'OFFLINE_ONLY.txt').is_file():DATA=DATA/'reference_complete'

def make_state_message(point):
    # 仓库URDF只有六轴，没有旧MoveIt夹爪变量，不能添加未知的gripper。
    msg=legacy_state_message(point)
    msg.state.joint_state.name=msg.state.joint_state.name[:6]
    msg.state.joint_state.position=msg.state.joint_state.position[:6]
    msg.state.joint_state.velocity=msg.state.joint_state.velocity[:6]
    return msg

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scheme',type=int,choices=range(1,6))
    parser.add_argument('--check-only',action='store_true')
    args=parser.parse_args()
    assert (DATA/'OFFLINE_ONLY.txt').is_file(), '离线生成器尚未完成'
    runs=[]
    for scheme in ([args.scheme] if args.scheme else range(1,6)):
        for direction in ('outbound','return'):
            points=load_csv(DATA/f'scheme{scheme}_{direction}.csv')
            runs.append((scheme,direction,points))
    for scheme,direction,points in runs:
        print(f'方案{scheme} {direction}: {len(points)}点，{points[-1].time_ns/1e9:.2f}s',flush=True)
    if args.check_only:
        print('仅检查成功，没有启动ROS或RViz。');return
    # 此进程和它启动的RViz进入独立DDS域，不接入真机所在默认域。
    os.environ['ROS_DOMAIN_ID']='84'
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from moveit_msgs.msg import DisplayRobotState
    rclpy.init(args=[])
    node=rclpy.create_node('five_profiles_offline_preview',use_global_arguments=False)
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub=node.create_publisher(DisplayRobotState,STATE_TOPIC,qos)
    assert pub.topic_name==STATE_TOPIC
    import yaml
    params=tempfile.NamedTemporaryFile(mode='w',suffix='.yaml',prefix='piper_display_',delete=False)
    with params:
        # 只显示刚体模型，无规划组；最小合法SRDF避免插件等待不存在的语义话题。
        yaml.safe_dump({'/**':{'ros__parameters':{
            'robot_description':DEFAULT_URDF.read_text(),
            'robot_description_semantic':'<robot name="piper"></robot>'}}},params)
    rviz=subprocess.Popen(['ros2','run','rviz2','rviz2','-d',str(Path(__file__).with_name('five_profiles.rviz')),
        '--ros-args','--params-file',params.name])
    def hold(point,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            pub.publish(make_state_message(point));rclpy.spin_once(node,timeout_sec=.05)
            if rviz.poll() is not None:raise RuntimeError('RViz已退出')
    try:
        end=time.monotonic()+45
        while pub.get_subscription_count()==0:
            if rviz.poll() is not None or time.monotonic()>end:
                raise RuntimeError('RViz RobotState未订阅显示话题')
            rclpy.spin_once(node,timeout_sec=.1)
        print('隔离域84，1倍速显示，不发送机械臂控制指令。',flush=True)
        for scheme,direction,points in runs:
            print(f'开始方案{scheme} {direction}，{points[-1].time_ns/1e9:.2f}秒；先停留3秒',flush=True)
            hold(points[0],3)
            start=time.monotonic()
            for point in points:
                while time.monotonic()<start+point.time_ns/1e9:
                    rclpy.spin_once(node,timeout_sec=min(.01,max(0,start+point.time_ns/1e9-time.monotonic())))
                if rviz.poll() is not None:raise RuntimeError('RViz已退出')
                pub.publish(make_state_message(point))
            print(f'完成方案{scheme} {direction}，实耗{time.monotonic()-start:.3f}s',flush=True)
            hold(points[-1],3)
        print('五组回放完成。保留RViz和零位模型；Ctrl+C结束预览。',flush=True)
        while rviz.poll() is None:rclpy.spin_once(node,timeout_sec=.2)
    except KeyboardInterrupt:
        print('停止显示回放；没有控制真机。',flush=True)
    finally:
        node.destroy_node();rclpy.shutdown()
        # 不主动关闭RViz窗口，让用户继续查看最终模型。

if __name__=='__main__':main()
