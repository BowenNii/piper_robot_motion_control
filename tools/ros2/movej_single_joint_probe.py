#!/usr/bin/env python3
"""单轴或显式六轴MOVE J对照：一次目标/重复固定目标/五次小目标。

单轴模式指定轴在0与0.5rad之间，其他五轴保持prepare实测位置。
六轴模式必须显式--all-joints，目标固定为[.3,.3,-.3,.3,.3,.5]rad。
无自动使能、回零、模式切换或自动串联。check不初始化ROS。
固定目标组没有已知内部参考速度/加速度，不伪造跟随误差。
"""
import argparse
import csv
import gc
import json
import math
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import time

from csv_trajectory_preview import JOINTS, DEFAULT_URDF, read_joint_limits
from real_ik_trajectory import (RobotIO, robot_lock, digest, save_json, difference,
                              COMMAND, START_TOL, FINAL_TOL, LATE_S)
from numeric_log_buffer import NumericBuffer
from real_ik_trajectory import driver_configuration

MODES=('once','repeat','stream')
DT=.02
DURATION=10.76  # 三组相同观察窗口；stream峰值约4.992deg/s，非固定组速度设定。
COUNT=539
AMPLITUDE=.5
MULTI_GOAL=[.3,.3,-.3,.3,.3,.5]


def joint_index(joint):
    if type(joint) is not int or joint not in (1,4,5,6):
        raise ValueError('当前只验证J1、J4、J5、J6，不允许任意关节')
    return joint-1


def motion_indices(joint):
    if joint=='all':return list(range(6))
    return [joint_index(joint)]


def targets(start,direction,mode,joint=1):
    indices=motion_indices(joint)
    if mode not in MODES or direction not in ('outbound','return'):
        raise ValueError('无效模式/方向')
    if len(start)!=6 or not all(math.isfinite(x) for x in start):
        raise ValueError('起点必须为六个有限角度')
    reference=MULTI_GOAL if joint=='all' else [AMPLITUDE if j in indices else 0. for j in range(6)]
    expected=[0.]*6 if direction=='outbound' else reference
    if difference(start,expected)>.005:
        raise ValueError('不在本实验指定起点附近；不会自动移动到起点')
    goal=list(start)
    for index in indices:goal[index]=reference[index] if direction=='outbound' else 0.
    output=[]
    limits=read_joint_limits(DEFAULT_URDF)
    for k in range(COUNT):
        u=k/(COUNT-1);s=10*u**3-15*u**4+6*u**5
        q=[a+(b-a)*s for a,b in zip(start,goal)] if mode=='stream' else list(goal)
        if any(not lo<=x<=hi for x,(lo,hi) in zip(q,limits)):
            raise ValueError('角度超出URDF；不裁剪')
        output.append(q)
    return goal,output


def prepare(folder,mode,direction,joint=1):
    motion_indices(joint)
    folder=Path(folder).resolve()
    if folder.exists() or not folder.parent.is_dir():
        raise ValueError('必须用新目录，父目录先建立')
    io=RobotIO()
    try:
        io.ready();configuration=driver_configuration(io);start=io.stationary()
        goal,qs=targets(start,direction,mode,joint)
        folder.mkdir()
        with (folder/'targets.csv').open('x',newline='') as f:
            w=csv.writer(f);w.writerow(['index','target_time_s','should_publish',*[f'q{i}_rad' for i in range(1,7)]])
            for k,q in enumerate(qs):w.writerow([k,k*DT,int(mode!='once' or k==0),*q])
        kind='movej_multi_joint_probe_v1' if joint=='all' else 'movej_single_joint_probe_v2'
        save_json(folder/'manifest.json',dict(kind=kind,joint=joint,mode=mode,direction=direction,
            q_start_rad=start,q_goal_rad=goal,duration_s=DURATION,samples=COUNT,driver_config=configuration,
            targets_sha256=digest(folder/'targets.csv'),urdf_sha256=digest(DEFAULT_URDF),
            sender_sha256=digest(Path(__file__)),
            note='MOVE J only. once/repeat have unknown internal time law. No collision certification.'))
        if difference(io.stationary(),start)>START_TOL:
            raise RuntimeError('准备期间起点变化；本目录不得执行，请重新prepare')
        save_json(folder/'ready.json',{'prepared':True})
        print('准备完成，未发送目标，门控关闭。',flush=True)
        check(folder)
    finally:
        try:print('清理：',io.stop_and_close(False))
        finally:io.close()


def check(folder):
    folder=Path(folder);m=json.loads((folder/'manifest.json').read_text())
    if m['kind'] not in ('movej_j1_probe_v1','movej_single_joint_probe_v2','movej_multi_joint_probe_v1') or not json.loads((folder/'ready.json').read_text())['prepared']:
        raise ValueError('准备不完整')
    if m['kind']=='movej_j1_probe_v1':m['joint']=1
    indices=motion_indices(m['joint'])
    if (m['kind']=='movej_multi_joint_probe_v1')!=(m['joint']=='all'):
        raise ValueError('单轴/六轴元数据不一致')
    if digest(folder/'targets.csv')!=m['targets_sha256'] or digest(DEFAULT_URDF)!=m['urdf_sha256'] or digest(Path(__file__))!=m['sender_sha256']:
        raise ValueError('代码/模型/目标文件已改变，请重新prepare')
    goal,qs=targets(m['q_start_rad'],m['direction'],m['mode'],m['joint'])
    with (folder/'targets.csv').open() as f:rows=list(csv.DictReader(f))
    if len(rows)!=COUNT or m['duration_s']!=DURATION or m['samples']!=COUNT or difference(goal,m['q_goal_rad'])>1e-12:
        raise ValueError('计划元数据不符')
    for k,(row,q) in enumerate(zip(rows,qs)):
        if int(row['index'])!=k or abs(float(row['target_time_s'])-k*DT)>1e-10 or \
            int(row['should_publish'])!=int(m['mode']!='once' or k==0) or \
            difference([float(row[f'q{i}_rad']) for i in range(1,7)],q)>1e-12:
            raise ValueError('目标复核失败')
    description=f"六轴 {m['q_start_rad']} -> {goal}" if m['joint']=='all' else \
        f"J{m['joint']} {m['q_start_rad'][indices[0]]:.6f} -> {goal[indices[0]]:.6f}rad"
    print(f"{m['mode']} {m['direction']}：{description}；"
          f"发{1 if m['mode']=='once' else COUNT}点，观察窗口{DURATION}s。")
    return m,qs


def guard(io,start,goal,joint=1):
    indices=motion_indices(joint)
    io.healthy()
    for index in indices:
        if io.q[index]<min(start[index],goal[index])-.01 or io.q[index]>max(start[index],goal[index])+.01:
            raise RuntimeError(f'J{index+1}超出本段位置包络')
    if any(abs(io.q[j]-start[j])>.005 for j in range(6) if j not in indices):
        raise RuntimeError('非运动轴变化超过0.005rad')


def execute(folder):
    from sensor_msgs.msg import JointState
    m,qs=check(folder);folder=Path(folder)
    if not gc.isenabled() or not shutil.which('candump'):
        raise RuntimeError('需要开启GC并安装candump')
    messages=[]
    for q in qs:
        msg=JointState();msg.name=JOINTS;msg.position=q;messages.append(msg)
    log=Path(tempfile.mkdtemp(prefix='execution_',dir=folder))
    io=RobotIO(log,command_enabled=True,buffer_factory=NumericBuffer)
    sent=0;success=False;raw=None;capture=None
    result=dict(success=False,joint=m['joint'],mode=m['mode'],direction=m['direction'],
                targets_sha256=m['targets_sha256'],gc_enabled=gc.isenabled(),
                interface=COMMAND,reference_note='once/repeat: no known internal q(t), dq(t), ddq(t)')
    try:
        io.ready()
        if driver_configuration(io)!=m['driver_config']:raise RuntimeError('驱动参数与prepare不同')
        if difference(io.stationary(),m['q_start_rad'])>START_TOL:raise RuntimeError('起点改变，请重新prepare')
        pub=io.publisher;end=time.monotonic()+3
        while pub.get_subscription_count()<1:
            if time.monotonic()>end:raise RuntimeError('没有驱动订阅者')
            io.spin(.01);io.healthy()
        subs=io.node.get_subscriptions_info_by_topic(COMMAND)
        if len(subs)!=1 or subs[0].node_name!='agx_arm_ctrl_single_node' or subs[0].node_namespace!='/':
            raise RuntimeError('必须只有预期真机驱动订阅；先关闭外部录包或其他控制程序')
        commands=io.log(log,'commands.csv',['index','target_time_s','send_elapsed_s','lateness_s',
            'publish_start_elapsed_s','publish_return_epoch_s',*[f'q{i}_rad' for i in range(1,7)]])
        raw=(log/'raw_can.log').open('xb')
        capture=subprocess.Popen(['candump','-L','can0'],stdout=raw,stderr=subprocess.PIPE)
        end=time.monotonic()+3
        while time.monotonic()<end:
            io.spin(.01);guard(io,m['q_start_rad'],m['q_goal_rad'],m['joint'])
            if capture.poll() is not None:raise RuntimeError('CAN采集退出')
        if (log/'raw_can.log').stat().st_size==0:raise RuntimeError('没有CAN反馈，不发送')
        io.check_graph()
        if difference(io.stationary(),m['q_start_rad'])>START_TOL:raise RuntimeError('开门前起点变化')
        io.gate(True);io.healthy()
        start=time.monotonic();result['trajectory_start_receive_elapsed_s']=start-io.started
        result['trajectory_start_epoch_s']=time.time()-(time.monotonic()-start)
        previous=m['q_start_rad']
        print('人工确认范围后执行；Ctrl+C请求软件保持/关门，不是硬件急停。',flush=True)
        for k,msg in enumerate(messages):
            deadline=start+k*DT
            while time.monotonic()<deadline:
                io.spin(min(.005,max(0,deadline-time.monotonic())))
                guard(io,m['q_start_rad'],m['q_goal_rad'],m['joint'])
            guard(io,m['q_start_rad'],m['q_goal_rad'],m['joint'])
            if capture.poll() is not None:raise RuntimeError('CAN采集中断')
            if m['mode']=='stream' and difference(io.q,previous)>.01:
                raise RuntimeError('stream跟随偏差超过0.01rad')
            before=time.monotonic();late=before-deadline
            if late>=LATE_S:
                result['abort_timing']=dict(index=k,lateness_s=late)
                raise RuntimeError('迟到达到20ms，停止而非补发')
            if m['mode']!='once' or k==0:
                before=time.monotonic();late=before-deadline
                if late>=LATE_S:
                    result['abort_timing']=dict(index=k,lateness_s=late,stage='immediately_before_publish')
                    raise RuntimeError('发布前迟到达到20ms，未发布该点')
                pub.publish(msg);elapsed=time.monotonic()-start;sent+=1
                commands.writerow([k,k*DT,elapsed,late,before-start,time.time(),*qs[k]])
                previous=qs[k]
            if k%50==0:print(f'观察进度{k*DT:.2f}s，累计发送{sent}点',flush=True)
        result['command_window_elapsed_s']=time.monotonic()-start
        end=time.monotonic()+3
        while time.monotonic()<end:
            io.spin(.01);guard(io,m['q_start_rad'],m['q_goal_rad'],m['joint'])
            if capture.poll() is not None:raise RuntimeError('保持段CAN采集中断')
        final=io.stationary();error=difference(final,m['q_goal_rad'])
        if error>FINAL_TOL or io.status.motion_status!=0:raise RuntimeError('终点未到位')
        result.update(q_final_rad=final,final_joint_error_max_rad=error);success=True
    except BaseException as exc:
        result['exception']=str(exc);raise
    finally:
        cleanup=io.stop_and_close(sent>0 and not success)
        result.update(sent_points=sent,success=success and 'gate_close_error' not in cleanup,cleanup=cleanup)
        try:
            if capture is not None:
                if capture.poll() is None:capture.send_signal(signal.SIGINT)
                try:_,err=capture.communicate(timeout=3)
                except subprocess.TimeoutExpired:capture.kill();_,err=capture.communicate()
                result['candump_returncode']=capture.returncode;result['candump_stderr']=err.decode(errors='replace')
                if capture.returncode not in (0,-signal.SIGINT):result['success']=False
        finally:
            if raw is not None:raw.close()
            try:io.close()
            except Exception as exc:
                result['success']=False;result['log_flush_error']=str(exc);raise
            finally:save_json(log/'summary.json',result)
        print('记录：',log,flush=True)
        if result['success']:print(f'单程完成，发送{sent}点，终点关节反馈偏差{error:.6g}rad；门控已关闭。')
        elif success:raise RuntimeError('运动完成，但记录或关门控未确认')


def main():
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='action',required=True)
    p=sub.add_parser('prepare');p.add_argument('folder',type=Path);p.add_argument('--mode',choices=MODES,required=True)
    p.add_argument('--direction',choices=['outbound','return'],required=True)
    group=p.add_mutually_exclusive_group(required=True)
    group.add_argument('--joint',type=int,choices=(1,4,5,6))
    group.add_argument('--all-joints',action='store_true',help='显式六轴对照，固定共同目标，不是六轴全0.5')
    p=sub.add_parser('check');p.add_argument('folder',type=Path)
    p=sub.add_parser('execute');p.add_argument('folder',type=Path)
    p.add_argument('--confirm-real-hardware',action='store_true',required=True)
    p.add_argument('--path-clear-and-stop-ready',action='store_true',required=True)
    args=parser.parse_args()
    if args.action=='check':check(args.folder)
    else:
        with robot_lock():
            if args.action=='prepare':prepare(args.folder,args.mode,args.direction,'all' if args.all_joints else args.joint)
            else:execute(args.folder)


if __name__=='__main__':main()
