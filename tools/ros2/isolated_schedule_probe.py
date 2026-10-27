#!/usr/bin/env python3
"""隔离ROS调度测试：两个进程，200Hz历史反馈回放+50Hz无控制作用的目标发布。

固定DDS域93，仅/offline_diagnostics/*话题，无CAN、服务、Action、真机控制。
按历史header相对时间播放，不复制历史receive时刻的32ms停顿。
不能复现USB-CAN或真实驱动负载，测试通过不等于真机发送已修复。
"""
import argparse
import csv
import gc
import json
import math
import os
import resource
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
from csv_trajectory_preview import JOINTS, load_csv
from real_ik_trajectory import RobotIO, RowBuffer
from numeric_log_buffer import NumericBuffer

PREFIX='/offline_diagnostics/'

class ListBuffer:
    def __init__(self,header,capacity):self.header=header;self.capacity=capacity;self.data=[]
    def writerow(self,row):
        if len(self.data)>=self.capacity:raise RuntimeError('诊断缓存超限')
        self.data.append(list(row))
    def column(self,j):return [row[j] for row in self.data]
    def export_rows(self):yield self.header;yield from self.data

def replay(source,duration):
    import rclpy
    from sensor_msgs.msg import JointState
    from agx_arm_msgs.msg import AgxArmStatus
    with source.open() as f:rows=list(csv.DictReader(f))
    if not rows:raise ValueError('历史反馈为空')
    times=[float(r['header_stamp_s'])-float(rows[0]['header_stamp_s']) for r in rows]
    rclpy.init(args=[])
    node=rclpy.create_node('isolated_feedback_replay',use_global_arguments=False)
    pub=node.create_publisher(JointState,PREFIX+'joint_feedback',20)
    status=node.create_publisher(AgxArmStatus,PREFIX+'status',20)
    try:
        end=time.monotonic()+15
        while pub.get_subscription_count()==0 or status.get_subscription_count()==0:
            if time.monotonic()>end:raise RuntimeError('没有隔离订阅者')
            rclpy.spin_once(node,timeout_sec=.05)
        start=time.monotonic();epoch=time.time();i=0;k=0
        # 200Hz新鲜数据时钟，历史不足部分保持最后角度，绝不是实际关节跟随。
        while k*.005<=duration:
            deadline=start+k*.005
            while time.monotonic()<deadline:
                time.sleep(min(.001,max(0.,deadline-time.monotonic())))
            t=k*.005
            while i+1<len(rows) and times[i+1]<=t:i+=1
            row=rows[i];msg=JointState();msg.name=JOINTS
            msg.position=[float(row[f'q{j}_rad']) for j in range(1,7)]
            msg.velocity=[float(row.get(f'dq{j}_reported') or 0) for j in range(1,7)]
            msg.effort=[float(row.get(f'effort{j}_reported') or 0) for j in range(1,7)]
            stamp=round((epoch+t)*1e9);msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(stamp,10**9)
            pub.publish(msg)
            s=AgxArmStatus();s.ctrl_mode=1;s.mode_feedback=1
            s.joint_angle_limit=[False]*6;s.communication_status_joint=[False]*6;status.publish(s)
            rclpy.spin_once(node,timeout_sec=0);k+=1
    finally:node.destroy_node();rclpy.shutdown()

def probe(plan,source,mode='baseline',output_dir=None,seconds=None):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import JointState
    from agx_arm_msgs.msg import AgxArmStatus
    points=load_csv(plan)
    if seconds is not None:points=[p for p in points if p.time_ns/1e9<=seconds]
    duration=points[-1].time_ns/1e9
    if duration>60:raise ValueError('本诊断最多60秒')
    if output_dir is None:output=Path(tempfile.mkdtemp(prefix='piper_schedule_probe_'))
    else:output=Path(output_dir);output.mkdir(exist_ok=False)
    rclpy.init(args=[])
    node=rclpy.create_node('isolated_schedule_probe',use_global_arguments=False)
    # 复用原工具的回调解析/历史窗口/内存日志，但不调用RobotIO构造函数。
    # 因而不会创建任何控制publisher、服务客户端或CAN连接。
    state_header=['receive_elapsed_s','header_stamp_s',*[f'q{j}_rad' for j in range(1,7)],
            *[f'dq{j}_reported' for j in range(1,7)],*[f'effort{j}_reported' for j in range(1,7)]]
    status_header=['receive_elapsed_s','ctrl_mode','arm_status','mode_feedback','motion_status',
            'err_status','any_angle_limit','any_communication_error']
    # baseline/preallocated保留原RowBuffer，只有buffered替换反馈、状态、调度和回调缓存。
    make_buffer=NumericBuffer if mode=='buffered' else ListBuffer
    io=SimpleNamespace(started=time.monotonic(),header=None,header_at=0.,q=None,q_at=0.,history=[],
        state_writer=NumericBuffer(state_header,15000) if mode=='buffered' else RowBuffer(state_header),
        status_writer=NumericBuffer(status_header,15000) if mode=='buffered' else RowBuffer(status_header))
    callback_times=make_buffer(['receive_s','callback_duration_s'],15000);command_receive=[]
    def feedback(msg):
        before=time.monotonic();RobotIO._state(io,msg)
        callback_times.writerow([before-io.started,time.monotonic()-before])
    node.create_subscription(JointState,PREFIX+'joint_feedback',feedback,20)
    node.create_subscription(AgxArmStatus,PREFIX+'status',lambda msg:RobotIO._status(io,msg),20)
    node.create_subscription(JointState,PREFIX+'command_probe',lambda msg:command_receive.append(time.monotonic()),20)
    pub=node.create_publisher(JointState,PREFIX+'command_probe',1)
    assert pub.topic_name==PREFIX+'command_probe'
    messages=None
    if mode!='baseline':
        messages=[]
        for p in points:
            msg=JointState();msg.name=JOINTS;msg.position=p.positions;messages.append(msg)
    executor=SingleThreadedExecutor();executor.add_node(node)
    worker=threading.Thread(target=executor.spin,daemon=True);worker.start()
    log=(output/'replay_process.log').open('w')
    process=subprocess.Popen([sys.executable,__file__,'--worker','--feedback',str(source),
        '--duration',str(duration+3)],stdout=log,stderr=subprocess.STDOUT)
    samples=make_buffer(['index','target_time_s','observed_s','lateness_s','message_build_s','publish_s',
        'process_cpu_s','feedback_age_s','message_build_cpu_s','publish_lateness_s'],len(points))
    memory=[];gc_events=[];gc_started={};gc_active=False
    def gc_event(phase,info):
        key=(threading.get_ident(),info['generation'])
        if phase=='start' and gc_active:gc_started[key]=time.monotonic()
        elif key in gc_started:
            started=gc_started.pop(key)
            gc_events.append([started-io.started,time.monotonic()-started,key[1],key[0]])
    gc.callbacks.append(gc_event)
    result={'simulation_only':True,'domain_id':93,'mode':mode,'duration_s':duration,'plan':str(plan),'feedback_source':str(source),
        'gc_enabled':gc.isenabled(),'memory_note':'Linux ru_maxrss: peak RSS KiB, not current RSS; includes initialization',
        'scope':'ROS and Python scheduling only; no CAN/real driver/firmware, no hardware safety certification'}
    print('隔离ROS域93；不操作真机。输出：',output,flush=True)
    try:
        end=time.monotonic()+15
        while io.q is None:
            if process.poll() is not None or time.monotonic()>end:raise RuntimeError('历史回放未启动，请看replay_process.log')
            time.sleep(.01)
        start=time.monotonic();gc_active=True
        memory.append([0.,resource.getrusage(resource.RUSAGE_SELF).ru_maxrss])
        for k,p in enumerate(points):
            deadline=start+p.time_ns/1e9
            while time.monotonic()<deadline:time.sleep(min(.005,max(0.,deadline-time.monotonic())))
            if not worker.is_alive() or process.poll() is not None:raise RuntimeError('隔离回放或接收线程提前退出')
            before=time.monotonic();cpu=time.process_time();late=before-deadline
            if messages is None:
                msg=JointState();msg.name=JOINTS;msg.position=p.positions
            else:msg=messages[k]
            built=time.monotonic();build_cpu=time.process_time()-cpu
            pub.publish(msg);after=time.monotonic()
            samples.writerow([k,p.time_ns/1e9,before-start,late,built-before,after-built,cpu,after-io.q_at,build_cpu,built-deadline])
            if k%50==0:memory.append([after-start,resource.getrusage(resource.RUSAGE_SELF).ru_maxrss])
            if k%500==0:print(f'诊断进度{p.time_ns/1e9:.1f}/{duration:.2f}s',flush=True)
        gc_active=False
        end_peak_rss=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        memory.append([time.monotonic()-start,end_peak_rss])
        time.sleep(.3)
        late=samples.column(9)
        receives=io.state_writer.column(0) if mode=='buffered' else [float(r[0]) for r in io.state_writer.rows[1:]]
        result.update(completed=True,max_lateness_ms=max(late)*1000,late_at_least_20ms=sum(x>=.02 for x in late),
            feedback_samples=len(receives),max_feedback_receive_gap_ms=max((b-a)*1000 for a,b in zip(receives,receives[1:])),
            max_callback_ms=max(callback_times.column(1))*1000,command_received=len(command_receive),
            max_pre_lateness_ms=max(samples.column(3))*1000,
            max_message_build_ms=max(samples.column(4))*1000,
            start_peak_rss_kib=memory[0][1],end_peak_rss_kib=end_peak_rss)
        result['max_gc_pause_ms']=max((x[1]*1000 for x in gc_events),default=0)
        print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
    except BaseException as exc:
        result.update(completed=False,error=str(exc));raise
    finally:
        gc_active=False;gc.callbacks.remove(gc_event)
        if process.poll() is None:process.terminate()
        try:process.wait(timeout=3)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        log.close();executor.shutdown();worker.join(timeout=3)
        node.destroy_node();rclpy.shutdown()
        for name,rows in [('feedback.csv',io.state_writer.export_rows() if mode=='buffered' else io.state_writer.rows),
            ('status.csv',io.status_writer.export_rows() if mode=='buffered' else io.status_writer.rows),
            ('schedule.csv',samples.export_rows()),
            ('gc_events.csv',[['start_receive_s','duration_s','generation','thread_id'],*gc_events]),
            ('callback_times.csv',callback_times.export_rows()),
            ('memory.csv',[['elapsed_s','peak_rss_kib'],*memory])]:
            with (output/name).open('x',newline='') as f:csv.writer(f).writerows(rows)
        (output/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan',type=Path);p.add_argument('--feedback',type=Path,required=True)
    p.add_argument('--mode',choices=['baseline','preallocated','buffered'],default='baseline')
    p.add_argument('--compare',action='store_true',help='自动顺序运行三种隔离模式')
    p.add_argument('--output-dir',type=Path,help='新的输出目录，不覆盖')
    p.add_argument('--seconds',type=float,help='仅截取开头若干秒作冒烟测试，不作完整周期结论')
    p.add_argument('--worker',action='store_true',help=argparse.SUPPRESS)
    p.add_argument('--duration',type=float,default=60,help=argparse.SUPPRESS)
    a=p.parse_args();os.environ['ROS_DOMAIN_ID']='93'
    os.environ.setdefault('RMW_FASTRTPS_PUBLICATION_MODE','ASYNCHRONOUS')
    if a.seconds is not None and (not math.isfinite(a.seconds) or a.seconds<=0):p.error('--seconds必须为正数')
    if a.worker:replay(a.feedback,a.duration)
    elif a.plan and a.compare:
        root=a.output_dir or Path(tempfile.mkdtemp(prefix='piper_schedule_comparison_'))
        if a.output_dir:root.mkdir(exist_ok=False)
        summaries=[]
        for mode in ['baseline','preallocated','buffered']:
            args=[sys.executable,__file__,'--plan',str(a.plan),'--feedback',str(a.feedback),'--mode',mode,
                '--output-dir',str(root/mode)]
            if a.seconds is not None:args+=['--seconds',str(a.seconds)]
            subprocess.run(args,check=True)
            summaries.append(json.loads((root/mode/'summary.json').read_text()))
        fields=['mode','duration_s','max_lateness_ms','late_at_least_20ms','max_gc_pause_ms',
            'max_feedback_receive_gap_ms','max_message_build_ms','start_peak_rss_kib','end_peak_rss_kib']
        with (root/'comparison.csv').open('x',newline='') as f:
            w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(summaries)
        print('三组隔离测试完成；不是硬件安全认证。汇总：',root/'comparison.csv',flush=True)
    elif a.plan:probe(a.plan,a.feedback,a.mode,a.output_dir,a.seconds)
    else:p.error('需要--plan')
