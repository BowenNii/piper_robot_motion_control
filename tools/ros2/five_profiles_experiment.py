#!/usr/bin/env python3
"""五组实验分段工具：prepare关门读起点，check离线复核，execute人工确认后单程。

绝不自动使能、回零、切模式或循环往返。正常门控关闭不等于急停。
MOVE J可能在固件内重新规划，记录结果不代表硬实时或碰撞安全认证。
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
from csv_trajectory_preview import REPO, DEFAULT_URDF, JOINTS, load_csv
from numeric_log_buffer import NumericBuffer
from real_ik_trajectory import (RobotIO, robot_lock, digest, save_json, difference,
                              COMMAND, START_TOL, TRACK_TOL, FINAL_TOL, LATE_S)

ROOT=REPO/'experiments/project1/2026-10-25_five_profiles'
GEN=REPO/'build/debug/joint_profile_comparison'
DECODER=REPO/'build/debug/project1_feedback_export'
GOAL=[.3,.3,-.3,.3,.3,.5]

def driver_configuration(io):
    """只读参数；记录参考系相关设置，防止把TCP偏移当成模型误差。"""
    from rcl_interfaces.srv import GetParameters
    from real_ik_trajectory import DRIVER
    names=['arm_type','can_port','fast_mode','speed_percent','auto_enable','effector_type','tcp_offset','pub_rate']
    response=io.service(DRIVER+'/get_parameters',GetParameters,GetParameters.Request(names=names))
    values={}
    for name,v in zip(names,response.values):
        if v.type==1:values[name]=v.bool_value
        elif v.type==2:values[name]=v.integer_value
        elif v.type==3:values[name]=v.double_value
        elif v.type==4:values[name]=v.string_value
        elif v.type==8:values[name]=list(v.double_array_value)
        else:values[name]=None
    offset=values.get('tcp_offset')
    if not isinstance(offset,list) or len(offset)!=6 or any(not math.isfinite(x) or abs(x)>1e-12 for x in offset):
        raise RuntimeError('本次模型对比要求已确认的零TCP偏移；不自动修改驱动参数')
    return values

def profile_state(t,c):
    """独立Python复核C++解析时间规律；x归一化到总位移1。"""
    T=c['motion_duration_s'];scheme=int(c['scheme'])
    if t>=T:return 1.,0.,0.
    if scheme<=3:
        u=t/T
        return 10*u**3-15*u**4+6*u**5,30*u*u*(1-u)**2/T,60*u*(1-u)*(1-2*u)/T**2
    ta=c['accel_time_s'];tc=c['cruise_time_s'];vn=1/(ta+tc)
    def accel(z):
        u=z/ta
        if scheme==5:return .5*vn*z*z/ta,vn*u,vn/ta
        return (vn*ta*(2.5*u**4-3*u**5+u**6),vn*(10*u**3-15*u**4+6*u**5),
                vn/ta*30*u*u*(1-u)**2)
    if t<ta:return accel(t)
    if t<ta+tc:return .5*vn*ta+vn*(t-ta),vn,0.
    x,v,a=accel(T-t);return 1-x,v,-a

def validate(points,c):
    if not 0<c['grid_duration_s']<=60:raise ValueError('时长必须在(0,60]秒')
    if len(points)!=round(c['grid_duration_s']/.02)+1:raise ValueError('采样点数不符')
    q0=points[0].positions;delta=[b-a for a,b in zip(q0,points[-1].positions)]
    if max(map(abs,delta))>.601:raise ValueError('运动幅度超过本实验范围')
    for k,p in enumerate(points):
        if p.time_ns!=k*20_000_000:raise ValueError('必须20ms等间隔采样')
        s,v,a=profile_state(p.time_ns/1e9,c)
        for actual,expected in ((p.positions,[q0[j]+delta[j]*s for j in range(6)]),
                (p.velocities,[d*v for d in delta]),(p.accelerations,[d*a for d in delta])):
            if difference(actual,expected)>1e-9:raise ValueError('解析曲线复核失败')
        if max(map(abs,p.velocities))>math.radians(8)+1e-9 or max(map(abs,p.accelerations))>.07:
            raise ValueError('超出本实验速度/加速度上限，不是厂家安全限值')
        if k and difference(p.positions,points[k-1].positions)>.003:raise ValueError('相邻角度跳变')
    if max(map(abs,points[-1].velocities))>1e-10:raise ValueError('终点速度非零')

def check(folder):
    folder=Path(folder);m=json.loads((folder/'manifest.json').read_text())
    if m['kind']!='project1_five_profiles_v1':raise ValueError('不是本工具准备的实验')
    for key,path in [('csv',folder/'trajectory.csv'),('generator',GEN),('urdf',DEFAULT_URDF)]:
        if digest(path)!=m['hashes'][key]:raise ValueError(f'{key}已改变，请重新prepare')
    points=load_csv(folder/'trajectory.csv');validate(points,m['profile'])
    expected=GOAL if m['direction']=='outbound' else [0.]*6
    if difference(points[-1].positions,expected)>1e-9:raise ValueError('目标不是本实验指定目标')
    print(f"方案{m['scheme']} {m['direction']}：{len(points)}点，{points[-1].time_ns/1e9:.2f}秒；离线复核通过。")
    return m,points

def prepare(folder,scheme,direction):
    folder=Path(folder).resolve()
    if folder.exists() or not folder.parent.is_dir():raise ValueError('需新目录，父目录先建立')
    io=RobotIO()
    try:
        io.ready();configuration=driver_configuration(io);start=io.stationary()
        expected=[0.]*6 if direction=='outbound' else GOAL
        if difference(start,expected)>.005:raise RuntimeError('不在指定段起点附近，不会自动移动到起点')
        goal=GOAL if direction=='outbound' else [0.]*6
        folder.mkdir()
        args=[str(GEN),str(folder/'generated'),'--start',*map(str,start),'--goal',*map(str,goal)]
        with (folder/'generator.log').open('x') as f:
            p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT)
            try:
                end=time.monotonic()+60
                while p.poll() is None:
                    io.spin(.01);io.healthy()
                    if time.monotonic()>end:raise RuntimeError('生成器超时')
                if p.returncode:raise RuntimeError('C++生成失败，请查看generator.log')
            finally:
                if p.poll() is None:p.terminate();p.wait(timeout=3)
        if difference(io.stationary(),start)>START_TOL:raise RuntimeError('生成期间起点变化')
        # 生成器的outbound表示指定start->goal，因此回程也选此文件。
        shutil.copyfile(folder/'generated'/f'scheme{scheme}_outbound.csv',folder/'trajectory.csv')
        with (folder/'generated'/'summary.csv').open() as f:
            c=next({k:float(v) for k,v in row.items()} for row in csv.DictReader(f) if int(row['scheme'])==scheme)
        validate(load_csv(folder/'trajectory.csv'),c)
        save_json(folder/'manifest.json',dict(kind='project1_five_profiles_v1',scheme=scheme,direction=direction,
            q_start_rad=start,q_goal_rad=goal,profile=c,generator_arguments=args,driver_config=configuration,
            hashes={'csv':digest(folder/'trajectory.csv'),'generator':digest(GEN),'urdf':digest(DEFAULT_URDF)},
            note='Measured start, MOVE J interface, no collision/dynamics certification; not automatic return'))
        (folder/'observations.md').write_text(
            '# 人工实验记录\n\n- 视频文件：\n- 负载/夹爪/安装情况：\n- 实际执行驱动/固件版本：\n'
            '- 运动中观察（时间点、关节、抖动程度）：\n- 到位后观察：\n- 是否人工停止及原因：\n',encoding='utf-8')
        print('准备完成，没有发送运动目标；门控关闭。');check(folder)
    finally:
        try:print('清理：',io.stop_and_close(False))
        finally:io.close()

def execute(folder):
    """一段轨迹，运动前后各3秒原始CAN；受监督执行，保持旧工具保护线。"""
    from sensor_msgs.msg import JointState
    m,points=check(folder);folder=Path(folder)
    if not gc.isenabled():raise RuntimeError('本版本要求GC保持开启')
    if not shutil.which('candump'):raise RuntimeError('缺少candump，禁止无原始CAN记录运行')
    # 所有消息在订阅线程启动、门控打开之前创建；关键循环只索引，不重新构造。
    messages=[]
    for point in points:
        msg=JointState();msg.name=JOINTS;msg.position=point.positions
        messages.append(msg)
    log=Path(tempfile.mkdtemp(prefix='execution_',dir=folder))
    io=RobotIO(log,command_enabled=True,buffer_factory=NumericBuffer);capture=None;raw=None;sent=0;success=False
    result={'success':False,'scheme':m['scheme'],'direction':m['direction'],'csv_sha256':m['hashes']['csv'],
            'execution_implementation':'preallocated_numeric_buffers_v1','gc_enabled':gc.isenabled(),
            'log_capacity_rows':20000,'lateness_limit_s':LATE_S,
            'clock_note':'CAN epoch clock; commands log monotonic + explicit wall-clock origin; no arbitrary alignment shift'}
    try:
        io.ready()
        if 'driver_config' in m:
            result['driver_config']=driver_configuration(io)
            if result['driver_config']!=m['driver_config']:raise RuntimeError('驱动参数与prepare时不一致')
        if difference(io.stationary(),points[0].positions)>START_TOL:raise RuntimeError('起点偏差超过0.001rad，请重新prepare')
        pub=io.publisher
        deadline=time.monotonic()+3
        while pub.get_subscription_count()!=1:
            if time.monotonic()>deadline:raise RuntimeError('必须且只能有一个真机驱动订阅者')
            io.spin(.01);io.healthy()
        subs=io.node.get_subscriptions_info_by_topic(COMMAND)
        if subs[0].node_name!='agx_arm_ctrl_single_node' or subs[0].node_namespace!='/':
            raise RuntimeError('不是预期驱动订阅者')
        commands=io.log(log,'commands.csv',['index','target_time_s','send_elapsed_s','lateness_s',
            'publish_start_elapsed_s','publish_return_epoch_s',*[f'q{j}_rad' for j in range(1,7)]])
        raw=(log/'raw_can.log').open('xb')
        capture=subprocess.Popen(['candump','-L','can0'],stdout=raw,stderr=subprocess.PIPE)
        end=time.monotonic()+3
        while time.monotonic()<end:
            io.spin(.01);io.healthy()
            if capture.poll() is not None:raise RuntimeError('candump已退出')
        if (log/'raw_can.log').stat().st_size==0:raise RuntimeError('未收到原始CAN，不发送目标')
        io.check_graph();io.stationary()
        if difference(io.q,points[0].positions)>START_TOL:raise RuntimeError('开门前起点改变')
        io.gate(True);io.healthy()
        start=time.monotonic();result['trajectory_start_receive_elapsed_s']=start-io.started
        result['trajectory_start_epoch_s']=time.time()-(time.monotonic()-start)
        target=points[0].positions
        for k,point in enumerate(points):
            loop_started=time.monotonic()
            deadline=start+point.time_ns/1e9
            while time.monotonic()<deadline:
                io.spin(min(.005,max(0.,deadline-time.monotonic())));io.healthy()
                if difference(io.q,target)>TRACK_TOL:raise RuntimeError('关节跟随偏差超过0.01rad')
            after_wait=time.monotonic()
            io.healthy()
            if capture.poll() is not None:raise RuntimeError('原始CAN采集中断')
            if difference(io.q,target)>TRACK_TOL:raise RuntimeError('关节跟随偏差超过0.01rad')
            before=time.monotonic();late=before-deadline
            if late>=LATE_S:
                # 触发点尚未发布，不写进commands；在summary保留准确证据。
                result['abort_timing']={'index':k,'target_time_s':point.time_ns/1e9,
                    'observed_elapsed_s':before-start,'lateness_s':late,
                    'after_wait_elapsed_s':after_wait-start,
                    'post_wait_checks_s':before-after_wait,'loop_elapsed_s':before-loop_started}
                raise RuntimeError('迟到达到20ms，停止而非追赶补发')
            msg=messages[k]
            # 即使只是索引，也可能被操作系统暂停；保留发布前的第二道检查。
            before_publish=time.monotonic()
            if before_publish-deadline>=LATE_S:
                result['abort_timing']={'index':k,'target_time_s':point.time_ns/1e9,
                    'observed_elapsed_s':before_publish-start,'lateness_s':before_publish-deadline,
                    'stage':'after_message_lookup_before_publish','message_lookup_s':before_publish-before,
                    'after_wait_elapsed_s':after_wait-start,'post_wait_checks_s':before-after_wait}
                raise RuntimeError('发布前迟到达到20ms，未发布该点；停止而非追赶补发')
            pub.publish(msg);elapsed=time.monotonic()-start
            sent+=1;target=point.positions
            commands.writerow([k,point.time_ns/1e9,elapsed,late,before-start,time.time(),*target])
            if k%50==0:print(f'已发{sent}/{len(points)}，计划时间{point.time_ns/1e9:.2f}s',flush=True)
        result['command_elapsed_s']=time.monotonic()-start
        end=time.monotonic()+3
        while time.monotonic()<end:
            io.spin(.01);io.healthy()
            if difference(io.q,target)>TRACK_TOL:raise RuntimeError('保持段超差')
        final=io.stationary();error=difference(final,target)
        if error>FINAL_TOL or io.status.motion_status!=0:raise RuntimeError('终点未到位')
        result.update(q_final_rad=final,final_joint_error_max_rad=error);success=True
        print(f'单程完成：{sent}点；终点最大关节偏差{error:.6g}rad，不是末端实测精度。')
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
                result['candump_returncode']=capture.returncode
                result['candump_stderr']=err.decode(errors='replace')
                if capture.returncode not in (0,-signal.SIGINT):result['success']=False
        finally:
            if raw is not None:raw.close()
            try:io.close()
            except Exception as exc:
                result['success']=False;result['log_flush_error']=str(exc);raise
            finally:save_json(log/'summary.json',result)
        print('记录：',log,flush=True)
        if success and not result['success']:raise RuntimeError('运动结束，但记录或门控清理未确认成功')

def analyze(execution):
    execution=Path(execution).resolve()
    subprocess.run([str(DECODER),str(execution/'raw_can.log'),str(execution/'decoded')],check=True)
    print('CAN解码完成。在MATLAB调用：')
    print(f"analyze_project1_complete('{execution}')")

def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='mode',required=True)
    q=sub.add_parser('prepare');q.add_argument('folder',type=Path);q.add_argument('--scheme',type=int,choices=range(1,6),required=True)
    q.add_argument('--direction',choices=['outbound','return'],required=True)
    q=sub.add_parser('check');q.add_argument('folder',type=Path)
    q=sub.add_parser('execute');q.add_argument('folder',type=Path)
    q.add_argument('--confirm-real-hardware',action='store_true',required=True)
    q.add_argument('--path-clear-and-stop-ready',action='store_true',required=True)
    q=sub.add_parser('analyze');q.add_argument('execution',type=Path)
    a=p.parse_args()
    if a.mode=='check':check(a.folder)
    elif a.mode=='analyze':analyze(a.execution)
    else:
        with robot_lock():
            if a.mode=='prepare':prepare(a.folder,a.scheme,a.direction)
            else:execute(a.folder)

if __name__=='__main__':main()
