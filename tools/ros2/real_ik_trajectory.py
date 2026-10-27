#!/usr/bin/env python3
"""小幅真机IK跟随实验（MOVE J，不是硬实时轨迹跟踪）。

prepare：关门控、读取新鲜静止关节反馈、调用C++生成CSV；不发送运动目标。
check：离线检查已准备实验；不初始化ROS。
execute：必须额外--confirm-real-hardware；检查后开门、按CSV时间发MOVE J、记录、关门。
不自动使能/回零/切换MIT，不控制夹爪。没有碰撞检查、硬件看门狗或急停保障。
prepare固定10秒、501点、位移≤2mm；prepare-endpoint仅支持本次指定目标，
近零起点、60秒3001点，终点IK后走关节五次轨迹（不保证末端直线）。
"""
import argparse
import csv
from contextlib import contextmanager
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time

from csv_trajectory_preview import REPO, DEFAULT_URDF, JOINTS, load_csv
from csv_action_sender import ordered_positions

FEEDBACK = "/feedback/joint_states"
STATUS = "/feedback/arm_status"
COMMAND = "/control/move_j"
DRIVER = "/agx_arm_ctrl_single_node"
DURATION = 10.0
EXCURSION = 0.05
ENDPOINT_REFERENCE=[.3,.3,-.3,.3,.3,.5]
START_TOL = 0.001
TRACK_TOL = 0.01  # 实验自设保护线，不是厂家跟踪精度。
FINAL_TOL = 0.005
STALE_S = 0.25
LATE_S = 0.02  # 一整个采样周期；跨周期不再集中追赶发送。
MOTION_TOPICS = ["/control/"+name for name in
    ("joint_states", "move_j", "move_js", "move_mit", "move_p", "move_l", "move_c", "move_cpv")]


class RowBuffer:
    """短实验有界内存日志；运动期间不让磁盘写入阻塞反馈回调或发送调度。"""
    def __init__(self, header): self.rows=[list(header)]
    def writerow(self,row):
        if len(self.rows)>=20000: raise RuntimeError("实验日志缓存超限")
        self.rows.append(list(row))


@contextmanager
def robot_lock():
    """同一台机器仅允许一个本工具prepare/execute，退出自动释放锁。"""
    fd=os.open("/tmp/piper_real_ik_trajectory.lock",os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW,0o600)
    try:
        try: fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError as exc: raise RuntimeError("另一个真机轨迹工具正在运行，请勿并行操作") from exc
        yield
    finally: os.close(fd)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save_json(path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False, allow_nan=False)


def difference(a, b):
    return max(abs(x-y) for x,y in zip(a,b))


def check_points(points,endpoint=False):
    """独立复核时间、相邻步幅、总幅度及q的差分导数；不信任CSV导数列。"""
    duration=60 if endpoint else 10
    if len(points)!=duration*50+1 or points[-1].time_ns!=duration*1_000_000_000:
        raise ValueError(f"本模式只允许{duration*50+1}点、{duration}秒")
    dt=0.02
    q=[p.positions for p in points]
    for k,p in enumerate(points):
        if p.time_ns!=k*20_000_000:
            raise ValueError("CSV必须为均匀20ms采样")
        if difference(p.positions,q[0])>(.6 if endpoint else EXCURSION):
            raise ValueError("关节总变化超过当前实验幅度限制")
        if endpoint:
            u=k/(len(points)-1);s=10*u**3-15*u**4+6*u**5
            expected=[a+s*(b-a) for a,b in zip(q[0],q[-1])]
            if difference(p.positions,expected)>1e-12:
                raise ValueError("终点IK模式必须采用关节空间五次轨迹")
        if k and difference(p.positions,q[k-1])>0.002:
            raise ValueError("相邻目标变化超过0.002rad")
    velocity=[]; acceleration=[]
    for k in range(len(q)):
        v=[]; a=[]
        for j in range(6):
            if k==0:
                vi=(-11*q[0][j]+18*q[1][j]-9*q[2][j]+2*q[3][j])/(6*dt)
                ai=(2*q[0][j]-5*q[1][j]+4*q[2][j]-q[3][j])/dt**2
            elif k==len(q)-1:
                vi=(11*q[-1][j]-18*q[-2][j]+9*q[-3][j]-2*q[-4][j])/(6*dt)
                ai=(2*q[-1][j]-5*q[-2][j]+4*q[-3][j]-q[-4][j])/dt**2
            else:
                vi=(q[k+1][j]-q[k-1][j])/(2*dt)
                ai=(q[k+1][j]-2*q[k][j]+q[k-1][j])/dt**2
            v.append(vi); a.append(ai)
        if difference(v,points[k].velocities)>1e-8 or difference(a,points[k].accelerations)>1e-8:
            raise ValueError("CSV导数与关节角差分不一致")
        velocity+=v; acceleration+=a
    vmax=max(map(abs,velocity)); amax=max(map(abs,acceleration))
    if endpoint and (max(map(abs,q[0]))>.002 or difference(q[-1],ENDPOINT_REFERENCE)>.001):
        raise ValueError("本次终点IK实验只允许近零起点及指定目标附近的逆解")
    if vmax>0.02 or amax>0.05:
        raise ValueError("轨迹超过自设离线速度0.02rad/s或加速度0.05rad/s²")
    if any(max(map(abs,p.velocities))>1e-6 or max(map(abs,p.accelerations))>1e-4
           for p in (points[0],points[-1])):
        raise ValueError("轨迹端点不满足近零静止边界")
    return {"velocity_peak_rad_s":vmax,"acceleration_peak_rad_s2":amax,
            "max_excursion_rad":max(difference(row,q[0]) for row in q)}


def load_session(folder):
    folder=Path(folder)
    meta=json.loads((folder/"manifest.json").read_text(encoding="utf-8"))
    endpoint=meta.get("kind")=="real_move_j_endpoint_ik_v1"
    if meta.get("kind") not in ("real_move_j_relative_ik_v1","real_move_j_endpoint_ik_v1") or \
            meta.get("duration_s")!=(60 if endpoint else DURATION):
        raise ValueError("不是本工具准备的实验")
    for name, expected in meta["hashes"].items():
        path=folder/"trajectory.csv" if name=="csv" else Path(meta[name])
        if digest(path)!=expected:
            raise ValueError(f"{name}文件发生变化，请重新prepare")
    if endpoint:
        if meta.get("q_reference_rad")!=ENDPOINT_REFERENCE:raise ValueError("本次终点参考角不一致")
    else:
        offset=meta["offset_m"]
        if len(offset)!=3 or not all(math.isfinite(x) for x in offset) or not 0<math.hypot(*offset)<=0.002000000001:
            raise ValueError("位移必须大于0且不超过2mm")
    points=load_csv(folder/"trajectory.csv")
    metrics=check_points(points,endpoint)
    if difference(points[0].positions,meta["q_start_rad"])>1e-12:
        raise ValueError("CSV起点与清单不一致")
    return meta,points,metrics


class RobotIO:
    """固定根命名空间接口；构造函数只订阅，不自动使能或发目标。"""
    def __init__(self, log_dir=None, command_enabled=False, buffer_factory=RowBuffer):
        self.buffer_factory=buffer_factory
        import rclpy
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
        from sensor_msgs.msg import JointState
        from agx_arm_msgs.msg import AgxArmStatus
        from std_srvs.srv import SetBool, Empty
        from rcl_interfaces.srv import GetParameters
        self.ros=rclpy
        os.environ.setdefault("RMW_FASTRTPS_PUBLICATION_MODE","ASYNCHRONOUS")
        rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
        self.node=rclpy.create_node("real_ik_csv_tool",use_global_arguments=False)
        self.executor=SingleThreadedExecutor(context=self.node.context)
        self.executor.add_node(self.node)
        self.interrupted=False
        self.old_handlers={s:signal.signal(s,self._signal) for s in (signal.SIGINT,signal.SIGTERM)}
        self.q=None; self.q_at=0.; self.header=None; self.header_at=0.
        self.status=None; self.status_at=0.; self.history=[]; self.files=[]
        self.clients={}
        self.started=time.monotonic(); self.publisher=None
        self.state_writer=None; self.status_writer=None
        if log_dir:
            self.state_writer=self.log(log_dir,"feedback.csv",["receive_elapsed_s","header_stamp_s",
                *[f"q{i}_rad" for i in range(1,7)],*[f"dq{i}_reported" for i in range(1,7)],
                *[f"effort{i}_reported" for i in range(1,7)]])
            self.status_writer=self.log(log_dir,"status.csv",["receive_elapsed_s","ctrl_mode","arm_status",
                "mode_feedback","motion_status","err_status","any_angle_limit","any_communication_error"])
        self.receive_group=MutuallyExclusiveCallbackGroup()
        feedback_qos=QoSProfile(depth=20,reliability=ReliabilityPolicy.RELIABLE)
        sub=self.node.create_subscription(JointState,FEEDBACK,self._state,feedback_qos,
                                          callback_group=self.receive_group)
        stat=self.node.create_subscription(AgxArmStatus,STATUS,self._status,feedback_qos,
                                           callback_group=self.receive_group)
        if sub.topic_name!=FEEDBACK or stat.topic_name!=STATUS:
            raise RuntimeError("不允许重映射真机接口")
        self.clients={name:self.node.create_client(kind,name) for name,kind in
            (("/control_enable",SetBool),("/emergency_stop",Empty),(DRIVER+"/get_parameters",GetParameters))}
        if command_enabled:
            self.publisher=self.node.create_publisher(JointState,COMMAND,1)
            if self.publisher.topic_name!=COMMAND: raise RuntimeError("不允许重映射控制话题")
        # 接收回调独立运行，不与定时发送/服务等待争用主线程。
        self.executor_error=None
        self.thread=threading.Thread(target=self._receive,daemon=True)
        self.thread.start()

    def _receive(self):
        try: self.executor.spin()
        except Exception as exc: self.executor_error=str(exc)

    def _signal(self,*_):
        self.interrupted=True

    def spin(self, seconds=0.01):
        time.sleep(max(0.,seconds))

    def log(self,folder,name,header):
        stream=(Path(folder)/name).open("x",newline="",encoding="utf-8")
        buffer=self.buffer_factory(header);self.files.append((stream,buffer))
        return buffer

    def _state(self,msg):
        try:
            q=ordered_positions(msg.name,msg.position)
            stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            if stamp<=0 or (self.header is not None and stamp<self.header):
                return
        except (ValueError,KeyError):
            return
        now=time.monotonic()
        if stamp!=self.header:
            self.header=stamp; self.header_at=now
        self.q=q; self.q_at=now; self.history.append((now,q))
        self.history=[row for row in self.history if now-row[0]<2.5]
        if self.state_writer:
            def optional(values):
                d=dict(zip(msg.name,values))
                return [d.get(j,"") if math.isfinite(d.get(j,float("nan"))) else "" for j in JOINTS]
            self.state_writer.writerow([now-self.started,stamp/1e9,*q,
                                       *optional(msg.velocity),*optional(msg.effort)])

    def _status(self,msg):
        self.status=msg; self.status_at=time.monotonic()
        if self.status_writer:
            self.status_writer.writerow([self.status_at-self.started,msg.ctrl_mode,msg.arm_status,
                msg.mode_feedback,msg.motion_status,msg.err_status,any(msg.joint_angle_limit),
                any(msg.communication_status_joint)])

    def healthy(self):
        now=time.monotonic(); s=self.status
        if self.interrupted:
            raise RuntimeError("用户请求停止")
        if self.executor_error is not None:
            raise RuntimeError("反馈接收线程异常："+self.executor_error)
        if not self.thread.is_alive(): raise RuntimeError("反馈接收线程已退出")
        if self.q is None or s is None or now-min(self.q_at,self.header_at,self.status_at)>STALE_S:
            raise RuntimeError("反馈未就绪、已过期或关节时间戳不再更新："
                f"接收角度年龄={now-self.q_at:.3f}s，时间戳更新年龄={now-self.header_at:.3f}s，"
                f"状态年龄={now-self.status_at:.3f}s")
        # 同机ROS时钟核对消息本身年龄，防止队列中的旧消息不断到达掩盖积压。
        stamp_age=(self.node.get_clock().now().nanoseconds-self.header)/1e9
        if abs(stamp_age)>STALE_S:
            raise RuntimeError(f"关节消息时间戳已过期或时钟不一致：年龄={stamp_age:.3f}s")
        if len(s.joint_angle_limit)!=6 or len(s.communication_status_joint)!=6:
            raise RuntimeError("状态标志长度不正确")
        if s.ctrl_mode!=1 or s.mode_feedback!=1 or s.arm_status!=0 or s.err_status!=0 or any(s.joint_angle_limit) or any(s.communication_status_joint):
            raise RuntimeError("真机不在正常CAN/MOVE J状态或报告了故障")

    def future(self,future,timeout=3.,allow_interrupt=False):
        deadline=time.monotonic()+timeout
        while not future.done():
            if self.interrupted and not allow_interrupt:
                raise RuntimeError("用户请求停止")
            if time.monotonic()>deadline:
                raise RuntimeError("服务响应超时，结果不确定")
            self.spin(.01)
        result=future.result()
        if result is None: raise RuntimeError("服务没有返回结果")
        return result

    def service(self,name,kind,request,allow_interrupt=False):
        # 复用客户端；接收线程运行时不反复创建/销毁DDS实体。
        if name not in self.clients: self.clients[name]=self.node.create_client(kind,name)
        client=self.clients[name]
        if not client.wait_for_service(timeout_sec=2.):
            raise RuntimeError(f"服务不存在：{name}")
        return self.future(client.call_async(request),allow_interrupt=allow_interrupt)

    def gate(self,enabled):
        from std_srvs.srv import SetBool
        # 等门控响应后再处理取消，避免已发出的开门请求尚未返回时就退出。
        response=self.service("/control_enable",SetBool,SetBool.Request(data=enabled),allow_interrupt=True)
        if not response.success: raise RuntimeError("门控服务拒绝请求："+response.message)
        return response.message

    def stop_and_close(self,send_stop):
        """电子保持请求在关门前发送；服务返回不代表已物理停止。绝不自动失能。"""
        result={}
        if send_stop:
            from std_srvs.srv import Empty
            try:
                self.service("/emergency_stop",Empty,Empty.Request(),allow_interrupt=True)
                result["stop_service_returned"]=True
            except Exception as exc: result["stop_error"]=str(exc)
        try: result["gate_close"]=self.gate(False)
        except Exception as exc: result["gate_close_error"]=str(exc)
        return result

    def ready(self):
        from rcl_interfaces.srv import GetParameters
        self.gate(False)  # 服务改变内部变量；不使用可能未同步的control_enabled参数判断。
        keys=["arm_type","can_port","fast_mode","speed_percent","auto_enable"]
        response=self.service(DRIVER+"/get_parameters",GetParameters,GetParameters.Request(names=keys))
        v=response.values
        if len(v)!=5 or [v[0].string_value,v[1].string_value]!=["piper","can0"] or \
                v[2].type!=1 or v[2].bool_value or v[3].type!=2 or v[3].integer_value!=5 or \
                v[4].type!=1 or v[4].bool_value:
            raise RuntimeError("驱动必须为piper/can0、fast_mode=false、speed_percent=5、auto_enable=false")
        self.check_graph()
        self.wait_stationary()

    def wait_stationary(self):
        # 图发现/服务查询只在开门前进行；查询后重新积累一段新鲜位置，
        # 不把这些非实时操作造成的接收停顿带入定时发送循环。
        begin=time.monotonic()
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            self.spin(.01)
            if self.interrupted: raise RuntimeError("用户取消")
            if time.monotonic()-begin>=.6 and self.q is not None and self.status:
                self.stationary(); return
        raise RuntimeError("未收到足够的新鲜反馈")

    def check_graph(self):
        drivers=[(n,ns) for n,ns in self.node.get_node_names_and_namespaces() if n=="agx_arm_ctrl_single_node"]
        if drivers!=[("agx_arm_ctrl_single_node","/")]:
            raise RuntimeError("只能存在一个根命名空间真机驱动")
        pubs=self.node.get_publishers_info_by_topic(FEEDBACK)
        if len(pubs)!=1 or pubs[0].node_name!="agx_arm_ctrl_single_node" or pubs[0].node_namespace!="/":
            raise RuntimeError("反馈话题发布者不唯一或不是预期真机驱动")
        for topic in MOTION_TOPICS:
            for pub in self.node.get_publishers_info_by_topic(topic):
                if pub.node_name!=self.node.get_name() or pub.node_namespace!="/":
                    raise RuntimeError(f"存在其他运动发布者：{topic} {pub.node_name}；先关闭MoveIt/其他发送器")

    def stationary(self):
        self.healthy()
        recent=[q for t,q in self.history if time.monotonic()-t<=.5]
        if len(recent)<5 or any(max(row[j] for row in recent)-min(row[j] for row in recent)>.001 for j in range(6)):
            raise RuntimeError("近期位置反馈不稳定，不准备/执行轨迹")
        # 速度来自另一类电机反馈，已有静止非零现象；只记录，不据此计算dq或放宽位置保护。
        return list(self.q)

    def close(self):
        self.executor.shutdown()
        self.thread.join(timeout=3)
        for stream,buffer in self.files:
            try: csv.writer(stream).writerows(buffer.export_rows() if hasattr(buffer,'export_rows') else buffer.rows)
            finally: stream.close()
        self.node.destroy_node(); self.ros.shutdown()
        for s,h in self.old_handlers.items(): signal.signal(s,h)


def prepare(folder,cpp,offset_mm,endpoint=False):
    folder=Path(folder)
    if folder.exists(): raise ValueError("输出目录已存在，请换一个新名称")
    offset=[x/1000. for x in offset_mm]
    if not all(math.isfinite(x) for x in offset) or not 0<math.hypot(*offset)<=.002000000001:
        raise ValueError("相对位移范数必须在(0,2]mm内")
    cpp=Path(cpp).resolve()
    if not cpp.is_file(): raise ValueError("找不到C++程序，请先编译ik_trajectory_validation")
    io=RobotIO(); process=None
    try:
        io.ready(); start=io.stationary()
        if endpoint and max(map(abs,start))>.002:
            raise RuntimeError("反馈不在零位附近（0.002rad内），不会自动回零")
        folder.mkdir(parents=True,exist_ok=False)
        duration=60 if endpoint else 10
        args=[str(cpp),str(folder.resolve()/"trajectory.csv"),str(duration),
              "--endpoint-joint-target" if endpoint else "--relative-target",
              *[format(x,".17g") for x in start],
              *[format(x,".17g") for x in (ENDPOINT_REFERENCE if endpoint else offset)]]
        # 同时保持ROS回调运转，不在C++求解期间阻塞反馈接收。
        with (folder/"generator.log").open("x",encoding="utf-8") as log:
            process=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT)
            deadline=time.monotonic()+90
            while process.poll() is None:
                io.spin(.01); io.healthy()
                if time.monotonic()>deadline: raise RuntimeError("C++生成超时")
        if process.returncode!=0: raise RuntimeError("C++生成失败，请查看generator.log")
        io.healthy()
        if difference(io.stationary(),start)>START_TOL: raise RuntimeError("生成期间起点发生变化，请重新prepare")
        points=load_csv(folder/"trajectory.csv"); metrics=check_points(points,endpoint)
        meta={"kind":"real_move_j_endpoint_ik_v1" if endpoint else "real_move_j_relative_ik_v1",
            "duration_s":duration,"sample_count":len(points),
            "offset_m":offset,"q_start_rad":start,"q_goal_rad":points[-1].positions,
            "cpp":str(cpp),"urdf":str(DEFAULT_URDF),"generator_arguments":args,
            "hashes":{"csv":digest(folder/"trajectory.csv"),"cpp":digest(cpp),"urdf":digest(DEFAULT_URDF)},
            "metrics":metrics,"note":"MOVE J会再次平滑；仅指令时长，不保证实机严格跟踪；未检查碰撞"}
        if endpoint:
            meta.pop("offset_m")
            meta["q_reference_rad"]=ENDPOINT_REFERENCE
            meta["path_mode"]="endpoint_ik_then_joint_quintic"
        save_json(folder/"manifest.json",meta)
        print("准备完成，未发送运动目标。门控保持关闭。")
        print("目录：",folder.resolve()); print("起点：",start); print("终点：",points[-1].positions)
        print("离线检查：",metrics)
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try: process.wait(timeout=3)
            except subprocess.TimeoutExpired: process.kill(); process.wait()
        cleanup=io.stop_and_close(False); io.close()
        if "gate_close_error" in cleanup: print("门控关闭未确认：",cleanup,file=sys.stderr)


def execute(folder):
    from sensor_msgs.msg import JointState
    meta,points,metrics=load_session(folder)
    log_dir=Path(tempfile.mkdtemp(prefix="execution_",dir=folder))
    io=RobotIO(log_dir,command_enabled=True); sent=0; success=False; gate_attempted=False
    result={"success":False,"simulation_only":False,"planned_command_duration_s":meta["duration_s"],
            "csv_sha256":meta["hashes"]["csv"],"metrics":metrics,
            "command_topic":COMMAND,"feedback_topic":FEEDBACK}
    print("执行记录目录：",log_dir,flush=True)
    try:
        io.ready()
        if difference(io.stationary(),points[0].positions)>START_TOL:
            raise RuntimeError("当前位置与CSV起点差超过0.001rad；不要补发旧轨迹，请重新prepare新目录")
        pub=io.publisher
        if pub.topic_name!=COMMAND: raise RuntimeError("控制话题被重映射")
        commands=io.log(log_dir,"commands.csv",["index","target_time_s","send_elapsed_s","lateness_s",*[f"q{i}_rad" for i in range(1,7)]])
        deadline=time.monotonic()+3
        while pub.get_subscription_count()!=1:
            if time.monotonic()>deadline: raise RuntimeError("运动话题必须只有一个驱动订阅者")
            io.spin(.01);io.healthy()
        subs=io.node.get_subscriptions_info_by_topic(COMMAND)
        if len(subs)!=1 or subs[0].node_name!="agx_arm_ctrl_single_node" or subs[0].node_namespace!="/":
            raise RuntimeError("运动话题订阅者不是预期真机驱动；停止其他订阅/记录器")
        io.check_graph(); io.wait_stationary()
        if difference(io.q,points[0].positions)>START_TOL: raise RuntimeError("起点已变化")
        print(f"即将发送{meta['duration_s']}秒MOVE J指令；Ctrl+C请求电子保持并关门控，不等于硬件急停。",flush=True)
        gate_attempted=True; io.gate(True); io.healthy()
        if difference(io.q,points[0].positions)>START_TOL: raise RuntimeError("开门控后起点变化")
        start=time.monotonic()
        # feedback/status使用io.started；保存时钟原点差，供MATLAB直接对齐。
        result["trajectory_start_receive_elapsed_s"]=start-io.started
        result["endpoint_observation_s"]=2.0
        last_target=points[0].positions; next_progress=0
        for k,point in enumerate(points):
            deadline=start+point.time_ns/1e9
            while time.monotonic()<deadline:
                io.spin(min(.005,max(0.,deadline-time.monotonic())))
                io.healthy()
                if difference(io.q,last_target)>TRACK_TOL: raise RuntimeError("反馈与最近已发目标差超过0.01rad")
            io.spin(0); io.healthy()
            if difference(io.q,last_target)>TRACK_TOL: raise RuntimeError("跟随偏差超过0.01rad")
            io.healthy()
            lateness=time.monotonic()-deadline
            if lateness>=LATE_S: raise RuntimeError("发送调度迟到达到20ms（一个周期）；停止而非跨周期追赶补发")
            msg=JointState();msg.name=JOINTS;msg.position=point.positions
            # 只发位置；move_j回调不使用velocity/effort。不发送夹爪字段。
            pub.publish(msg);sent+=1;last_target=point.positions
            elapsed=time.monotonic()-start
            commands.writerow([k,point.time_ns/1e9,elapsed,lateness,*point.positions])
            if point.time_ns/1e9>=next_progress:
                print(f"指令进度 {point.time_ns/1e9:.2f}s；已发{sent}/{len(points)}点",flush=True);next_progress+=1
        result["command_elapsed_s"]=time.monotonic()-start
        # 不继续发终点：观察2秒，允许默认底层规划完成，并核对位置/状态。
        deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            io.spin(.01);io.healthy()
            if difference(io.q,last_target)>TRACK_TOL: raise RuntimeError("终点观察期间偏差超过0.01rad")
        final=io.stationary();error=difference(final,last_target)
        if error>FINAL_TOL or io.status.motion_status!=0: raise RuntimeError("终点未到位或偏差超过0.005rad")
        result.update(success=True,q_final_rad=final,final_joint_error_max_rad=error)
        success=True
        print(f"本轮完成：已发{sent}点；终点最大关节偏差{error:.6g}rad。不是末端实测精度。")
    except BaseException as exc:
        result["exception"]=str(exc)
        raise
    finally:
        result["sent_points"]=sent
        if gate_attempted:
            result["cleanup"]=io.stop_and_close(sent>0 and not success)
        else:
            result["cleanup"]=io.stop_and_close(False)
        if "gate_close_error" in result["cleanup"]:
            result["success"]=False
            print("警告：关门控未确认，请人工检查并使用停止措施。",file=sys.stderr)
        try: save_json(log_dir/"summary.json",result)
        finally: io.close()
        print("已保存：",log_dir,flush=True)
        if success and not result["success"]: raise RuntimeError("运动结束但门控清理失败")


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest="mode",required=True)
    p=sub.add_parser("prepare",help="关门控、采集起点、调用C++；不发运动目标")
    p.add_argument("folder",type=Path)
    p.add_argument("--offset-mm",type=float,nargs=3,default=[0.,0.,2.],metavar=("DX","DY","DZ"))
    p.add_argument("--cpp",type=Path,default=REPO/"build/debug/ik_trajectory_validation")
    p=sub.add_parser("prepare-endpoint",help="近零起点->指定T的终点IK；60秒关节五次轨迹，不发运动目标")
    p.add_argument("folder",type=Path)
    p.add_argument("--cpp",type=Path,default=REPO/"build/debug/ik_trajectory_validation")
    p=sub.add_parser("check",help="纯离线检查")
    p.add_argument("folder",type=Path)
    p=sub.add_parser("execute",help="发送真机运动指令")
    p.add_argument("folder",type=Path)
    p.add_argument("--confirm-real-hardware",action="store_true",required=True)
    args=parser.parse_args()
    try:
        if args.mode in ("prepare","prepare-endpoint"):
            with robot_lock(): prepare(args.folder,args.cpp,getattr(args,"offset_mm",[0,0,2]),args.mode=="prepare-endpoint")
        elif args.mode=="check":
            meta,points,metrics=load_session(args.folder)
            print("离线检查通过；未初始化ROS、未发指令。")
            print("起点：",points[0].positions,"\n终点：",points[-1].positions,"\n",metrics)
        else:
            with robot_lock(): execute(args.folder)
        return 0
    except (Exception,KeyboardInterrupt) as exc:
        print("失败：",exc,file=sys.stderr);return 1


if __name__=="__main__":
    raise SystemExit(main())
