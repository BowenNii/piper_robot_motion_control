"""单段实验：独立FK核对、终点与响应指标，不读写文件。"""
from __future__ import annotations
import numpy as np
from ...common.records import FeedbackRecord
from ...robot_model.types import RobotModel
from ...kinematics.forward import forward, fk_sequence
from ...validation.pose import pose_error
from ...validation.response import joint_responses
from .stream import prepare_stream

def analyze_record(data: FeedbackRecord, model: RobotModel) -> tuple[dict, list[dict]]:
    """位置内部m，输出表为mm/deg；终点仍取计划结束后2～3s均值。"""
    record = data.metadata
    man = record['manifest']
    t, q, T = data.time_s, data.q_rad, data.poses
    ct, qc = data.command_time_s, data.command_q_rad
    qs, qg = data.q_start_rad, data.q_goal_rad
    active, duration = data.active_joints, data.duration_s
    # MATLAB round(linspace(1,N,101))使用正数四舍五入；Python内建round为银行舍入。
    ix = np.unique(np.floor(np.linspace(1, len(t), 101)+.5).astype(int)-1)
    ep, er = pose_error(fk_sequence(model, q[ix]), T[ix])
    if ep.max() > 1e-9 or er.max() > 1e-7:
        raise ValueError("Python/C++ FK crosscheck failed")
    tail = (t >= duration+2) & (t <= duration+3)
    if tail.sum() <= 20:
        raise ValueError("Insufficient endpoint hold samples")
    qend = q[tail].mean(axis=0)
    endp, endr = pose_error(forward(model, qend), forward(model, qg))

    response = []
    for values in joint_responses(t, q, ct[0], qs, qg, active):
        response.append(dict(record=record['record'], group=record['group'],
                             mode=man['mode'], direction=man['direction'], **values))
    span = data.frame_span_ms
    row = dict(record=record['record'], group=record['group'], run_name=record['run_name'],
               span_max_ms=span.max(), span_p95_ms=np.sort(span)[int(np.ceil(len(span)*.95))-1],
               span_over2_fraction=np.mean(span > 2),
               pc_lateness_max_ms=data.command_lateness_ms.max(),
               ros_receive_gap_max_ms=data.receive_gap_ms.max(),
               endpoint_joint_max_deg=np.rad2deg(abs(qend-qg).max()),
               endpoint_model_position_mm=float(endp*1000), endpoint_model_rotation_deg=float(np.rad2deg(endr)),
               fk_crosscheck_max_m=ep.max())
    z = dict(record=record, row=row, t=t, q=q, T=T, p=T[:, :3, 3]*1000, ct=ct,
             qc=qc, qs=qs, qg=qg, active=active, duration=duration)

    if man['mode'] == 'stream':
        prepare_stream(z, model)
    return z, response
