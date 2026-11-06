"""C轨迹准备与延迟交叉验证；A/B不构造中间时间参考。"""
from __future__ import annotations
import numpy as np
from ...kinematics.forward import fk_sequence
from ...validation.path import path_distance
from ...validation.alignment import fit_delay, tracking_errors
from ...validation.statistics import trace_metrics


def errors(z: dict, lag: float):
    return tracking_errors(z['tr'], z['qtrace'], z['Ttrace'], z['ct'], z['qc'], z['refT'], lag)


def prepare_stream(z: dict, model) -> None:
    """计算空间路径、保留原始时间轴，拟合一个共同延迟候选值。"""
    t, q, T, ct, qc = (z[key] for key in ('t', 'q', 'T', 'ct', 'qc'))
    duration = z['duration']
    z['refT'] = fk_sequence(model, qc)
    z['refP'] = z['refT'][:, :3, 3] * 1000
    mask = (t >= ct[0]) & (t <= duration)
    z.update(tr=t[mask], qtrace=q[mask], Ttrace=T[mask], ptrace=T[mask, :3, 3]*1000)
    #ptrace实际路径：反馈对应的位姿 → 筛选 → 位置；refP指令关节角 qc 经 FK 计算希望机械臂走出的参考路径
    z['path_forward'] = path_distance(z['ptrace'], z['refP'])
    z['path_reverse'] = path_distance(z['refP'], z['ptrace'])

    z['lag_candidate'], z['lag_boundary'] = fit_delay(
        z['tr'], z['qtrace'], ct, qc, z['active'], duration)


def evaluate_stream(z: dict, source: dict, cfg: dict) -> None:
    """source必须为同组去程；返程沿用去程延迟，不重新寻找最优值。"""
    record = z['record']
    man = record['manifest']
    row = z['row']
    lag = source['lag_candidate']
    z['lag_used'] = lag
    je, pe, re = errors(z, 0)
    ae, ap, ar = errors(z, lag)
    valid = z['tr'] >= z['ct'][0]+lag
    z.update(jerror=je, pe=pe, re=re, aligned_t=z['tr'][valid],
             jaligned=ae[valid], paligned=ap[valid], raligned=ar[valid])
    for prefix, value, key in [('joint', abs(je).max(axis=1), 'deg'),
                               ('position', pe, 'mm'), ('rotation', re, 'deg')]:
        m = trace_metrics(z['tr'], value, cfg[f'{"orientation" if prefix == "rotation" else prefix}_tolerance_{key}'])
        row[f'stream_{prefix}_peak_{key}'] = m['peak']
        row[f'stream_{prefix}_rms_{key}'] = m['rms']
        row[f'{prefix}_over_fraction'] = m['over_fraction']
        row[f'{prefix}_longest_over_s'] = m['longest_over_s']
    for prefix, value, key in [('joint', abs(ae[valid]).max(axis=1), 'deg'),
                               ('position', ap[valid], 'mm'), ('rotation', ar[valid], 'deg')]:
        row[f'aligned_{prefix}_rms_{key}'] = trace_metrics(z['aligned_t'], value)['rms']
    row.update(path_forward_peak_mm=z['path_forward'].max(),
               path_forward_rms_mm=np.sqrt(np.mean(z['path_forward']**2)),
               path_reverse_peak_mm=z['path_reverse'].max(), lag_from_outbound_ms=lag*1000,
               lag_search_boundary=int(source['lag_boundary']))
    holdout = valid.copy()
    if man['direction'] == 'outbound':
        holdout &= ~((z['tr'] >= .2*z['duration']) & (z['tr'] <= .8*z['duration']))
    for name, values in [('raw', je), ('aligned', ae)]:
        row[f'{name}_holdout_joint_rms_deg'] = np.sqrt(np.mean(abs(values[holdout]).max(axis=1)**2))
