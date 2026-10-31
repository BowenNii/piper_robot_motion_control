"""历史动态FK统计，不虚构未记录的动态目标参考。"""
from __future__ import annotations
import numpy as np
from pathlib import Path
from ...common.paths import REPO
from ...robot_model.types import RobotModel
from ...kinematics.forward import fk_sequence
from ...validation.pose import pose_error
from ...validation.phases import split_motion_phases
from ...validation.statistics import summarize_masked as _summary
from ...data_io.csv import columns, write_rows
from ...data_io.feedback import feedback
from ...data_io.provenance import sha256
from .baseline import _compare
from scipy.spatial.transform import Rotation
from ...plotting.history import plot_dynamic_run

def analyze_dynamic(folder: Path, model: RobotModel, hashes: dict) -> None:
    root = REPO / 'experiments/project1'
    folder.mkdir()
    rows = []
    phases = []
    dynamic = root/'2026-10-05_dynamic_fk'
    targets = [[np.deg2rad(2), 0, 0, 0, 0, 0], [.3, 0, 0, 0, 0, 0], [0]*6, [.3, .3, -.3, .3, .3, .5], [0]*6]
    labels = ['小幅单轴预检', '单轴去程', '单轴回零', '多轴去程', '多轴回零']
    for file, target, label in zip(sorted((dynamic/'data_csv').glob('run*.csv')), targets, labels):
        print(f'历史动态分析 {file.stem}', flush=True)
        hashes[str(file.relative_to(REPO))] = sha256(file)
        d, q, T = feedback(file)
        t = d['t_s']
        t = t - t[0]
        qdeg = np.rad2deg(q)
        pythonT = fk_sequence(model, q)
        cp, cr = pose_error(pythonT, T)
        can = np.repeat(np.eye(4)[None], len(q), axis=0)
        can[:, :3, :3] = Rotation.from_euler('xyz', columns(d, [f'feedback_{a}_rad' for a in ['roll', 'pitch', 'yaw']])).as_matrix()
        can[:, :3, 3] = columns(d, [f'feedback_{a}_m' for a in 'xyz'])
        p, r = pose_error(pythonT, can)
        p *= 1000
        r = np.rad2deg(r)
        span = d['frame_span_s'] * 1000
        phase, bounds, segments = split_motion_phases(t, qdeg)
        endpoint = t >= t[-1] - 1
        dt = np.diff(t)
        row = _summary(t, p, r, span, np.ones(len(t), bool))
        row.update(run=file.stem, label=label, samples=len(t), duration_s=t[-1], motion_start_s=bounds[0],
                   motion_end_s=bounds[1], motion_duration_s=bounds[1]-bounds[0], activity_segments=int(segments),
                   median_dt_ms=np.median(dt)*1000, max_dt_ms=dt.max()*1000,
                   large_gap_count=int(np.sum(dt > 3*np.median(dt))), over_window_count=int(np.sum(span > 2)),
                   limit_violation_count=int(np.sum(np.any((q < model.q_min-np.deg2rad(.001)) | (q > model.q_max+np.deg2rad(.001)), axis=1))),
                   cpp_position_max_mm=cp.max()*1000, cpp_rotation_max_deg=np.rad2deg(cr.max()),
                   endpoint_max_abs_error_deg=np.max(abs(np.rad2deg(q[endpoint].mean(axis=0)-target))),
                   endpoint_max_range_deg=np.ptp(qdeg[endpoint], axis=0).max())
        rows.append(row)
        for k, name in enumerate(['pre_static', 'motion_buffered', 'post_static'], 1):
            mask = phase == k
            tt = t[mask]
            stats = _summary(t, p, r, span, mask)
            stats.update(run=file.stem, phase=name, samples=int(mask.sum()), start_s=tt[0] if len(tt) else np.nan,
                         end_s=tt[-1] if len(tt) else np.nan, duration_s=tt[-1]-tt[0] if len(tt) else np.nan,
                         sufficient_static_baseline=int(k != 2 and len(tt) > 0 and tt[-1]-tt[0] >= .5))
            phases.append(stats)
        plot_dynamic_run(t, qdeg, p, r, span, folder, file)
    write_rows(folder / 'run_summary.csv', rows)
    write_rows(folder / 'phase_summary.csv', phases)
    _compare(rows, dynamic/'analysis/run_summary.csv', folder/'matlab_comparison.csv')
    _compare(phases, dynamic/'analysis/phase_summary.csv', folder/'phase_matlab_comparison.csv')
