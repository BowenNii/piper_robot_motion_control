"""CAN导出CSV到关节/位姿序列的适配与格式检查。"""
from __future__ import annotations
import numpy as np
from .csv import read_numeric, columns
from ..validation.timing import strict_time

def feedback(path, strict=True):
    names = ['t_s', 'timestamp_s', 'frame_span_s']+[f'q{i}_rad' for i in range(1, 7)]
    names += [f'poe_{a}_m' for a in 'xyz']+[f'poe_r{i}{j}' for i in range(1, 4) for j in range(1, 4)]
    d = read_numeric(path, names)
    if strict:
        strict_time(d['timestamp_s'])
        strict_time(d['t_s'])
    elif np.any(np.diff(d['timestamp_s']) < 0) or np.any(np.diff(d['t_s']) < 0):
        raise ValueError("Time moved backward")
    if np.max(np.abs((d['t_s']-d['t_s'][0])-(d['timestamp_s']-d['timestamp_s'][0]))) > 1e-6:
        raise ValueError("Relative/epoch time inconsistent")
    if np.any(d['frame_span_s'] < 0):
        raise ValueError("Negative CAN span")
    q = columns(d, [f'q{i}_rad' for i in range(1, 7)])
    T = np.repeat(np.eye(4)[None], len(q), axis=0)
    T[:, :3, :3] = columns(d, [f'poe_r{i}{j}' for i in range(1, 4) for j in range(1, 4)]).reshape(-1, 3, 3)
    T[:, :3, 3] = columns(d, [f'poe_{a}_m' for a in 'xyz'])
    if not np.allclose(T[:, :3, :3].transpose(0, 2, 1) @ T[:, :3, :3], np.eye(3), atol=1e-7, rtol=0):
        raise ValueError("Invalid rotation matrices")
    if not np.allclose(np.linalg.det(T[:, :3, :3]), 1, atol=1e-7, rtol=0):
        raise ValueError("Reflection instead of SO(3)")
    return d, q, T
