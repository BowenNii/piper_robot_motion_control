"""从扁平CSV列还原位姿序列，不重新计算FK。"""
from __future__ import annotations
import numpy as np
from .csv import columns

def pose_columns(d, prefix):
    T = np.repeat(np.eye(4)[None], len(d['index']), axis=0)
    T[:, :3, 3] = columns(d, [f'{prefix}_{a}_m' for a in 'xyz'])
    T[:, :3, :3] = columns(d, [f'{prefix}_r{i}{j}' for i in range(1, 4) for j in range(1, 4)]).reshape(-1, 3, 3)
    return T
