"""离线惯性回归矩阵及样本/轴顺序堆叠。"""
from __future__ import annotations
import numpy as np
from dataclasses import replace
from ..lie_group.transforms import skew
from ..dynamics.rnea import rnea

def regressor(q, dq, ddq, model):
    """慢速离线10n参数回归矩阵，不用于实时环。基矩阵无需物理有效。"""
    Gs = np.zeros_like(model.Glist)
    Y = np.zeros((model.n, 10 * model.n))
    for i in range(model.n):
        for k in range(10):
            G = np.zeros((6, 6))
            if k == 0:
                G[3:, 3:] = np.eye(3)
            elif k < 4:
                h = np.eye(3)[k - 1]
                G[:3, 3:] = skew(h)
                G[3:, :3] = skew(h).T
            else:
                row, col = [(0, 0), (1, 1), (2, 2), (0, 1), (0, 2), (1, 2)][k-4]
                G[row, col] = G[col, row] = 1
            Gs[i] = G
            Y[:, 10*i+k] = rnea(q, dq, ddq, replace(model, Glist=Gs))
            Gs[i] = 0
    return Y

def stack_regressors(q, dq, ddq, model):
    """堆叠顺序：样本1的所有轴，再样本2；tau应以(N,n).reshape(-1)对应。"""
    q, dq, ddq = [np.asarray(v, dtype=float) for v in (q, dq, ddq)]
    if q.ndim != 2 or q.shape[1] != model.n or dq.shape != q.shape or ddq.shape != q.shape or not len(q):
        raise ValueError('Expected nonempty (N,n) sequences')
    return np.vstack([regressor(a, b, c, model) for a, b, c in zip(q, dq, ddq)])
