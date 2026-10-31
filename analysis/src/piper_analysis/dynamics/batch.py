"""批量标称动力学预测，无文件读写和绘图。"""
from __future__ import annotations
import numpy as np
from .rnea import rnea
from .crba import crba
from .terms import gravity, velocity_bias

def torque_prediction(q, dq, ddq, model):
    q, dq, ddq = [np.asarray(v, dtype=float) for v in (q, dq, ddq)]
    if q.ndim != 2 or q.shape[1] != model.n or dq.shape != q.shape or ddq.shape != q.shape:
        raise ValueError('Expected three (N,n) arrays')
    total, g, c, inertial = [], [], [], []
    for qi, vi, ai in zip(q, dq, ddq):
        total.append(rnea(qi, vi, ai, model))
        g.append(gravity(qi, model))
        c.append(velocity_bias(qi, vi, model))
        inertial.append(crba(qi, model) @ ai)
    return {key: np.array(value) for key, value in
            [('total', total), ('gravity', g), ('velocity_bias', c), ('inertial', inertial)]}
