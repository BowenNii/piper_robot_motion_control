"""重力、速度偏置和理想正动力学。"""
from __future__ import annotations
import numpy as np
from ..common.validation import finite
from dataclasses import replace
from .rnea import rnea
from .crba import crba

def gravity(q, model):
    return rnea(q, np.zeros(model.n), np.zeros(model.n), model)

def velocity_bias(q, dq, model):
    """返回c(q,dq)向量，不声称返回唯一的C矩阵。"""
    return rnea(q, dq, np.zeros(model.n), replace(model, gravity=np.zeros(3)))

def forward_dynamics(q, dq, tau, model):
    """理想无摩擦、无外力正动力学；质量矩阵奇异时显式失败。"""
    tau = finite(tau, (model.n,))
    return np.linalg.solve(crba(q, model), tau-rnea(q, dq, np.zeros(model.n), model))
