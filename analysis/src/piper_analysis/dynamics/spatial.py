"""空间运动/力叉乘与串联父子变换，[角;线]约定。"""
from __future__ import annotations
import numpy as np
from ..common.validation import finite
from ..lie_group.transforms import skew, adjoint, inverse, exp_twist

def motion_cross(v):
    v = finite(v, (6,))
    W = skew(v[:3])
    return np.block([[W, np.zeros((3, 3))], [skew(v[3:]), W]])

def force_cross(v):
    return -motion_cross(v).T

def chain_transforms(model, q):
    q = finite(q, (model.n,))
    A, X = ([], [])
    T = np.eye(4)
    finite(model.S, (6, model.n))
    finite(model.Mlist, (model.n + 1, 4, 4))
    finite(model.Glist, (model.n, 6, 6))
    finite(model.gravity, (3,))
    for i in range(model.n):
        T = T @ model.Mlist[i]
        a = adjoint(inverse(T)) @ model.S[:, i]
        A.append(a)
        X.append(adjoint(exp_twist(-a*q[i]) @ inverse(model.Mlist[i])))
    return np.array(A), np.array(X)
