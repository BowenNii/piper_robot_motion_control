"""SE(3)变换；长度m、角度rad，旋量[omega;v]。"""
from __future__ import annotations
import numpy as np
from scipy.spatial.transform import Rotation
from ..common.validation import finite

def skew(v):
    x, y, z = finite(v, (3,))
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])

def transform(R=None, p=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = finite(R, (3, 3))
    if p is not None:
        T[:3, 3] = finite(p, (3,))
    return T

def inverse(T):
    T = finite(T, (4, 4))
    R = T[:3, :3].T
    return transform(R, -R @ T[:3, 3])

def adjoint(T):
    R, p = T[:3, :3], T[:3, 3]
    return np.block([[R, np.zeros((3, 3))], [skew(p) @ R, R]])

def exp_twist(x):
    """Exp([omega;v])；输入已乘关节增量，兼容纯平移和小转角。"""
    x = finite(x, (6,))
    w, v = x[:3], x[3:]
    theta = np.linalg.norm(w)
    W = skew(w)
    if theta < 1e-6:
        a = .5 - theta**2 / 24 + theta**4 / 720
        b = 1/6 - theta**2 / 120 + theta**4 / 5040
    else:
        a = (1 - np.cos(theta)) / theta**2
        b = (theta - np.sin(theta)) / theta**3
    return transform(Rotation.from_rotvec(w).as_matrix(),
                     (np.eye(3) + a*W + b*W@W) @ v)

def log_transform(T):
    """SE(3)主值对数[omega;v]，用于离线插值核对。"""
    T = finite(T, (4, 4))
    w = Rotation.from_matrix(T[:3, :3]).as_rotvec()
    W = skew(w)
    theta = np.linalg.norm(w)
    if theta < 1e-6:
        a, b = .5-theta**2/24+theta**4/720, 1/6-theta**2/120+theta**4/5040
    else:
        a, b = (1-np.cos(theta))/theta**2, (theta-np.sin(theta))/theta**3
    v = np.linalg.solve(np.eye(3)+a*W+b*W@W, T[:3, 3])
    return np.r_[w, v]
