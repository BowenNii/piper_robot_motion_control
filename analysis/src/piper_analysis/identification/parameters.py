"""每link十维原点惯性参数，不是质心惯量参数。"""
from __future__ import annotations
import numpy as np

def inertial_parameters(model):
    """每link: m,hx,hy,hz,Ixx,Iyy,Izz,Ixy,Ixz,Iyz；I在link原点。"""
    rows = []
    for G in model.Glist:
        H = G[:3, 3:]
        rows.extend([np.trace(G[3:, 3:])/3, H[2, 1], H[0, 2], H[1, 0],
                     G[0, 0], G[1, 1], G[2, 2], G[0, 1], G[0, 2], G[1, 2]])
    return np.array(rows)
