"""递归牛顿—欧拉逆动力学，不含摩擦/转子惯量。Ftip采用所需抵抗的负载符号。"""
from __future__ import annotations
import numpy as np
from typing import TYPE_CHECKING
from numpy.typing import ArrayLike
from ..common.validation import finite
from ..lie_group.transforms import adjoint, inverse
from .spatial import chain_transforms, motion_cross, force_cross

if TYPE_CHECKING:
    from ..robot_model.types import RobotModel

def rnea(q: ArrayLike, dq: ArrayLike, ddq: ArrayLike, model: RobotModel,
         Ftip: ArrayLike | None = None) -> np.ndarray:
    """tau=M(q)ddq+c(q,dq)+g(q)，不含摩擦。SI单位。"""
    n = model.n
    q, dq, ddq = [finite(v, (n,)) for v in (q, dq, ddq)]
    F = np.zeros(6) if Ftip is None else finite(Ftip, (6,)).copy()
    A, X = chain_transforms(model, q)
    v = np.zeros((n + 1, 6))
    acc = v.copy()
    acc[0, 3:] = -model.gravity
    for i in range(n):
        v[i+1] = X[i] @ v[i] + A[i]*dq[i]
        acc[i+1] = X[i] @ acc[i] + A[i]*ddq[i] + motion_cross(v[i+1]) @ (A[i]*dq[i])
    tau = np.zeros(n)
    child = adjoint(inverse(model.Mlist[-1]))
    for i in range(n-1, -1, -1):
        G = model.Glist[i]
        F = child.T @ F + G @ acc[i+1] + force_cross(v[i+1]) @ (G @ v[i+1])
        tau[i] = A[i] @ F
        child = X[i]
    return tau
