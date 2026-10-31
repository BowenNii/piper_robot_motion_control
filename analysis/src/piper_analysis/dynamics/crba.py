"""复合刚体法质量矩阵，不依赖数据读写。"""
from __future__ import annotations
import numpy as np
from typing import TYPE_CHECKING
from numpy.typing import ArrayLike
from .spatial import chain_transforms

if TYPE_CHECKING:
    from ..robot_model.types import RobotModel

def crba(q: ArrayLike, model: RobotModel) -> np.ndarray:
    """复合刚体惯量矩阵，独立于逐列调用RNEA的实现。"""
    A, X = chain_transforms(model, q)
    Ic = model.Glist.copy()
    n = model.n
    M = np.zeros((n, n))
    for i in range(n-1, -1, -1):
        F = Ic[i] @ A[i]
        M[i, i] = A[i] @ F
        j = i
        while j > 0:
            F = X[j].T @ F
            j -= 1
            M[i, j] = M[j, i] = A[j] @ F
        if i > 0:
            Ic[i-1] += X[i].T @ Ic[i] @ X[i]
    return M
