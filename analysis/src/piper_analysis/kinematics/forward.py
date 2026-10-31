"""空间POE正运动学与批量计算。"""
from __future__ import annotations
import numpy as np
from typing import TYPE_CHECKING
from numpy.typing import ArrayLike
from ..common.validation import finite
from ..lie_group.transforms import exp_twist

if TYPE_CHECKING:
    from ..robot_model.types import RobotModel

def forward(model: RobotModel, q: ArrayLike) -> np.ndarray:
    q = finite(q, (model.n,))
    T = np.eye(4)
    for i in range(model.n):
        T = T @ exp_twist(model.S[:, i]*q[i])
    return T @ model.home

def fk_sequence(model: RobotModel, q: ArrayLike) -> np.ndarray:
    q = finite(q)
    if q.ndim != 2 or q.shape[1] != model.n:
        raise ValueError("Expected (N,n) joint sequence")
    return np.array([forward(model, row) for row in q])
