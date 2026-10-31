"""POE空间/物体雅可比；旋量顺序[角;线]，不读取实验文件。"""
import numpy as np

from ..common.validation import finite
from ..lie_group.transforms import adjoint, exp_twist, inverse
from .forward import forward


def jacobian_space(model, q) -> np.ndarray:
    q = finite(q, (model.n,))
    result = np.zeros_like(model.S)
    prefix = np.eye(4)
    for joint in range(model.n):
        result[:, joint] = adjoint(prefix) @ model.S[:, joint]
        prefix = prefix @ exp_twist(model.S[:, joint] * q[joint])
    return result


def jacobian_body(model, q, pose=None) -> np.ndarray:
    """pose可复用已算出的同一q的FK；省略时自行计算。"""
    if pose is None:
        pose = forward(model, q)
    return adjoint(inverse(pose)) @ jacobian_space(model, q)
