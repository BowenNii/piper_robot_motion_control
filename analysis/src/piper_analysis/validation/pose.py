"""同基座、同参考点的位姿误差，位置m、姿态rad。"""
from __future__ import annotations
import numpy as np
from ..common.validation import finite

def pose_error(actual, reference):
    """支持(...,4,4)，返回平移范数m与SO(3)最短角rad；不是RPY相减。"""
    actual, reference = finite(actual), finite(reference)
    if actual.shape[-2:] != (4, 4) or reference.shape[-2:] != (4, 4):
        raise ValueError("Expected poses (...,4,4)")
    R = np.swapaxes(reference[..., :3, :3], -1, -2) @ actual[..., :3, :3]
    v = np.stack([R[..., 2, 1]-R[..., 1, 2], R[..., 0, 2]-R[..., 2, 0],
                  R[..., 1, 0]-R[..., 0, 1]], axis=-1)/2
    angle = np.arctan2(np.linalg.norm(v, axis=-1),
                       np.clip((np.trace(R, axis1=-2, axis2=-1)-1)/2, -1, 1))
    return np.linalg.norm(actual[..., :3, 3]-reference[..., :3, 3], axis=-1), angle
