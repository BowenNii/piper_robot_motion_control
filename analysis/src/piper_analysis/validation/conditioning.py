"""任务加权奇异值指标；不单独据此宣称真机安全。"""
import numpy as np

from ..common.validation import finite


def weighted_sigma_min(body_jacobian, rotation_weight: float) -> float:
    matrix = finite(body_jacobian).copy()
    if matrix.ndim != 2 or matrix.shape[0] != 6 or matrix.shape[1] == 0:
        raise ValueError('Expected a nonempty 6xn body Jacobian')
    if not np.isfinite(rotation_weight) or rotation_weight <= 0:
        raise ValueError('Rotation weight must be finite and positive')
    matrix[:3] *= rotation_weight
    return np.linalg.svd(matrix, compute_uv=False)[-1]
