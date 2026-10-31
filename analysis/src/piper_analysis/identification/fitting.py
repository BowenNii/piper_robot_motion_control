"""SVD最小二乘/岭回归，报告秩与拟合残差。"""
from __future__ import annotations
import numpy as np
from ..common.validation import finite

def fit_parameters(Y, tau, ridge=0.):
    """SVD最小二乘；报告秩。结果不是自动物理可行或已辨识参数。"""
    Y, tau = finite(Y), finite(tau)
    if Y.ndim != 2 or tau.shape != (len(Y),) or len(Y) == 0:
        raise ValueError("Expected stacked (samples*n,parameters) and torque vector")
    if not np.isfinite(ridge) or ridge < 0:
        raise ValueError("ridge must be nonnegative")
    singular = np.linalg.svd(Y, compute_uv=False)
    if ridge:
        design = np.vstack([Y, np.sqrt(ridge)*np.eye(Y.shape[1])])
        values = np.r_[tau, np.zeros(Y.shape[1])]
    else:
        design, values = Y, tau
    pi = np.linalg.lstsq(design, values, rcond=None)[0]
    return dict(parameters=pi, rank=int(np.linalg.matrix_rank(Y)), singular_values=singular,
                residual_rms=float(np.sqrt(np.mean((Y@pi-tau)**2))))
