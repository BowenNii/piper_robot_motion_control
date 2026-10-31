"""数值输入检查；不裁剪、不静默修复数据。"""
from __future__ import annotations
import numpy as np
from numpy.typing import ArrayLike

def finite(value: ArrayLike, shape: tuple[int, ...] | None = None) -> np.ndarray:
    a = np.asarray(value, dtype=float)
    if not np.isfinite(a).all() or (shape is not None and a.shape != shape):
        raise ValueError(f"Expected finite array of shape {shape}, got {a.shape}")
    return a
