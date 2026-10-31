"""离线重采样，拒绝跨越未允许的大间隔。"""
from __future__ import annotations
import numpy as np
from ..common.validation import finite
from ..validation.timing import strict_time

def uniform_resample(t, values, period, max_gap):
    """只有指定允许最大间隔才插值；拒绝跨越丢包大间隔。"""
    t = strict_time(t)
    values = finite(values)
    if len(values) != len(t) or not np.isfinite(period) or period <= 0 or not np.isfinite(max_gap) or max_gap <= 0:
        raise ValueError('Invalid resample arguments')
    if np.max(np.diff(t)) > max_gap:
        raise ValueError('Gap exceeds explicit interpolation allowance')
    grid = t[0]+np.arange(int(np.floor((t[-1]-t[0])/period))+1)*period
    flat = values.reshape(len(t), -1)
    result = np.column_stack([np.interp(grid, t, flat[:, j]) for j in range(flat.shape[1])])
    return grid, result.reshape((len(grid),)+values.shape[1:])
