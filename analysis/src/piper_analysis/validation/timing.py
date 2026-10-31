"""时间轴检查、实际发布参考索引与持续时间检测。"""
from __future__ import annotations
import numpy as np
from ..common.validation import finite

def strict_time(t):
    t = finite(t)
    if t.ndim != 1 or len(t) < 2 or np.any(np.diff(t) <= 0):
        raise ValueError("Need >=2 strictly increasing times")
    return t

def zoh_indices(times, query):
    times = strict_time(times)
    query = finite(query)
    if np.any(query < times[0]) or np.any(query > times[-1]):
        raise ValueError("No extrapolation beyond published command coverage")
    return np.searchsorted(times, query, side='right')-1

def first_time(t, mask, hold_s=0.):
    start = None
    for i, flag in enumerate(mask):
        if flag:
            if start is None:
                start = i
            if t[i]-t[start] >= hold_s:
                return float(t[start])
        else:
            start = None
    return np.nan
