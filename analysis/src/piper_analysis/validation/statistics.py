"""左端保持时间加权RMS；末样本不外推。"""
from __future__ import annotations
import numpy as np
from ..common.validation import finite
from .timing import strict_time

def trace_metrics(t, error, limit=None):
    t = strict_time(t)
    error = finite(error, t.shape)
    if np.any(error < 0):
        raise ValueError("Error must be nonnegative")
    dt = np.diff(t)
    v = error[:-1]
    result = dict(peak=float(error.max()), rms=float(np.sqrt(np.sum(v*v*dt)/dt.sum())),
                  over_fraction=np.nan, longest_over_s=np.nan)
    if limit is not None:
        if isinstance(limit, bool) or not np.isfinite(limit) or limit <= 0:
            raise ValueError("Limit must be positive or None")
        over = v > limit
        current = longest = 0.0
        for d, flag in zip(dt, over):
            current = current+d if flag else 0.
            longest = max(longest, current)
        result.update(over_fraction=float(dt[over].sum()/dt.sum()), longest_over_s=float(longest))
    return result

def summarize_masked(t, p, r, span, mask):
    out = {}
    for prefix, values, unit in [('position', p, 'mm'), ('rotation', r, 'deg')]:
        x = values[mask]
        tm = t[mask]
        for stat, value in [('mean', np.mean(x) if len(x) else np.nan),
                            ('rms', np.sqrt(np.mean(x*x)) if len(x) else np.nan),
                            ('max', np.max(x) if len(x) else np.nan)]:
            out[f'{prefix}_{stat}_{unit}'] = value
        out[f'{prefix}_peak_t_s'] = tm[np.argmax(x)] if len(x) else np.nan
    out['frame_span_max_ms'] = np.max(span[mask]) if mask.any() else np.nan
    return out
