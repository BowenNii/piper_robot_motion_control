"""双向零相位离线滤波，使用未来数据，不用于实时控制。"""
from __future__ import annotations
import numpy as np
from ..common.validation import finite
from scipy.signal import butter, sosfiltfilt

def lowpass_offline(values, sample_rate, cutoff, order=4):
    """双向零相位离线滤波，使用未来数据，不能用于实时控制。"""
    values = finite(values)
    if (not np.isfinite(sample_rate) or not np.isfinite(cutoff) or not 0 < cutoff < sample_rate/2
            or isinstance(order, bool) or not isinstance(order, int) or order < 1):
        raise ValueError('Invalid cutoff/rate/order')
    return sosfiltfilt(butter(order, cutoff, fs=sample_rate, output='sos'), values, axis=0)
