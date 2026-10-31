"""离线SG导数及历史轨迹差分；不是电机实测加速度。"""
from __future__ import annotations
import numpy as np
from ..common.validation import finite
from scipy.signal import savgol_filter
from ..validation.timing import strict_time

def estimate_derivatives(t, q, window=21, polynomial=3):
    """等间隔Savitzky–Golay导数；边界单独谨慎解释，非电机实测加速度。"""
    t = strict_time(t)
    q = finite(q)
    if len(q) != len(t) or not np.allclose(np.diff(t), np.mean(np.diff(t)), rtol=1e-5, atol=1e-10):
        raise ValueError('Resample to a uniform grid first')
    if window % 2 != 1 or not 2 <= polynomial < window or window > len(t):
        raise ValueError('Need odd window <= N and 2 <= polynomial < window')
    dt = np.mean(np.diff(t))
    return tuple(savgol_filter(q, window, polynomial, deriv=k, delta=dt, axis=0) for k in (1, 2))

def sample_derivatives(q, dt):
    """精确复现项目1 CSV的内点中心差分、端点4点单边差分。"""
    q = finite(q)
    if q.ndim != 2 or len(q) < 4 or not np.isfinite(dt) or dt <= 0:
        raise ValueError('Need >=4 samples and positive dt')
    v = np.zeros_like(q)
    a = v.copy()
    v[1:-1] = (q[2:]-q[:-2])/(2*dt)
    a[1:-1] = (q[2:]-2*q[1:-1]+q[:-2])/dt**2
    v[0] = (-11*q[0]+18*q[1]-9*q[2]+2*q[3])/(6*dt)
    v[-1] = (11*q[-1]-18*q[-2]+9*q[-3]-2*q[-4])/(6*dt)
    a[0] = (2*q[0]-5*q[1]+4*q[2]-q[3])/dt**2
    a[-1] = (2*q[-1]-5*q[-2]+4*q[-3]-q[-4])/dt**2
    return v, a
