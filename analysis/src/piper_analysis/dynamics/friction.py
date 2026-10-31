"""平滑黏滞+库仑摩擦；参数需独立辨识。"""
from __future__ import annotations
import numpy as np
from ..common.validation import finite

def friction(dq, viscous, coulomb, velocity_scale):
    dq = finite(dq)
    n = dq.shape
    b, fc, vs = [finite(v, n) for v in (viscous, coulomb, velocity_scale)]
    if dq.ndim != 1 or np.any(b < 0) or np.any(fc < 0) or np.any(vs <= 0):
        raise ValueError("Require nonnegative friction and positive velocity scale")
    return b*dq + fc*np.tanh(dq/vs)
