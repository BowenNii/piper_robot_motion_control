"""质量、质心和质心惯量到空间惯量。"""
from __future__ import annotations
import numpy as np
from ..common.validation import finite
from ..lie_group.transforms import skew

def spatial_inertia(mass, I_com, com):
    """质量kg、质心惯量kg m²、质心m；惯量需在link轴表达。"""
    I, c = finite(I_com, (3, 3)), finite(com, (3,))
    if not np.isfinite(mass) or mass < 0 or not np.allclose(I, I.T, atol=1e-12):
        raise ValueError("Invalid mass or nonsymmetric inertia")
    eig = np.linalg.eigvalsh(I)
    if eig[0] < -1e-12 or eig[-1] > eig[:2].sum() + 1e-10:
        raise ValueError("Nonphysical COM inertia")
    C = skew(c)
    return np.block([[I+mass*C@C.T, mass*C], [mass*C.T, mass*np.eye(3)]])
