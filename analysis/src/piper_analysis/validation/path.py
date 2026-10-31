"""点到空间折线距离；应双向评价，不能替代时间跟踪误差。"""
from __future__ import annotations
import numpy as np
from ..common.validation import finite

def path_distance(points, path):
    points, path = finite(points), finite(path)
    if points.ndim != 2 or points.shape[1] != 3 or path.ndim != 2 or path.shape[1] != 3 or len(path) < 2:
        raise ValueError("Expected (N,3) points and >=2 path vertices")
    distance = np.full(len(points), np.inf)
    for a, b in zip(path[:-1], path[1:]):
        d = b - a
        den = d @ d
        u = np.zeros(len(points)) if den < np.finfo(float).eps else np.clip((points-a)@d/den, 0, 1)
        distance = np.minimum(distance, np.linalg.norm(points-(a+u[:, None]*d), axis=1))
    return distance
