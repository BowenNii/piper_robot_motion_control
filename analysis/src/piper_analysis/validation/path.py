"""点到空间折线距离；应双向评价，不能替代时间跟踪误差。"""
from __future__ import annotations
import numpy as np
from ..common.validation import finite

def path_distance(points, path):
    """衡量参考路径和实际反馈路径在空间上的偏差距离"""
    points, path = finite(points), finite(path)
    if points.ndim != 2 or points.shape[1] != 3 or path.ndim != 2 or path.shape[1] != 3 or len(path) < 2:
        raise ValueError("Expected (N,3) points and >=2 path vertices")
    distance = np.full(len(points), np.inf)#初始化距离,有多少个待测点，就建立多少个距离都为无穷大
    for a, b in zip(path[:-1], path[1:]):#依次取出路径的每条线段，从开头取到最后一个元素之前；下标 1（第二个元素）取到末尾
        d = b - a
        den = d @ d
        #u:找到每个待测点在线段上的最近位置
        u = np.zeros(len(points)) if den < np.finfo(float).eps else np.clip((points-a)@d/den, 0, 1)
        #distance:计算距离，并保留目前最小值
        distance = np.minimum(distance, np.linalg.norm(points-(a+u[:, None]*d), axis=1))
    return distance
