"""显式刚性负载合并，不猜测工具参数。"""
from __future__ import annotations
import numpy as np
from dataclasses import replace
from ..common.validation import finite
from ..lie_group.transforms import adjoint, inverse
from ..dynamics.spatial_inertia import spatial_inertia

def with_payload(model, mass, I_com, com, T_link_payload):
    """显式合并刚性负载到末link；不猜夹爪参数，不改工具参考点。

    com/I_com在payload轴表达，T_link_payload为末link到payload坐标变换。
    返回新模型，原模型不变。连接点/质量/惯量需要实测或可信标称值。
    """
    T = finite(T_link_payload, (4, 4))
    if (not np.allclose(T[3], [0, 0, 0, 1], atol=1e-12)
            or not np.allclose(T[:3, :3].T@T[:3, :3], np.eye(3), atol=1e-10)
            or not np.isclose(np.linalg.det(T[:3, :3]), 1, atol=1e-10)):
        raise ValueError('Payload transform must be SE(3)')
    X = adjoint(inverse(T))
    G = model.Glist.copy()
    G[-1] += X.T @ spatial_inertia(mass, I_com, com) @ X
    return replace(model, Glist=G, source=model.source+' + explicit payload')
