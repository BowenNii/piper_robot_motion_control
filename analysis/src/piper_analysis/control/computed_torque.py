"""计算力矩律，kp/kd为加速度反馈增益，尚无真机安全适配。"""
from __future__ import annotations
import numpy as np
from ..common.validation import finite
from ..dynamics.rnea import rnea

def computed_torque(q_des, dq_des, ddq_des, q, dq, kp, kd, model):
    """理想计算力矩律；kp/kd为加速度反馈增益。仅数值，未限幅/真机验证。"""
    n = model.n
    qd, vd, ad, q, dq, kp, kd = [finite(v, (n,)) for v in (q_des, dq_des, ddq_des, q, dq, kp, kd)]
    if np.any(kp < 0) or np.any(kd < 0):
        raise ValueError("Feedback gains must be nonnegative")
    return rnea(q, dq, ad+kp*(qd-q)+kd*(vd-dq), model)
