"""事后延迟估计与实际发布ZOH参考误差。不得替代实时原始误差验收。"""
import numpy as np

from .pose import pose_error
from .timing import zoh_indices


def fit_delay(t, q, command_time, command_q, active, duration) -> tuple[float, bool]:
    """估计实际关节运动相对于指令滞后了多少秒"""
    fit = (t >= .2 * duration) & (t <= .8 * duration)
    candidates = np.arange(301) / 1000#创建301 个候选延迟，每隔 1 ms 尝试一次
    scores = []
    for lag in candidates:#对每个延迟，根据时间插值生成对应的参考关节角
        reference = np.column_stack([
            np.interp(t[fit] - lag, command_time, command_q[:, joint], left=np.nan, right=np.nan)
            for joint in active
        ])
        scores.append(np.mean((q[fit][:, active] - reference) ** 2))#计算并保存误差分数
    if not np.isfinite(scores).all():#检查所有分数是否有效
        raise ValueError('Delay fit outside reference coverage')
    best = int(np.argmin(scores))#找到最小分数的下标
    return candidates[best], best == len(candidates) - 1


def tracking_errors(t, q, poses, command_time, command_q, command_poses, lag=0.):
    """返回有符号关节误差deg、位置范数mm、姿态角deg，不作空间配准。"""
    query = np.maximum(command_time[0], t - lag)
    indices = zoh_indices(command_time, query)
    joint_error = np.rad2deg(q - command_q[indices])
    position, rotation = pose_error(poses, command_poses[indices])
    return joint_error, position * 1000, np.rad2deg(rotation)
