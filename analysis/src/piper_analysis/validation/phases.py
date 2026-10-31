"""反馈变化的启发式分段，不等于固件轨迹阶段或控制状态。"""
import numpy as np


def split_motion_phases(t, q_deg, change_threshold_deg=.0005, padding_s=.05, activity_gap_s=.2):
    changed = np.flatnonzero(np.any(abs(np.diff(q_deg, axis=0)) > change_threshold_deg, axis=1))
    phase = np.ones(len(t), int)
    bounds = [np.nan, np.nan]
    segments = 0
    if len(changed):
        bounds = [t[changed[0]], t[changed[-1] + 1]]
        segments = 1 + np.sum(np.diff(t[changed]) > activity_gap_s)
        phase[(t >= bounds[0] - padding_s) & (t <= bounds[1] + padding_s)] = 2
        phase[t > bounds[1] + padding_s] = 3
    return phase, bounds, int(segments)
