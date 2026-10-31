"""描述性响应指标。1%行程不是任务验收容差，也不等于固件通信延迟。"""
import numpy as np

from .timing import first_time


def joint_responses(t, q, command_start, q_start, q_goal, active) -> list[dict]:
    """起动需持续20ms；99%首次到达；终态1%行程带内至少持续0.5s至记录末尾。"""
    mask = t >= command_start
    relative_time = t[mask] - command_start
    rows = []
    for joint in active:
        progress = (q[mask, joint] - q_start[joint]) / (q_goal[joint] - q_start[joint])
        in_band = abs(q[mask, joint] - q_goal[joint]) <= .01 * abs(q_goal[joint] - q_start[joint])
        bad = np.flatnonzero(~in_band)
        settled = relative_time[0] if len(bad) == 0 else np.nan
        if (
            len(bad) and bad[-1] + 1 < len(relative_time)
            and relative_time[-1] - relative_time[bad[-1] + 1] >= .5
        ):
            settled = relative_time[bad[-1] + 1]
        rows.append(dict(
            joint=int(joint + 1),
            onset_1pct_s=first_time(relative_time, progress >= .01, .02),
            arrival_99pct_s=first_time(relative_time, progress >= .99),
            settled_1pct_band_s=settled,
        ))
    return rows
