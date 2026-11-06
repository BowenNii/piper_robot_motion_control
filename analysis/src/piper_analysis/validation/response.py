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
        #当前位置与目标位置的误差，不超过总行程的 1%，目标带区域
        in_band = abs(q[mask, joint] - q_goal[joint]) <= .01 * abs(q_goal[joint] - q_start[joint])
        bad = np.flatnonzero(~in_band)#计算稳定时间，bad[-1] 就是最后一个带外点的下标
        settled = relative_time[0] if len(bad) == 0 else np.nan
        if (#检查是否满足稳定条件——最后一个带外点之后，持续在带内直到记录结束，且至少有 0.5 秒。
            len(bad) and bad[-1] + 1 < len(relative_time)
            and relative_time[-1] - relative_time[bad[-1] + 1] >= .5
        ):
            settled = relative_time[bad[-1] + 1]#确定稳定时间点，必须在最后一个带外点之后，且至少持续 0.5s
        rows.append(dict(
            joint=int(joint + 1),#关节编号
            onset_1pct_s=first_time(relative_time, progress >= .01, .02),#达到1%行程并持续至少20ms的起动时间
            arrival_99pct_s=first_time(relative_time, progress >= .99),#首次达到99%行程的时间
            settled_1pct_band_s=settled,#进入目标±1%行程带并保持至少0.5秒至记录末尾的稳定时间
        ))
    return rows
