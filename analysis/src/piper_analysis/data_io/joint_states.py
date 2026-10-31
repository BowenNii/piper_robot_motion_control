"""Project2状态CSV适配，拒绝未确认的力矩单位/传动侧。"""
from __future__ import annotations
from pathlib import Path
from ..common.records import JointStateSeries
from ..validation.timing import strict_time
from .csv import read_numeric, columns

def load_joint_states(source: Path, measured_torque_verified: bool = False) -> JointStateSeries:
    names = ['time_s']+[f'{prefix}{i}_{unit}' for prefix, unit in
             [('q', 'rad'), ('dq', 'rad_s'), ('ddq', 'rad_s2')] for i in range(1, 7)]
    d = read_numeric(source, names)
    t = strict_time(d['time_s'])
    q, dq, ddq = [columns(d, [f'{prefix}{i}_{unit}' for i in range(1, 7)]) for prefix, unit in
                  [('q', 'rad'), ('dq', 'rad_s'), ('ddq', 'rad_s2')]]
    torque_names = [f'tau{i}_Nm' for i in range(1, 7)]
    present = set(torque_names) & set(d)
    if present and len(present) != 6:
        raise ValueError('Partial measured torque columns are not allowed')
    if present and not measured_torque_verified:
        raise ValueError('Verify torque units/sign/transmission side before passing --measured-torque-verified')
    measured = columns(d, torque_names) if present else None
    return JointStateSeries(source, t, q, dq, ddq, measured)
