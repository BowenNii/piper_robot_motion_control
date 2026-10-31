"""离线IK的回代、差分与奇异值核对；不重新求解或下发。"""
from __future__ import annotations
import numpy as np
from ...common.paths import REPO
from ...kinematics.forward import forward, fk_sequence
from ...validation.pose import pose_error
from ...data_io.csv import read_numeric, columns, write_rows
from ...data_io.provenance import sha256
from .baseline import _compare
from ...lie_group.transforms import inverse, exp_twist, log_transform
from ...kinematics.jacobian import jacobian_body
from ...validation.conditioning import weighted_sigma_min
from ...signals.derivatives import sample_derivatives
from ...data_io.poses import pose_columns as _pose
from ...plotting.history import plot_ik

def analyze_ik(folder, model, hashes):
    folder.mkdir()
    file = REPO / 'experiments/project1/2026-10-18_ik_trajectory/data_csv/trajectory.csv'
    hashes[str(file.relative_to(REPO))] = sha256(file)
    d = read_numeric(file)
    q = columns(d, [f'q{i}_rad' for i in range(1, 7)])
    t = d['t_s']
    if not all(np.isfinite(value).all() for value in d.values()):
        raise ValueError('IK CSV contains nonfinite values')
    if not np.array_equal(d['index'], np.arange(len(q))) or np.any(np.diff(t) <= 0):
        raise ValueError('IK sample indices/time are invalid')
    if np.any(q < model.q_min) or np.any(q > model.q_max):
        raise ValueError('IK trajectory exceeds URDF limits')
    fixed = ['position_tolerance_m', 'orientation_tolerance_rad', 'max_sample_step_rad',
             'rotation_weight', 'dls_lambda', 'ik_max_joint_step_rad', 'max_iterations',
             'sample_count', 'duration_s', 'sample_period_s',
             'offline_velocity_limit_rad_s', 'offline_acceleration_limit_rad_s2']
    fixed += [f'{prefix}{i}_rad' for prefix in ['q_min', 'q_max', 'q_start', 'q_goal'] for i in range(1, 7)]
    if any(not np.all(d[key] == d[key][0]) for key in fixed):
        raise ValueError('IK configuration changed within CSV')
    if not np.allclose([d[f'q_min{i}_rad'][0] for i in range(1, 7)], model.q_min, atol=1e-12, rtol=0) or not np.allclose([d[f'q_max{i}_rad'][0] for i in range(1, 7)], model.q_max, atol=1e-12, rtol=0):
        raise ValueError('IK CSV/URDF limits mismatch')
    duration = d['duration_s'][0]
    u = t / duration
    if (not np.allclose(t, np.linspace(0, duration, len(t)), atol=1e-12, rtol=0)
            or not np.allclose(d['s'], 10*u**3-15*u**4+6*u**5, atol=1e-12, rtol=0)):
        raise ValueError('IK time scaling mismatch')
    target, cpp = (_pose(d, 'target'), _pose(d, 'poe'))
    actual = fk_sequence(model, q)
    p, r = pose_error(actual, target)
    cp, cr = pose_error(actual, cpp)
    qs = columns(d, [f'q_start{i}_rad' for i in range(1, 7)])[0]
    qg = columns(d, [f'q_goal{i}_rad' for i in range(1, 7)])[0]
    T0, T1 = forward(model, qs), forward(model, qg)
    if 'path_mode' in d and d['path_mode'][0] == 1:
        expected_q = qs+d['s'][:, None]*(qg-qs)
        if not np.allclose(q, expected_q, atol=1e-12, rtol=0):
            raise ValueError('Joint interpolation mismatch')
        expected = fk_sequence(model, expected_q)
    else:
        xi = log_transform(inverse(T0)@T1)
        expected = np.array([T0@exp_twist(xi*s) for s in d['s']])
    tp, tr = pose_error(expected, target)
    if tp.max() > 1e-9 or tr.max() > 1e-8 or cp.max() > 1e-9 or cr.max() > 1e-8:
        raise ValueError('Declared IK path/FK crosscheck mismatch')
    if np.any(p > d['position_tolerance_m']) or np.any(r > d['orientation_tolerance_rad']):
        raise ValueError('IK residual exceeds declared tolerance')
    delta = np.diff(np.vstack([columns(d, [f'q_start{i}_rad' for i in range(1, 7)])[0], q]), axis=0)
    if not np.allclose(delta, columns(d, [f'delta_q{i}_rad' for i in range(1, 7)]), atol=1e-12, rtol=0):
        raise ValueError('IK adjacent increment mismatch')
    dq, ddq = sample_derivatives(q, d['sample_period_s'][0])
    if abs(delta).max() > d['max_sample_step_rad'][0] or abs(dq).max() > d['offline_velocity_limit_rad_s'][0] or abs(ddq).max() > d['offline_acceleration_limit_rad_s2'][0]:
        raise ValueError('IK continuity/derivatives exceed declared limits')
    if not np.allclose(dq, columns(d, [f'dq{i}_rad_s' for i in range(1, 7)]), atol=1e-10, rtol=0):
        raise ValueError('IK derivative mismatch')
    if not np.allclose(ddq, columns(d, [f'ddq{i}_rad_s2' for i in range(1, 7)]), atol=1e-8, rtol=0):
        raise ValueError('IK acceleration mismatch')
    sigma = []
    for qi, Ti in zip(q, actual):
        J = jacobian_body(model, qi, pose=Ti)
        sigma.append(weighted_sigma_min(J, d['rotation_weight'][0]))
    if not np.allclose(sigma, d['weighted_sigma_min'], atol=1e-10, rtol=0):
        raise ValueError('IK weighted singular values mismatch')
    row = dict(samples=len(q), position_mean_m=p.mean(), position_rms_m=np.sqrt(np.mean(p*p)), position_max_m=p.max(),
               rotation_mean_rad=r.mean(), rotation_rms_rad=np.sqrt(np.mean(r*r)), rotation_max_rad=r.max(),
               max_abs_delta_q_rad=abs(delta).max(), min_weighted_sigma=min(sigma), total_iterations=d['iterations'].sum(),
               max_iterations_per_sample=d['iterations'].max(), cross_position_max_m=cp.max(), cross_rotation_max_rad=cr.max(),
               max_velocity_rad_s=abs(dq).max(), max_acceleration_rad_s2=abs(ddq).max(),
               endpoint_velocity_max_rad_s=abs(dq[[0, -1]]).max(), endpoint_acceleration_max_rad_s2=abs(ddq[[0, -1]]).max())
    for key in ['position_tolerance_m', 'orientation_tolerance_rad', 'max_sample_step_rad', 'duration_s', 'sample_period_s']:
        row[key] = d[key][0]
    write_rows(folder/'summary.csv', [row])
    _compare([row], file.parents[1]/'analysis/summary.csv', folder/'matlab_comparison.csv')
    checks = np.column_stack([d['index'], d['s'], p, r, abs(delta).max(axis=1), sigma, d['iterations'], cp, cr, t])
    np.savetxt(folder/'point_checks.csv', checks, delimiter=',', header='index,s,position_error_m,rotation_error_rad,max_abs_delta_q_rad,weighted_sigma_min,iterations,cross_position_m,cross_rotation_rad,t_s', comments='')
    plot_ik(t, q, p, r, cp, cr, sigma, d, dq, ddq, folder)
