"""十个静态位形独立统计，位形编号不是时间轴。"""
from __future__ import annotations
import numpy as np
from pathlib import Path
from ...common.paths import REPO
from ...robot_model.types import RobotModel
from ...validation.statistics import summarize_masked as _summary
from ...data_io.csv import columns, write_rows
from ...data_io.feedback import feedback
from ...data_io.provenance import sha256
from .baseline import _compare
from ...plotting.history import plot_static_pose, plot_static_overview

def analyze_static(folder: Path, model: RobotModel, hashes: dict) -> None:
    root = REPO / 'experiments/project1'
    folder.mkdir()
    rows = []
    static = root/'2026-10-03_Multibody_static _fk'
    for file in sorted((static/'data_csv').glob('pose*.csv')):
        d, q, T = feedback(file, strict=False)
        hashes[str(file.relative_to(REPO))] = sha256(file)
        p = d['position_error_m'] * 1000
        r = np.rad2deg(d['rotation_error_rad'])
        span = d['frame_span_s'] * 1000
        row = dict(pose=file.stem, sample_count=len(q), duration_s=d['t_s'][-1]-d['t_s'][0])
        for j in range(6):
            row[f'q{j + 1}_mean_deg'] = np.mean(np.rad2deg(q[:, j]))
            row[f'q{j + 1}_range_deg'] = np.ptp(np.rad2deg(q[:, j]))
        pf = columns(d, [f'feedback_{a}_m' for a in 'xyz']) * 1000
        pc = T[:, :3, 3] * 1000
        for j, a in enumerate('xyz'):
            row[f'feedback_{a}_mean_mm'] = pf[:, j].mean()
            row[f'poe_{a}_mean_mm'] = pc[:, j].mean()
            row[f'delta_{a}_mean_mm'] = pc[:, j].mean()-pf[:, j].mean()
        row.update({k: v for k, v in _summary(d['t_s'], p, r, span, np.ones(len(q), bool)).items() if not k.endswith('peak_t_s')})
        row.update(span_over_window_count=int(np.sum(span > 2+1e-6)),
                   position_recheck_max_mm=np.max(abs(np.linalg.norm(pc-pf, axis=1)-p)))
        rows.append(row)
        plot_static_pose(d, q, p, r, folder, file)
    write_rows(folder / 'summary.csv', rows)
    _compare(rows, static / 'summary.csv', folder / 'matlab_comparison.csv')
    plot_static_overview(rows, folder)
