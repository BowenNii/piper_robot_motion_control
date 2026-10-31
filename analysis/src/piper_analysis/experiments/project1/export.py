"""Project1结果序列化，不计算误差或选择延迟。"""
from __future__ import annotations
import numpy as np
from pathlib import Path
import json
from ...common.paths import REPO
from ...data_io.csv import write_rows
from ...data_io.provenance import sha256

def export_trace(z: dict, output: Path) -> None:
    record = z['record']
    je, pe, re = z['jerror'], z['pe'], z['re']
    values = np.column_stack([z['tr'], je, pe, re, z['path_forward']])
    names = ['time_s']+[f'joint{i}_error_deg' for i in range(1, 7)]+[
        'model_position_error_mm', 'model_rotation_error_deg', 'path_distance_mm']
    np.savetxt(output/'traces'/f"{record['record']:02d}_stream_errors.csv", values, delimiter=',',
               header=','.join(names), comments='', fmt='%.17g')
    np.savez_compressed(output/'traces'/f"{record['record']:02d}_plot_data.npz",
                        **{key: z[key] for key in ('tr', 'qtrace', 'ptrace', 'refP', 'ct', 'qc', 'aligned_t', 'jaligned', 'paligned', 'raligned')})


def export_tables(cache: list[dict], response: list[dict], output: Path) -> None:
    write_rows(output / 'run_metrics.csv', [z['row'] for z in cache])
    write_rows(output / 'response_metrics.csv', response)


def export_provenance(output: Path, sources: dict, cfg: dict, model) -> None:
    code = {str(path.relative_to(REPO)): sha256(path)
            for path in sorted((REPO / 'analysis').rglob('*.py'))}
    provenance = dict(
        offline_only=True, source_hashes=sources, pipeline_source_hashes=code, config=cfg,
        urdf_sha256=sha256(model.source), model_endpoint='link6, no attached payload',
        note='Feedback-angle FK is not external TCP measurement. Delay alignment is post hoc.',
    )
    (output / 'provenance.json').write_text(json.dumps(provenance, indent=2, allow_nan=False) + '\n')
