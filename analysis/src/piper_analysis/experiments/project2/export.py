"""Project2结果导出与来源记录。"""
from __future__ import annotations
import numpy as np
import json
from ...data_io.csv import write_rows
from ...data_io.provenance import sha256

def export_predictions(t, prediction, residual, residual_metrics, output) -> None:
    fields = ['time_s']
    arrays = [t[:, None]]
    for key, values in prediction.items():
        fields.extend([f'{key}{j}_Nm' for j in range(1, 7)])
        arrays.append(values)
    if residual is not None:
        fields.extend([f'residual{j}_Nm' for j in range(1, 7)])
        arrays.append(residual)
        write_rows(output/'residual_metrics.csv', residual_metrics)
    np.savetxt(output/'predictions.csv', np.column_stack(arrays), delimiter=',', header=','.join(fields), comments='')

def export_provenance(source, output, model, measured) -> None:
    (output/'provenance.json').write_text(json.dumps(dict(offline_only=True, input_sha256=sha256(source),
        urdf_sha256=sha256(model.source), measured_torque_verified=measured is not None,
        filtering='none; input dq/ddq must be documented', payload='not included', friction='not included'), indent=2)+'\n')
