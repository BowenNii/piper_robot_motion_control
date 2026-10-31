"""合成状态的离线演示流程；不是轨迹发送器或真机实验。"""
import json
from pathlib import Path

import numpy as np

from ...data_io.provenance import sha256
from ...dynamics.batch import torque_prediction
from ...dynamics.crba import crba
from ...plotting.torque import plot_decomposition
from ...robot_model.urdf import load_piper


def run_demo(output: Path) -> Path:
    if output.exists():
        raise FileExistsError(f'Refusing to overwrite {output}')
    model = load_piper()
    time_s = np.linspace(0, 8, 201)
    omega = 2 * np.pi / 8
    center = np.array([0, .4, -.6, 0, 0, 0])
    amplitude = np.array([.1, .1, .1, .1, .05, .1])
    q = center + np.sin(omega * time_s[:, None]) * amplitude
    dq = omega * np.cos(omega * time_s[:, None]) * amplitude
    ddq = -omega ** 2 * np.sin(omega * time_s[:, None]) * amplitude
    prediction = torque_prediction(q, dq, ddq, model)
    components = prediction['gravity'] + prediction['velocity_bias'] + prediction['inertial']
    if not np.allclose(prediction['total'], components, atol=1e-10):
        raise RuntimeError('Dynamics decomposition failed')
    eigenvalues = [np.linalg.eigvalsh(crba(qi, model)).min() for qi in q]
    output.mkdir(parents=True)
    names = ['time_s']
    arrays = [time_s[:, None]]
    fields = [
        ('q', 'rad', q), ('dq', 'rad_s', dq), ('ddq', 'rad_s2', ddq),
        ('model_tau', 'Nm', prediction['total']), ('gravity', 'Nm', prediction['gravity']),
        ('velocity_bias', 'Nm', prediction['velocity_bias']), ('inertia', 'Nm', prediction['inertial']),
    ]
    for prefix, unit, values in fields:
        names.extend(f'{prefix}{joint}_{unit}' for joint in range(1, 7))
        arrays.append(values)
    np.savetxt(output / 'nominal_dynamics.csv', np.column_stack(arrays), delimiter=',',
               header=','.join(names), comments='')
    plot_decomposition(time_s, prediction, output)
    summary = dict(
        simulation_only=True, hardware_validated=False, samples=len(time_s),
        min_mass_matrix_eigenvalue=float(min(eigenvalues)), urdf_sha256=sha256(model.source),
        note='No payload/friction/rotor inertia. No real data identification. No commands sent.',
    )
    (output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return output
