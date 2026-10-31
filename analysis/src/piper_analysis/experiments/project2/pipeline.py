"""Project2离线流程：读取→模型预测→导出→绘图，不控制硬件。"""
from __future__ import annotations
import numpy as np
from pathlib import Path
from ...robot_model.urdf import load_piper
from ...data_io.joint_states import load_joint_states
from ...dynamics.batch import torque_prediction
from ...plotting.torque import plot_prediction
from ...validation.torque import torque_residuals
from .export import export_predictions, export_provenance


def analyze_csv(source, output, measured_torque_verified: bool = False):
    source, output = Path(source), Path(output)
    if output.exists():
        raise FileExistsError(f'Refusing to overwrite: {output}')
    states = load_joint_states(source, measured_torque_verified)
    model = load_piper()
    prediction = torque_prediction(states.q_rad, states.dq_rad_s, states.ddq_rad_s2, model)
    if np.any(states.q_rad < model.q_min) or np.any(states.q_rad > model.q_max):
        raise ValueError('Joint limits exceeded; investigate input/model rather than clipping')
    output.mkdir(parents=True)
    residual, metrics = torque_residuals(states.time_s, states.measured_torque_Nm, prediction['total'])
    export_predictions(states.time_s, prediction, residual, metrics, output)
    plot_prediction(states.time_s, prediction, states.measured_torque_Nm, output)
    export_provenance(source, output, model, states.measured_torque_Nm)
    return output
