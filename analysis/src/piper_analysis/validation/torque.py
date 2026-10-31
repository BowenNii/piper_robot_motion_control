"""标称模型与已确认关节力矩的残差指标，不将残差解释为外力测量。"""
from .statistics import trace_metrics


def torque_residuals(time_s, measured, predicted):
    if measured is None:
        return None, []
    residual = measured - predicted
    rows = []
    for joint in range(residual.shape[1]):
        metrics = trace_metrics(time_s, abs(residual[:, joint]))
        rows.append(dict(joint=joint + 1, residual_rms_Nm=metrics['rms'], residual_peak_Nm=metrics['peak']))
    return residual, rows
