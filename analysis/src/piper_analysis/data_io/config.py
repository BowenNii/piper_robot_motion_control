"""显式离线分析配置，不是硬件参数。"""
from __future__ import annotations
import json
from pathlib import Path
import numpy as np

def load_config(path):
    cfg = json.loads(Path(path).read_text())
    keys = {'joint_tolerance_deg', 'position_tolerance_mm', 'orientation_tolerance_deg'}
    if set(cfg)-keys-{'schema_version', 'scope', 'description'}:
        raise ValueError("Unknown analysis config fields")
    if cfg.get('schema_version') != 1 or cfg.get('scope') != 'offline_analysis_only':
        raise ValueError("Not an offline analysis configuration v1")
    for key in keys:
        value = cfg.get(key)
        if value is not None and (isinstance(value, bool) or not isinstance(value, (float, int))
                                  or not np.isfinite(value) or value <= 0):
            raise ValueError(f"Invalid {key}")
    return {key: cfg.get(key) for key in keys}
