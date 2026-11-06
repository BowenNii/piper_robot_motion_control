"""显式离线分析配置，不是硬件参数。"""
from __future__ import annotations
import json
from pathlib import Path
import numpy as np

def load_config(path: str | Path) -> dict[str, float | int | None]:
    """读取并校验离线分析容差配置，返回三个容差字段组成的字典。
    Args:
        path: JSON 配置文件路径（字符串或 Path）
    Returns:
        包含以下三个键的字典，可通过 cfg['字段名'] 读取：
        - joint_tolerance_deg：关节角误差容差，单位 °。
        - position_tolerance_mm：位置误差容差，单位 mm。
        - orientation_tolerance_deg：姿态角误差容差，单位 °。
    Raises:
        ValueError: JSON 格式错误，或配置字段、版本、用途、容差值不合法。
        OSError: 配置文件不存在、不可读或发生其他文件读取错误。
    """
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
