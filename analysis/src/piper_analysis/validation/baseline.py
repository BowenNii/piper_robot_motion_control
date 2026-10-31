"""纯数值/文本表格对照；不读取文件，不写报告。"""
import numpy as np


def compare_tables(reference: list[dict], actual: list[dict], ignored_fields=()) -> list[dict]:
    """按原分析容差逐字段核对；NaN只与NaN相等，文本与行数必须一致。"""
    if len(reference) != len(actual) or not reference:
        raise ValueError('Baseline row count mismatch or empty baseline')
    checks = []
    for field in reference[0]:
        if field in ignored_fields:
            continue
        try:
            left = np.array([float(row[field]) for row in reference])
            right = np.array([float(row[field]) for row in actual])
        except ValueError:
            if [row[field] for row in reference] != [row[field] for row in actual]:
                raise ValueError(f'Baseline label mismatch: {field}')
            continue
        valid = np.isfinite(left) & np.isfinite(right)
        delta = float(np.max(abs(left[valid] - right[valid]))) if valid.any() else np.nan
        checks.append(dict(
            field=field, max_abs_difference=delta,
            passed=bool(np.allclose(left, right, rtol=1e-7, atol=1e-8, equal_nan=True)),
        ))
    return checks
