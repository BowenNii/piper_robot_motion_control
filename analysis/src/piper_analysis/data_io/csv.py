"""CSV读取、数字列选择和独占写入，不覆盖已有文件。"""
from __future__ import annotations
import csv
from pathlib import Path
import numpy as np
from ..common.validation import finite

def read_numeric(path, required=()):
#读取 CSV 文件，按列组织成字典，尽可能把数据转换成数字，返回该列数据数组的字典
    with Path(path).open(newline='', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames or len(set(reader.fieldnames)) != len(reader.fieldnames):
            raise ValueError(f"Invalid CSV header: {path}")
        names = reader.fieldnames
        rows = list(reader)
    missing = set(required)-set(names)
    if missing or not rows:
        raise ValueError(f"Missing columns {missing} or empty data: {path}")
    data = {}
    for name in names:
        try:
            data[name] = np.array([float(row[name]) for row in rows])
        except (ValueError, TypeError):
            # 非数字文本列保留，不影响指定数字列；必需列另作严格检查。
            data[name] = np.array([row[name] for row in rows])
    for name in required:
        finite(data[name])
    return data

def columns(data, names):
    return finite(np.column_stack([data[name] for name in names]))

def write_rows(path, rows):
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with Path(path).open('x', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=fields, restval='NaN')
        w.writeheader()
        w.writerows(rows)
