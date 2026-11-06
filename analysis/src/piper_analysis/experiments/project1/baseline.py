"""把MATLAB基准适配成表格，并导出对照报告；数学比较在validation。"""
import csv
from pathlib import Path

from ...data_io.csv import write_rows
from ...validation.baseline import compare_tables


def _read_rows(path: Path) -> list[dict]:
    with path.open() as stream:
        return list(csv.DictReader(stream))


def compare_baseline(output: Path, baseline: Path) -> None:
    #把MATLAB基准适配成表格，并导出对照报告
    checks = []
    for filename in ('run_metrics.csv', 'response_metrics.csv'):
        values = compare_tables(_read_rows(baseline / filename), _read_rows(output / filename))
        checks.extend(dict(table=filename, **value) for value in values)
    write_rows(output / 'matlab_comparison.csv', checks)
    if not all(row['passed'] for row in checks):
        raise ValueError('MATLAB parity failed; inspect matlab_comparison.csv')


def _compare(rows: list[dict], baseline: Path, output: Path) -> None:
    """历史近常数峰值时刻对舍入敏感，保留原来的排除规则。"""
    reference = _read_rows(baseline)
    ignored = [key for key in reference[0] if key.endswith('peak_t_s')]
    checks = compare_tables(reference, rows, ignored_fields=ignored)
    write_rows(output, checks)
    if not all(row['passed'] for row in checks):
        raise ValueError(f'History parity failed: {output}')
